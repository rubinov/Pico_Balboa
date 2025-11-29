#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h" 
#include "hardware/i2c.h" 
#include "encoders.pio.h" 
#include "pico/time.h"

// --- BLUETOOTH INCLUDES ---
#include "pico/cyw43_arch.h"
#include "btstack.h"

// --- PIN ASSIGNMENTS ---
#define L_X_PIN 0   
#define L_B_PIN 1   
#define R_X_PIN 26  
#define R_B_PIN 27  
#define ENABLE_PIN 20

// --- I2C1: SENSORS (400kHz) ---
#define I2C_SENS_PORT i2c1
#define I2C_SENS_SDA 18
#define I2C_SENS_SCL 19
#define LSM6DS3_ADDR 0x6B

// --- I2C0: MOTOR CONTROLLER (100kHz) ---
#define I2C_MOT_PORT i2c0
#define I2C_MOT_SDA 16
#define I2C_MOT_SCL 17
#define BALBOA_ADDR 0x42
#define CMD_ACTIVATE_MOTORS 0x81

// --- LSM6DS3 REGISTERS ---
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL10_C    0x19
#define OUTX_L_G    0x22

// --- CONFIG VALUES ---
#define NORMAL_MODE_208HZ 0x50
#define RESET_STEPS       0x02
#define SET_FUNC_EN       0xBD

// --- DMA BUFFER CONFIGURATION ---
#define DMA_BUFFER_SIZE 1024 
#define DMA_BUFFER_MASK (DMA_BUFFER_SIZE - 1) 

// --- RECORDING CONFIGURATION ---
#define MAX_RECORD_SAMPLES 5000

// --- SEQUENCE CONFIGURATION ---
#define MAX_STEPS 50

typedef struct {
    uint32_t timestamp_us;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int32_t left_enc, right_enc;
    int16_t motor_cmd; 
} DataPoint;

typedef struct {
    uint32_t duration_ms;
    int16_t left_speed;
    int16_t right_speed;
} Step;

DataPoint record_buffer[MAX_RECORD_SAMPLES];
uint32_t record_index = 0;

Step sequence[MAX_STEPS];
int sequence_length = 0;

// --- GLOBALS ---
PIO pio = pio0;
uint sm_left = 0;
uint sm_right = 1;

int dma_chan_left;
int dma_chan_right;

volatile int32_t left_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096)));
volatile int32_t right_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096)));

uint32_t left_read_index = 0;
uint32_t right_read_index = 0;

volatile int32_t left_total = 0;
volatile int32_t right_total = 0;
volatile int32_t left_errors = 0;
volatile int32_t right_errors = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

char usb_cmd_buffer[64]; // Increased size for STEP params
int usb_cmd_idx = 0;

// Bluetooth Globals
static uint16_t rfcomm_channel_id = 0; 
static char bt_cmd_buffer[64]; // Increased size for STEP params
static int bt_cmd_idx = 0;

// Flags for Main Loop execution
volatile bool start_run_request = false;
volatile bool start_read_request = false;
volatile bool start_dma_dump_request = false;
volatile bool start_show_seq_request = false;
volatile bool start_help_request = false;

// --- BLUETOOTH HELPER ---
void bt_printf(const char *format, ...) {
    if (rfcomm_channel_id == 0) return; 

    char buffer[128]; 
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        rfcomm_send(rfcomm_channel_id, (uint8_t*)buffer, len);
    }
}

// --- BLUETOOTH SPP SETUP ---
uint8_t spp_service_buffer[256];

void process_command(const char* cmd);

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    if (packet_type == HCI_EVENT_PACKET) {
        switch (hci_event_packet_get_type(packet)) {
            case BTSTACK_EVENT_STATE:
                if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
                break;
            case RFCOMM_EVENT_INCOMING_CONNECTION:
                rfcomm_accept_connection(rfcomm_event_incoming_connection_get_rfcomm_cid(packet));
                break;
            case RFCOMM_EVENT_CHANNEL_OPENED:
                if (rfcomm_event_channel_opened_get_status(packet)) {
                    printf("RFCOMM Connection failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
                } else {
                    rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                    printf("RFCOMM Connected, ID %d\n", rfcomm_channel_id);
                    bt_printf("CONNECTED\n");
                }
                break;
            case RFCOMM_EVENT_CHANNEL_CLOSED:
                printf("RFCOMM Disconnected\n");
                rfcomm_channel_id = 0;
                break;
            default:
                break;
        }
    } else if (packet_type == RFCOMM_DATA_PACKET) {
        for (int i = 0; i < size; i++) {
            char c = (char)packet[i];
            if (c == '\n' || c == '\r') {
                if (bt_cmd_idx > 0) {
                    bt_cmd_buffer[bt_cmd_idx] = 0; 
                    printf("BT CMD: %s\n", bt_cmd_buffer); 
                    process_command(bt_cmd_buffer);
                    bt_cmd_idx = 0;
                }
            } else {
                if (bt_cmd_idx < 63) {
                    bt_cmd_buffer[bt_cmd_idx++] = c;
                }
            }
        }
    }
}

// --- MOTOR FUNCTIONS ---
void balboa_set_speeds(int16_t left_speed, int16_t right_speed) {
    uint8_t packet[5];
    packet[0] = CMD_ACTIVATE_MOTORS;
    packet[1] = (uint8_t)(left_speed & 0xFF);
    packet[2] = (uint8_t)((left_speed >> 8) & 0xFF);
    packet[3] = (uint8_t)(right_speed & 0xFF);
    packet[4] = (uint8_t)((right_speed >> 8) & 0xFF);
    i2c_write_blocking(I2C_MOT_PORT, BALBOA_ADDR, packet, 5, false);
}

void init_motor_i2c() {
    i2c_init(I2C_MOT_PORT, 100 * 1000);
    gpio_set_function(I2C_MOT_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_MOT_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MOT_SDA);
    gpio_pull_up(I2C_MOT_SCL);
}

// --- LSM6DS3 FUNCTIONS ---
void lsm6ds3_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_write_blocking(I2C_SENS_PORT, LSM6DS3_ADDR, buf, 2, false);
}

void lsm6ds3_init() {
    i2c_init(I2C_SENS_PORT, 400 * 1000);
    gpio_set_function(I2C_SENS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SENS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENS_SDA);
    gpio_pull_up(I2C_SENS_SCL);
    sleep_ms(10); 
    lsm6ds3_write_reg(CTRL2_G, NORMAL_MODE_208HZ);
    lsm6ds3_write_reg(CTRL1_XL, NORMAL_MODE_208HZ);
    lsm6ds3_write_reg(CTRL10_C, RESET_STEPS);
    lsm6ds3_write_reg(CTRL10_C, SET_FUNC_EN);
}

void lsm6ds3_read_all() {
    uint8_t reg = OUTX_L_G;
    uint8_t buffer[12];
    i2c_write_blocking(I2C_SENS_PORT, LSM6DS3_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_SENS_PORT, LSM6DS3_ADDR, buffer, 12, false);
    gx = (int16_t)((buffer[1] << 8) | buffer[0]);
    gy = (int16_t)((buffer[3] << 8) | buffer[2]);
    gz = (int16_t)((buffer[5] << 8) | buffer[4]);
    ax = (int16_t)((buffer[7] << 8) | buffer[6]);
    ay = (int16_t)((buffer[9] << 8) | buffer[8]);
    az = (int16_t)((buffer[11] << 8) | buffer[10]);
}

// --- ENCODER FUNCTIONS ---
void setup_dma_encoders() {
    dma_chan_left = dma_claim_unused_channel(true);
    dma_channel_config c_left = dma_channel_get_default_config(dma_chan_left);
    channel_config_set_transfer_data_size(&c_left, DMA_SIZE_32); 
    channel_config_set_read_increment(&c_left, false); 
    channel_config_set_write_increment(&c_left, true); 
    channel_config_set_dreq(&c_left, pio_get_dreq(pio, sm_left, false)); 
    channel_config_set_ring(&c_left, true, 12); 
    dma_channel_configure(dma_chan_left, &c_left, left_dma_buffer, &pio->rxf[sm_left], 0xFFFFFFFF, true);

    dma_chan_right = dma_claim_unused_channel(true);
    dma_channel_config c_right = dma_channel_get_default_config(dma_chan_right);
    channel_config_set_transfer_data_size(&c_right, DMA_SIZE_32);
    channel_config_set_read_increment(&c_right, false);
    channel_config_set_write_increment(&c_right, true);
    channel_config_set_dreq(&c_right, pio_get_dreq(pio, sm_right, false));
    channel_config_set_ring(&c_right, true, 12); 
    dma_channel_configure(dma_chan_right, &c_right, right_dma_buffer, &pio->rxf[sm_right], 0xFFFFFFFF, true);
}

void update_counts() {
    uint32_t l_curr_addr = (uint32_t)dma_hw->ch[dma_chan_left].write_addr;
    uint32_t l_write_idx = (l_curr_addr - (uint32_t)left_dma_buffer) / 4;
    l_write_idx &= DMA_BUFFER_MASK;

    while (left_read_index != l_write_idx) {
        int32_t val = left_dma_buffer[left_read_index] & 0x3; 
        if (val == 0) left_total++;
        else if (val == 2) left_total--;
        else left_errors++;
        left_read_index = (left_read_index + 1) & DMA_BUFFER_MASK;
    }

    uint32_t r_curr_addr = (uint32_t)dma_hw->ch[dma_chan_right].write_addr;
    uint32_t r_write_idx = (r_curr_addr - (uint32_t)right_dma_buffer) / 4;
    r_write_idx &= DMA_BUFFER_MASK;

    while (right_read_index != r_write_idx) {
        int32_t val = right_dma_buffer[right_read_index] & 0x3;
        if (val == 0) right_total++;
        else if (val == 2) right_total--;
        else right_errors++;
        right_read_index = (right_read_index + 1) & DMA_BUFFER_MASK;
    }
}

void setup_pio_encoders() {
    uint offset = pio_add_program(pio, &simple_quad_counter_program);
    simple_quad_counter_program_init(pio, sm_left, offset, L_X_PIN, L_B_PIN);
    simple_quad_counter_program_init(pio, sm_right, offset, R_X_PIN, R_B_PIN);
}

// --- SEQUENCE HANDLERS (CALLED FROM MAIN) ---

void run_blocking_sequence() {
    if (sequence_length == 0) {
        printf("SEQ: No steps defined.\n");
        bt_printf("SEQ: No steps defined.\n");
        return;
    }

    printf("SEQ: Starting sequence with %d steps...\n", sequence_length);
    record_index = 0;
    absolute_time_t run_start_time = get_absolute_time();
    
    for (int i = 0; i < sequence_length; i++) {
        Step s = sequence[i];
        printf("SEQ: Step %d (Dur: %dms, L: %d, R: %d)\n", i, s.duration_ms, s.left_speed, s.right_speed);
        
        balboa_set_speeds(s.left_speed, s.right_speed);
        
        // Use a relative timer for the step duration
        absolute_time_t step_end_time = make_timeout_time_ms(s.duration_ms);

        while (get_absolute_time() < step_end_time) {
            
            // Collect Data
            lsm6ds3_read_all();
            update_counts();
            absolute_time_t t_now = get_absolute_time();

            if (record_index < MAX_RECORD_SAMPLES) {
                DataPoint *p = &record_buffer[record_index];
                p->timestamp_us = to_us_since_boot(t_now);
                p->ax = ax; p->ay = ay; p->az = az;
                p->gx = gx; p->gy = gy; p->gz = gz;
                p->left_enc = left_total;
                p->right_enc = right_total;
                // We record the left speed as the generic motor cmd for simple viz, 
                // or you could expand struct to record both.
                p->motor_cmd = s.left_speed; 
                record_index++;
            }

            // Keep BT stack alive during long runs
            cyw43_arch_poll();
            sleep_ms(1); 
        }
    }

    balboa_set_speeds(0, 0);
    printf("SEQ: Complete. Recorded %d samples.\n", record_index);
    bt_printf("SEQ: Complete. Recorded %d samples.\n", record_index);
}

void dma_dump_sequence() {
    printf("DMA: Dumping 1024 buffer entries...\n");
    bt_printf("DMA: Dumping 1024 buffer entries...\n");
    
    // Snapshot current write pointers from DMA hardware registers
    uint32_t l_curr_addr = (uint32_t)dma_hw->ch[dma_chan_left].write_addr;
    uint32_t l_write_idx = (l_curr_addr - (uint32_t)left_dma_buffer) / 4;
    l_write_idx &= DMA_BUFFER_MASK;
    
    uint32_t r_curr_addr = (uint32_t)dma_hw->ch[dma_chan_right].write_addr;
    uint32_t r_write_idx = (r_curr_addr - (uint32_t)right_dma_buffer) / 4;
    r_write_idx &= DMA_BUFFER_MASK;

    printf("PTRS: L_Read=%d L_Write=%d | R_Read=%d R_Write=%d\n", left_read_index, l_write_idx, right_read_index, r_write_idx);
    bt_printf("PTRS: L_Read=%d L_Write=%d | R_Read=%d R_Write=%d\n", left_read_index, l_write_idx, right_read_index, r_write_idx);

    printf("Idx,LeftRaw,RightRaw\n");
    bt_printf("Idx,LeftRaw,RightRaw\n");

    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
        // Print raw values as Hex for debugging bitmasks
        printf("%d,0x%08X,0x%08X\n", i, left_dma_buffer[i], right_dma_buffer[i]);
        bt_printf("%d,0x%08X,0x%08X\n", i, left_dma_buffer[i], right_dma_buffer[i]);
        
        // Poll frequently to prevent BT buffer overflow or connection timeouts during large dump
        // INCREASED DELAY: Printing 1024 lines too fast floods the BT buffer.
        // We now pause for 10ms every 16 lines to let the stack drain.
        if ((i & 0x0F) == 0) { 
            cyw43_arch_poll();
            sleep_ms(10);
        }
    }
    printf("DMA: Dump Complete\n");
    bt_printf("DMA: Dump Complete\n");
}

void show_sequence() {
    printf("SEQ: Current Sequence (%d steps):\n", sequence_length);
    bt_printf("SEQ: Current Sequence (%d steps):\n", sequence_length);

    if (sequence_length == 0) {
        printf("  (Empty)\n");
        bt_printf("  (Empty)\n");
        return;
    }

    for (int i = 0; i < sequence_length; i++) {
        Step s = sequence[i];
        printf("  Step %d: Dur=%dms, L=%d, R=%d\n", i, s.duration_ms, s.left_speed, s.right_speed);
        bt_printf("  Step %d: Dur=%dms, L=%d, R=%d\n", i, s.duration_ms, s.left_speed, s.right_speed);
        cyw43_arch_poll(); 
        sleep_ms(2);
    }
}

void print_help() {
    printf("--- Pico_Balboa Command List ---\n");
    printf("  RUN             : Execute the current sequence\n");
    printf("  READ            : Offload recorded data (CSV format)\n");
    printf("  STEP i,ms,l,r   : Set step [i] with dur[ms] and speeds [l],[r]\n");
    printf("  SHOW_SEQ        : Display current sequence steps\n");
    printf("  DMA             : Dump raw DMA buffer (Debugging)\n");
    printf("  HELP            : Show this list\n");

    bt_printf("--- Command List ---\n");
    bt_printf("  RUN\n");
    bt_printf("  READ\n");
    bt_printf("  STEP i,ms,l,r\n");
    bt_printf("  SHOW_SEQ\n");
    bt_printf("  DMA\n");
    bt_printf("  HELP\n");
}

void read_sequence() {
    printf("CMD: Offloading Data... Records: %d\n", record_index);
    bt_printf("CMD: Offloading Data... Records: %d\n", record_index);
    balboa_set_speeds(0, 0); 

    if (record_index == 0) {
        printf("WARNING: No data recorded yet.\n");
        bt_printf("WARNING: No data recorded yet.\n");
        return;
    }

    printf("DATA_START\n");
    bt_printf("DATA_START\n");
    
    printf("Idx,Time_us,AX,AY,AZ,GX,GY,GZ,LeftEnc,RightEnc,MotorCmd\n");
    bt_printf("Idx,Time_us,AX,AY,AZ,GX,GY,GZ,LeftEnc,RightEnc,MotorCmd\n");

    for (uint32_t i = 0; i < record_index; i++) {
        DataPoint *p = &record_buffer[i];
        
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                i, p->timestamp_us,
                p->ax, p->ay, p->az,
                p->gx, p->gy, p->gz,
                p->left_enc, p->right_enc,
                p->motor_cmd);

        bt_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                i, p->timestamp_us,
                p->ax, p->ay, p->az,
                p->gx, p->gy, p->gz,
                p->left_enc, p->right_enc,
                p->motor_cmd);
        
        // Critical for BT transmission
        sleep_ms(5); 
        cyw43_arch_poll(); 
    }
    printf("DATA_END\n");
    bt_printf("DATA_END\n");
}

// --- COMMAND PROCESSING (NON-BLOCKING) ---

void process_command(const char* cmd) {
    if (strncmp(cmd, "RUN", 3) == 0) {
        start_run_request = true;
    }
    else if (strncmp(cmd, "READ", 4) == 0) {
        start_read_request = true;
    }
    else if (strncmp(cmd, "DMA", 3) == 0) {
        start_dma_dump_request = true;
    }
    else if (strncmp(cmd, "SHOW_SEQ", 8) == 0) {
        start_show_seq_request = true;
    }
    else if (strncmp(cmd, "HELP", 4) == 0) {
        start_help_request = true;
    }
    else if (strncmp(cmd, "STEP", 4) == 0) {
        // Format: STEP idx, duration, left, right
        int idx, dur, l, r;
        // Try parsing with commas
        int matches = sscanf(cmd + 4, "%d, %d, %d, %d", &idx, &dur, &l, &r);
        if (matches < 4) {
             // Try parsing with spaces if commas fail
             matches = sscanf(cmd + 4, "%d %d %d %d", &idx, &dur, &l, &r);
        }

        if (matches == 4) {
            if (idx >= 0 && idx < MAX_STEPS) {
                sequence[idx].duration_ms = dur;
                sequence[idx].left_speed = (int16_t)l;
                sequence[idx].right_speed = (int16_t)r;
                
                // Update sequence length if we added a new step at the end
                if (idx >= sequence_length) {
                    sequence_length = idx + 1;
                }
                
                printf("CMD: Set Step %d: %dms, L:%d, R:%d\n", idx, dur, l, r);
                bt_printf("OK: Step %d set\n", idx);
            } else {
                printf("CMD: Error, Step Index %d out of bounds (Max %d)\n", idx, MAX_STEPS-1);
                bt_printf("ERR: Index bounds\n");
            }
        } else {
            printf("CMD: Error parsing STEP. Usage: STEP idx, dur, l, r\n");
            bt_printf("ERR: Parse error\n");
        }
    }
    else {
        printf("Unknown Command: %s\n", cmd);
        bt_printf("Unknown Command: %s\n", cmd);
    }
}

void check_usb_input() {
    int c = getchar_timeout_us(0); 
    if (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (usb_cmd_idx > 0) {
                usb_cmd_buffer[usb_cmd_idx] = 0; 
                process_command(usb_cmd_buffer);
                usb_cmd_idx = 0;
            }
        } else {
            if (usb_cmd_idx < 63) {
                usb_cmd_buffer[usb_cmd_idx++] = (char)c;
            }
        }
    }
}

int main() {
    stdio_init_all();
    
    gpio_init(ENABLE_PIN); gpio_set_dir(ENABLE_PIN, GPIO_OUT); gpio_put(ENABLE_PIN, 1);
    gpio_init(L_X_PIN); gpio_set_dir(L_X_PIN, GPIO_IN); gpio_pull_down(L_X_PIN);
    gpio_init(L_B_PIN); gpio_set_dir(L_B_PIN, GPIO_IN); gpio_pull_down(L_B_PIN);
    gpio_init(R_X_PIN); gpio_set_dir(R_X_PIN, GPIO_IN); gpio_pull_down(R_X_PIN);
    gpio_init(R_B_PIN); gpio_set_dir(R_B_PIN, GPIO_IN); gpio_pull_down(R_B_PIN);

    // Initialize Subsystems
    setup_pio_encoders();
    setup_dma_encoders();
    lsm6ds3_init();
    init_motor_i2c();
    balboa_set_speeds(0, 0);
    
    // --- DEFAULT SEQUENCE ---
    sequence[0] = (Step){100, 100, 100};
    sequence[1] = (Step){1000, 0, 0};
    sequence_length = 2;
    
    // --- BLUETOOTH INIT ---
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }
    l2cap_init();
    rfcomm_init();
    rfcomm_register_service(packet_handler, 1, 0xffff); 
    sdp_init();
    
    // Re-generate SDP record
    spp_create_sdp_record((uint8_t*)spp_service_buffer, 0x10001, 1, "Pico_Balboa");
    sdp_register_service((uint8_t*)spp_service_buffer);
    gap_set_local_name("Pico_Balboa");
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_discoverable_control(1);
    hci_power_control(HCI_POWER_ON);
    
    printf("--- RP2035 Ready (USB + Bluetooth). Waiting... ---\n");

    while (true) {
        // 1. Check USB Commands
        check_usb_input();
        
        // 2. Poll Bluetooth
        cyw43_arch_poll();

        // 3. Handle Flags (Execution Context)
        if (start_run_request) {
            start_run_request = false;
            run_blocking_sequence();
        }

        if (start_read_request) {
            start_read_request = false;
            read_sequence();
        }

        if (start_dma_dump_request) {
            start_dma_dump_request = false;
            dma_dump_sequence();
        }

        if (start_show_seq_request) {
            start_show_seq_request = false;
            show_sequence();
        }

        if (start_help_request) {
            start_help_request = false;
            print_help();
        }

        sleep_ms(1);
    }
    return 0;
}