#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

typedef struct {
    uint32_t timestamp_us;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int32_t left_enc, right_enc;
    int16_t motor_cmd; 
} DataPoint;

DataPoint record_buffer[MAX_RECORD_SAMPLES];
uint32_t record_index = 0;
bool is_recording = false;

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

int16_t current_motor_speed = 0;
bool run_sequence_active = false;
absolute_time_t run_start_time;

char usb_cmd_buffer[32];
int usb_cmd_idx = 0;

// Bluetooth Globals
static uint16_t rfcomm_channel_id;
static char bt_cmd_buffer[32];
static int bt_cmd_idx = 0;

// --- BLUETOOTH SPP SETUP ---

// Buffer for SDP record (Must be RAM, populated at runtime)
uint8_t spp_service_buffer[256];

// Forward declaration for command processing
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
        // Handle incoming data from Bluetooth
        for (int i = 0; i < size; i++) {
            char c = (char)packet[i];
            if (c == '\n' || c == '\r') {
                if (bt_cmd_idx > 0) {
                    bt_cmd_buffer[bt_cmd_idx] = 0; // Null terminate
                    printf("BT CMD: %s\n", bt_cmd_buffer); // Echo to USB for debug
                    process_command(bt_cmd_buffer);
                    bt_cmd_idx = 0;
                }
            } else {
                if (bt_cmd_idx < 31) {
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

// --- COMMAND PROCESSING ---

void process_command(const char* cmd) {
    if (strncmp(cmd, "START", 5) == 0) {
        printf("CMD: Recording Started\n");
        record_index = 0;
        is_recording = true;
    } 
    else if (strncmp(cmd, "STOP", 4) == 0) {
        printf("CMD: Stopped\n");
        is_recording = false;
        run_sequence_active = false;
        current_motor_speed = 0;
        balboa_set_speeds(0, 0);
    }
    else if (strncmp(cmd, "RUN", 3) == 0) {
        printf("CMD: Run Sequence Initiated\n");
        // Start Run
        run_sequence_active = true;
        run_start_time = get_absolute_time();
        current_motor_speed = -100; 
        balboa_set_speeds(-100, -100);
        
        // Auto-start Recording
        printf("CMD: Recording Started (RUN)\n");
        record_index = 0;
        is_recording = true;
    }
    else if (strncmp(cmd, "READ", 4) == 0) {
        printf("CMD: Offloading Data...\n");
        bool was_recording = is_recording;
        is_recording = false; 
        balboa_set_speeds(0, 0);

        printf("DATA_START\n");
        printf("Idx,Time_us,AX,AY,AZ,GX,GY,GZ,LeftEnc,RightEnc,MotorCmd\n");
        for (uint32_t i = 0; i < record_index; i++) {
            DataPoint *p = &record_buffer[i];
            printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   i, p->timestamp_us,
                   p->ax, p->ay, p->az,
                   p->gx, p->gy, p->gz,
                   p->left_enc, p->right_enc,
                   p->motor_cmd);
        }
        printf("DATA_END\n");
        
        if (was_recording) printf("WARNING: Recording was interrupted by READ.\n");
    }
    else {
        printf("Unknown Command: %s\n", cmd);
    }
}

void check_usb_input() {
    int c = getchar_timeout_us(0); // Non-blocking read
    if (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (usb_cmd_idx > 0) {
                usb_cmd_buffer[usb_cmd_idx] = 0; // Null terminate
                process_command(usb_cmd_buffer);
                usb_cmd_idx = 0;
            }
        } else {
            if (usb_cmd_idx < 31) {
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
    
    // --- BLUETOOTH INIT ---
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }
    l2cap_init();
    rfcomm_init();
    rfcomm_register_service(packet_handler, 1, 0xffff); // Channel 1
    sdp_init();
    
    // Re-generate SDP record
    spp_create_sdp_record((uint8_t*)spp_service_buffer, 0x10001, 1, "Pico_Balboa");
    sdp_register_service((uint8_t*)spp_service_buffer);
    gap_set_local_name("Pico_Balboa");
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_discoverable_control(1);
    hci_power_control(HCI_POWER_ON);
    
    printf("--- RP2035 Ready (USB + Bluetooth). Waiting... ---\n");

    absolute_time_t t_now;
    uint32_t cycle_start_us;
    
    while (true) {
        t_now = get_absolute_time();
        cycle_start_us = to_us_since_boot(t_now);

        // 1. Check USB Commands
        check_usb_input();
        
        // 2. Poll Bluetooth (Wireless Stack)
        cyw43_arch_poll();

        // 3. Run Sequence Logic
        if (run_sequence_active) {
            int64_t elapsed_us = absolute_time_diff_us(run_start_time, t_now);
            if (elapsed_us >= 500000) { // 0.5 seconds
                // Stop Motors
                current_motor_speed = 0;
                balboa_set_speeds(0, 0);
                run_sequence_active = false;
                
                // Stop Recording automatically when Run is done
                is_recording = false;
                printf("INFO: Run Sequence Complete. Recording Stopped.\n");
            }
        }

        // 4. Read Sensors
        lsm6ds3_read_all();

        // 5. Update Encoders
        update_counts();
        
        // 6. Send Motor Command
        balboa_set_speeds(current_motor_speed, current_motor_speed);

        // 7. Record Data
        if (is_recording && record_index < MAX_RECORD_SAMPLES) {
            DataPoint *p = &record_buffer[record_index];
            p->timestamp_us = cycle_start_us;
            p->ax = ax; p->ay = ay; p->az = az;
            p->gx = gx; p->gy = gy; p->gz = gz;
            p->left_enc = left_total;
            p->right_enc = right_total;
            p->motor_cmd = current_motor_speed;
            
            record_index++;
            if (record_index >= MAX_RECORD_SAMPLES) {
                printf("INFO: Memory Buffer Full. Stopping Recording.\n");
                is_recording = false;
            }
        }
        
        // 8. Sleep
        sleep_ms(1);
    }
    return 0;
}