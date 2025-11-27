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
#define R_X_PIN 27  
#define R_B_PIN 26  
#define ENABLE_PIN 20

// --- I2C0: MOTOR CONTROLLER (100kHz) ---
#define I2C_MOT_PORT i2c0
#define I2C_MOT_SDA 16
#define I2C_MOT_SCL 17
#define BALBOA_ADDR 0x42
#define CMD_ACTIVATE_MOTORS 0x81

// --- DMA BUFFER CONFIGURATION ---
#define DMA_BUFFER_SIZE 1024 
#define DMA_BUFFER_MASK (DMA_BUFFER_SIZE - 1) 

// --- RECORDING CONFIGURATION ---
#define MAX_RECORD_SAMPLES 5000

// --- SEQUENCE CONFIGURATION ---
#define MAX_STEPS 50

// Simplified structure for encoder debugging
typedef struct {
    uint32_t timestamp_us;
    uint8_t left_raw;      // Raw 2-bit value from PIO (0-3)
    uint8_t right_raw;     // Raw 2-bit value from PIO (0-3)
    int32_t left_total;    // Running total after decoding
    int32_t right_total;   // Running total after decoding
    int16_t motor_cmd;     // Motor command for context
} EncoderDebugPoint;

typedef struct {
    uint32_t duration_ms;
    int16_t left_speed;
    int16_t right_speed;
} Step;

EncoderDebugPoint record_buffer[MAX_RECORD_SAMPLES];
uint32_t record_index = 0;

Step sequence[MAX_STEPS];
int sequence_length = 0;

// --- GLOBALS ---
PIO pio = pio0;
uint sm_left = 0;
uint sm_right = 1;

int dma_chan_left;
int dma_chan_right;

volatile uint8_t left_prev_state = 0xFF;   // Track previous state
volatile uint8_t right_prev_state = 0xFF;  // Initialize to invalid

volatile int32_t left_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096)));
volatile int32_t right_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096)));

uint32_t left_read_index = 0;
uint32_t right_read_index = 0;

volatile int32_t left_total = 0;
volatile int32_t right_total = 0;
volatile int32_t left_errors = 0;
volatile int32_t right_errors = 0;

// NEW: Store last raw values for recording
volatile uint8_t last_left_raw = 0;
volatile uint8_t last_right_raw = 0;

char usb_cmd_buffer[64];
int usb_cmd_idx = 0;

// Bluetooth Globals
static uint16_t rfcomm_channel_id = 0; 
static char bt_cmd_buffer[64];
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

int8_t quadrature_decode(uint8_t prev, uint8_t curr) {
    if (prev == 0xFF) return 0;  // First call initialization
    
    // Lookup table: [prev_state][curr_state] = direction
    static const int8_t lut[4][4] = {
        { 0,  1, -1,  0},  // From state 0
        {-1,  0,  0,  1},  // From state 1
        { 1,  0,  0, -1},  // From state 2
        { 0, -1,  1,  0}   // From state 3
    };
    
    return lut[prev & 0x3][curr & 0x3];
}

void update_counts() {
    // LEFT encoder
    uint32_t l_curr_addr = (uint32_t)dma_hw->ch[dma_chan_left].write_addr;
    uint32_t l_write_idx = (l_curr_addr - (uint32_t)left_dma_buffer) / 4;
    l_write_idx &= DMA_BUFFER_MASK;

    while (left_read_index != l_write_idx) {
        uint8_t val = left_dma_buffer[left_read_index] & 0x3;
        last_left_raw = val;  // Store for recording
        
        // SIMPLE 2X DECODER:
        // Check bit 1 (B channel): 0 = forward, 1 = reverse
        if ((val & 0x2) == 0) {
            left_total++;   // B=0 at X edge: forward
        } else {
            left_total--;   // B=1 at X edge: reverse
        }
        
        left_read_index = (left_read_index + 1) & DMA_BUFFER_MASK;
    }

    // RIGHT encoder
    uint32_t r_curr_addr = (uint32_t)dma_hw->ch[dma_chan_right].write_addr;
    uint32_t r_write_idx = (r_curr_addr - (uint32_t)right_dma_buffer) / 4;
    r_write_idx &= DMA_BUFFER_MASK;

    while (right_read_index != r_write_idx) {
        uint8_t val = right_dma_buffer[right_read_index] & 0x3;
        last_right_raw = val;  // Store for recording
        
        // SIMPLE 2X DECODER:
        if ((val & 0x2) == 0) {
            right_total++;   // B=0 at X edge: forward
        } else {
            right_total--;   // B=1 at X edge: reverse
        }
        
        right_read_index = (right_read_index + 1) & DMA_BUFFER_MASK;
    }
}


void setup_pio_encoders() {
    uint offset = pio_add_program(pio, &quad_both_edges_program);
    quad_both_edges_program_init(pio, sm_left, offset, L_X_PIN, L_B_PIN);
    quad_both_edges_program_init(pio, sm_right, offset, R_X_PIN, R_B_PIN);
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
    
    // Reset totals before run
    left_total = 0;
    right_total = 0;
    left_errors = 0;
    right_errors = 0;
    
    for (int i = 0; i < sequence_length; i++) {
        Step s = sequence[i];
        printf("SEQ: Step %d (Dur: %dms, L: %d, R: %d)\n", i, s.duration_ms, s.left_speed, s.right_speed);
        
        balboa_set_speeds(s.left_speed, s.right_speed);
        
        absolute_time_t step_end_time = make_timeout_time_ms(s.duration_ms);

        while (get_absolute_time() < step_end_time) {
            
            // Update encoder counts (this also updates last_left_raw and last_right_raw)
            update_counts();
            
            absolute_time_t t_now = get_absolute_time();

            if (record_index < MAX_RECORD_SAMPLES) {
                EncoderDebugPoint *p = &record_buffer[record_index];
                p->timestamp_us = to_us_since_boot(t_now);
                p->left_raw = last_left_raw;
                p->right_raw = last_right_raw;
                p->left_total = left_total;
                p->right_total = right_total;
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
    printf("SEQ: Final counts: L=%d, R=%d, Errors: L=%d, R=%d\n", 
           left_total, right_total, left_errors, right_errors);
    bt_printf("SEQ: Complete. Recorded %d samples.\n", record_index);
    bt_printf("SEQ: Final counts: L=%d, R=%d, Errors: L=%d, R=%d\n", 
              left_total, right_total, left_errors, right_errors);
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

    printf("PTRS: L_Read=%d L_Write=%d | R_Read=%d R_Write=%d\n", 
           left_read_index, l_write_idx, right_read_index, r_write_idx);
    bt_printf("PTRS: L_Read=%d L_Write=%d | R_Read=%d R_Write=%d\n", 
              left_read_index, l_write_idx, right_read_index, r_write_idx);

    printf("Idx,LeftRaw,RightRaw\n");
    bt_printf("Idx,LeftRaw,RightRaw\n");

    for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
        // Print raw values as Hex for debugging bitmasks
        printf("%d,0x%08X,0x%08X\n", i, left_dma_buffer[i], right_dma_buffer[i]);
        bt_printf("%d,0x%08X,0x%08X\n", i, left_dma_buffer[i], right_dma_buffer[i]);
        
        // Poll frequently to prevent BT buffer overflow
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
    printf("--- Pico_Balboa ENCODER DEBUG ---\n");
    printf("  RUN             : Execute sequence and record RAW encoder values\n");
    printf("  READ            : Offload recorded encoder data (CSV format)\n");
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
    printf("CMD: Offloading Encoder Debug Data... Records: %d\n", record_index);
    bt_printf("CMD: Offloading Data... Records: %d\n", record_index);
    balboa_set_speeds(0, 0); 

    if (record_index == 0) {
        printf("WARNING: No data recorded yet.\n");
        bt_printf("WARNING: No data recorded yet.\n");
        return;
    }

    printf("DATA_START\n");
    bt_printf("DATA_START\n");
    
    printf("Idx,Time_us,L_Raw,R_Raw,L_Total,R_Total,MotorCmd\n");
    bt_printf("Idx,Time_us,L_Raw,R_Raw,L_Total,R_Total,MotorCmd\n");

    for (uint32_t i = 0; i < record_index; i++) {
        EncoderDebugPoint *p = &record_buffer[i];
        
        printf("%d,%d,%d,%d,%d,%d,%d\n",
                i, p->timestamp_us,
                p->left_raw, p->right_raw,
                p->left_total, p->right_total,
                p->motor_cmd);

        bt_printf("%d,%d,%d,%d,%d,%d,%d\n",
                i, p->timestamp_us,
                p->left_raw, p->right_raw,
                p->left_total, p->right_total,
                p->motor_cmd);
        
        // Critical for BT transmission
        sleep_ms(5); 
        cyw43_arch_poll(); 
    }
    printf("DATA_END\n");
    printf("SUMMARY: Final Totals: L=%d, R=%d, Errors: L=%d, R=%d\n",
           left_total, right_total, left_errors, right_errors);
    
    bt_printf("DATA_END\n");
    bt_printf("SUMMARY: L=%d, R=%d, Err: L=%d, R=%d\n",
              left_total, right_total, left_errors, right_errors);
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
        int idx, dur, l, r;
        int matches = sscanf(cmd + 4, "%d, %d, %d, %d", &idx, &dur, &l, &r);
        if (matches < 4) {
             matches = sscanf(cmd + 4, "%d %d %d %d", &idx, &dur, &l, &r);
        }

        if (matches == 4) {
            if (idx >= 0 && idx < MAX_STEPS) {
                sequence[idx].duration_ms = dur;
                sequence[idx].left_speed = (int16_t)l;
                sequence[idx].right_speed = (int16_t)r;
                
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
    init_motor_i2c();
    balboa_set_speeds(0, 0);
    
    // --- DEFAULT SEQUENCE: Forward then reverse ---
    sequence[0] = (Step){2000, 100, 100};   // Forward 2 sec
    sequence[1] = (Step){500, 0, 0};        // Stop 0.5 sec
    sequence[2] = (Step){2000, -100, -100}; // Reverse 2 sec
    sequence[3] = (Step){500, 0, 0};        // Stop 0.5 sec
    sequence_length = 4;
    
    // --- BLUETOOTH INIT ---
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }
    l2cap_init();
    rfcomm_init();
    rfcomm_register_service(packet_handler, 1, 0xffff); 
    sdp_init();
    
    spp_create_sdp_record((uint8_t*)spp_service_buffer, 0x10001, 1, "Pico_Balboa_Debug");
    sdp_register_service((uint8_t*)spp_service_buffer);
    gap_set_local_name("Pico_Balboa_Debug");
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_discoverable_control(1);
    hci_power_control(HCI_POWER_ON);
    
    printf("=== RP2350 ENCODER DEBUG Ready (USB + Bluetooth) ===\n");
    printf("Default sequence: FWD(2s) -> STOP(0.5s) -> REV(2s) -> STOP(0.5s)\n");
    printf("Type HELP for commands\n");

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
