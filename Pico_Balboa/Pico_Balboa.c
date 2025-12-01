// Pico_Balboa.c
// Generated: November 30, 2024 at 18:15 UTC
// Raspberry Pi Pico 2 W - Balboa Robot Balance Controller
// Hardware: 620 counts/rev encoders, 80mm wheels, GEAR_RATIO=76
// Recording: 11 fields including trajectory_error, encoders reset after INIT

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
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
#define NORMAL_MODE_208HZ 0x58  // 208 Hz, ±1000 dps
#define RESET_STEPS       0x02
#define SET_FUNC_EN       0xBD

// --- DMA BUFFER CONFIGURATION ---
#define DMA_BUFFER_SIZE 1024 
#define DMA_BUFFER_MASK (DMA_BUFFER_SIZE - 1) 

// --- RECORDING CONFIGURATION ---
#define MAX_RECORD_SAMPLES 10000

// --- SEQUENCE CONFIGURATION ---
#define MAX_STEPS 50

// --- BALANCING CONFIGURATION ---
#define UPDATE_TIME_MS 10           // 100 Hz update rate
#define DEFAULT_INIT_TIME_MS 1000

// Step command types
typedef enum {
    CMD_DRIVE,      
    CMD_INIT,       
    CMD_STAND,      
    CMD_MOVE,       
    CMD_STOP        
} CommandType;

typedef struct {
    uint32_t timestamp_us;      // Microseconds since boot
    int16_t ax, ay, az;         // ax=accel_x, ay=accel_z, az=gyro_y
    int16_t gx, gy, gz;         // gx=motor_cmd_left, gy=motor_cmd_right, gz=angle(deg)
    int32_t left_enc, right_enc;
    int16_t motor_cmd;          // Base motor command (motorSpeed)
    int16_t trajectory_error;   // risingAngleOffset (deg)
} DataPoint;

typedef struct {
    CommandType cmd_type;           
    uint32_t duration_ms;           
    int16_t left_speed;             // Phase 1 speed for STAND
    int16_t right_speed;            // Phase 2 speed for STAND
    int32_t move_left;              
    int32_t move_right;             
    int32_t stand_trigger;          
} Step;

DataPoint record_buffer[MAX_RECORD_SAMPLES];
uint32_t record_index = 0;
bool is_recording = false;  // True during critical data recording (after INIT, before fall)

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

// --- BALANCING STATE VARIABLES ---
int16_t gx_zero = 0;
int16_t gy_zero = 0;
int16_t gz_zero = 0;

bool is_balancing = false;
int32_t target_left = 0;
int32_t target_right = 0;

// PID Variables
float pid_integral_error = 0.0f; // Accumulator for I term

// --- POLOLU-STYLE BALANCE VARIABLES ---
float angle = 0.0f;               // Angle in degrees (0 = vertical)
float angleRate = 0.0f;           // Angular rate in degrees/s
float risingAngleOffset = 0.0f;   // Trajectory error (angleRate*140 + angle)
int16_t motorSpeed = 0;           // Accumulated motor speed command
int32_t distanceLeft = 0;         // Left encoder position (for differential correction)
int32_t distanceRight = 0;        // Right encoder position (for differential correction)

// Pololu-style control gains (tunable at runtime)
int16_t ANGLE_RATE_RATIO = 140;      // Physical constant relating angle to rate (ms)
int16_t ANGLE_RESPONSE = 11;         // Trajectory stabilization gain
int16_t DISTANCE_DIFF_RESPONSE = 0;  // Differential correction gain (start disabled)
int16_t GEAR_RATIO = 76;             // Gear ratio (620 counts/rev, 80mm wheels)
int16_t MOTOR_SPEED_LIMIT = 300;     // Maximum motor speed

// Hardware specifications (for reference):
// - Encoder: 620 counts per wheel rotation
// - Wheel diameter: 80mm
// - Wheel circumference: π × 80mm = 251.3mm
// - Distance per count: 251.3mm / 620 = 0.405mm/count
// Note: GEAR_RATIO scales motor commands to physical motion
//       Adjusted from Pololu default (111) based on encoder ratio (620/910)

// PID parameters (Defaults, adjust via PID command)
// Note: P is now acting on Accel Z, D is acting on Gyro Y
float balance_kp = 0.05;    // Proportional gain (Accel Z error)
float balance_ki = 0.00;    // Integral gain (Accel Z accumulation)
float balance_kd = 0.50;    // Derivative gain (Gyro Y rate)

// PID component values for recording/debugging
int16_t pid_p = 0;
int16_t pid_d = 0;
int16_t actual_motor_cmd = 0;

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
        
        switch (s.cmd_type) {
            case CMD_DRIVE:
                printf("  Step %d: DRIVE Dur=%dms, L=%d, R=%d\n", 
                       i, s.duration_ms, s.left_speed, s.right_speed);
                bt_printf("  Step %d: DRIVE Dur=%dms, L=%d, R=%d\n", 
                          i, s.duration_ms, s.left_speed, s.right_speed);
                break;
            case CMD_INIT:
                printf("  Step %d: INIT Dur=%dms\n", i, s.duration_ms);
                bt_printf("  Step %d: INIT Dur=%dms\n", i, s.duration_ms);
                break;
            case CMD_STAND:
                printf("  Step %d: STAND s1=%d, s2=%d (trig=%ld)\n", 
                       i, s.left_speed, s.right_speed, (long)s.stand_trigger);
                bt_printf("  Step %d: STAND s1=%d, s2=%d (trig=%ld)\n", 
                          i, s.left_speed, s.right_speed, (long)s.stand_trigger);
                break;
            case CMD_MOVE:
                printf("  Step %d: MOVE L=%ld, R=%ld\n", i, s.move_left, s.move_right);
                bt_printf("  Step %d: MOVE L=%ld, R=%ld\n", i, s.move_left, s.move_right);
                break;
            case CMD_STOP:
                printf("  Step %d: STOP\n", i);
                bt_printf("  Step %d: STOP\n", i);
                break;
        }
        
        cyw43_arch_poll(); 
        sleep_ms(2);
    }
}

void print_help() {
    printf("--- Pico_Balboa Command List ---\n");
    printf("  RUN             : Execute the current sequence\n");
    printf("  READ            : Offload recorded data (CSV format)\n");
    printf("  STEP i,ms,l,r   : Set step [i] DRIVE with dur[ms] and speeds [l],[r]\n");
    printf("  INIT i[,ms]     : Set step [i] gyro calibration (default 1000ms)\n");
    printf("  STAND i,s1,s2   : Set step [i] stand. Phase1/2 speeds s1,s2. [trig] optional\n");
    printf("  MOVE i,l,r      : Set step [i] to move [l],[r] encoder counts\n");
    printf("  STOP i          : Set step [i] to stop balancing\n");
    printf("  PID p,i,d       : Set PID gains (P, I, D)\n");
    printf("  SHOW_SEQ        : Display current sequence steps\n");
    printf("  DMA             : Dump raw DMA buffer (Debugging)\n");
    printf("  HELP            : Show this list\n");

    bt_printf("--- Command List ---\n");
    bt_printf("  RUN, READ, SHOW_SEQ, DMA, HELP\n");
    bt_printf("  STEP i,ms,l,r\n");
    bt_printf("  INIT i[,ms]\n");
    bt_printf("  STAND i,s1,s2[,trig]\n");
    bt_printf("  MOVE i,l,r\n");
    bt_printf("  STOP i\n");
    bt_printf("  PID p,i,d\n");
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
    
    // Note: P term is stored in GX, D term in GZ for debugging
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
        sleep_ms(3); 
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
        int idx, dur, l, r;
        int matches = sscanf(cmd + 4, "%d, %d, %d, %d", &idx, &dur, &l, &r);
        if (matches < 4) matches = sscanf(cmd + 4, "%d %d %d %d", &idx, &dur, &l, &r);

        if (matches == 4 && idx >= 0 && idx < MAX_STEPS) {
            sequence[idx] = (Step){CMD_DRIVE, dur, (int16_t)l, (int16_t)r, 0, 0, 0};
            if (idx >= sequence_length) sequence_length = idx + 1;
            printf("CMD: Set Step %d: DRIVE %dms, L:%d, R:%d\n", idx, dur, l, r);
            bt_printf("OK: Step %d set\n", idx);
        }
    }
    else if (strncmp(cmd, "INIT", 4) == 0) {
        int idx, dur = DEFAULT_INIT_TIME_MS;
        int matches = sscanf(cmd + 4, "%d, %d", &idx, &dur);
        if (matches < 1) matches = sscanf(cmd + 4, "%d %d", &idx, &dur);
        
        if (matches >= 1 && idx >= 0 && idx < MAX_STEPS) {
            sequence[idx] = (Step){CMD_INIT, dur, 0, 0, 0, 0, 0};
            if (idx >= sequence_length) sequence_length = idx + 1;
            printf("CMD: Set Step %d: INIT %dms\n", idx, dur);
            bt_printf("OK: Step %d INIT set\n", idx);
        }
    }
    else if (strncmp(cmd, "STAND", 5) == 0) {
        int idx, s1, s2, trig = 8000;
        int matches = sscanf(cmd + 5, "%d, %d, %d, %d", &idx, &s1, &s2, &trig);
        if (matches < 3) matches = sscanf(cmd + 5, "%d, %d, %d", &idx, &s1, &s2);
        if (matches < 3) matches = sscanf(cmd + 5, "%d %d %d %d", &idx, &s1, &s2, &trig);
        
        if (matches >= 3 && idx >= 0 && idx < MAX_STEPS) {
            sequence[idx] = (Step){CMD_STAND, 0, (int16_t)s1, (int16_t)s2, 0, 0, trig};
            if (idx >= sequence_length) sequence_length = idx + 1;
            printf("CMD: Set Step %d: STAND s1=%d s2=%d trig=%d\n", idx, s1, s2, trig);
            bt_printf("OK: Step %d STAND set\n", idx);
        }
    }
    else if (strncmp(cmd, "MOVE", 4) == 0) {
        int idx, l, r;
        int matches = sscanf(cmd + 4, "%d, %d, %d", &idx, &l, &r);
        if (matches < 3) matches = sscanf(cmd + 4, "%d %d %d", &idx, &l, &r);
        
        if (matches == 3 && idx >= 0 && idx < MAX_STEPS) {
            sequence[idx] = (Step){CMD_MOVE, 0, 0, 0, l, r, 0};
            if (idx >= sequence_length) sequence_length = idx + 1;
            printf("CMD: Set Step %d: MOVE L:%d R:%d\n", idx, l, r);
            bt_printf("OK: Step %d MOVE set\n", idx);
        }
    }
    else if (strncmp(cmd, "STOP", 4) == 0) {
        int idx;
        if (sscanf(cmd + 4, "%d", &idx) == 1 && idx >= 0 && idx < MAX_STEPS) {
            sequence[idx] = (Step){CMD_STOP, 0, 0, 0, 0, 0, 0};
            if (idx >= sequence_length) sequence_length = idx + 1;
            printf("CMD: Set Step %d: STOP\n", idx);
            bt_printf("OK: Step %d STOP set\n", idx);
        }
    }
    else if (strncmp(cmd, "PID", 3) == 0) {
        float p, i, d;
        int matches = sscanf(cmd + 3, "%f, %f, %f", &p, &i, &d);
        if (matches < 3) matches = sscanf(cmd + 3, "%f %f %f", &p, &i, &d);
        
        if (matches == 3) {
            // For Pololu-style: P maps to ANGLE_RESPONSE
            ANGLE_RESPONSE = (int16_t)p;
            balance_kp = p;  // Keep for compatibility
            balance_ki = i;
            balance_kd = d;
            printf("CMD: Pololu gains set - ANGLE_RESPONSE=%d\n", ANGLE_RESPONSE);
            bt_printf("OK: ANGLE_RESPONSE=%d\n", ANGLE_RESPONSE);
        }
    }
    else if (strncmp(cmd, "ANGLE", 5) == 0) {
        int resp, ratio, diff;
        int matches = sscanf(cmd + 5, "%d, %d, %d", &resp, &ratio, &diff);
        if (matches < 3) matches = sscanf(cmd + 5, "%d %d %d", &resp, &ratio, &diff);
        
        if (matches == 3) {
            ANGLE_RESPONSE = resp;
            ANGLE_RATE_RATIO = ratio;
            DISTANCE_DIFF_RESPONSE = diff;
            printf("CMD: Pololu gains - RESP=%d, RATIO=%d, DIFF=%d\n", 
                   ANGLE_RESPONSE, ANGLE_RATE_RATIO, DISTANCE_DIFF_RESPONSE);
            bt_printf("OK: RESP=%d, RATIO=%d, DIFF=%d\n", 
                     ANGLE_RESPONSE, ANGLE_RATE_RATIO, DISTANCE_DIFF_RESPONSE);
        }
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


// --- SEQUENCE HANDLERS (CALLED FROM MAIN) ---

void calibrate_gyros(uint32_t duration_ms) {
    printf("INIT: Calibrating gyros for %dms...\n", duration_ms);
    bt_printf("INIT: Calibrating...\n");
    
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int32_t sum_ax = 0, sum_az = 0;
    int sample_count = 0;
    
    absolute_time_t cal_end = make_timeout_time_ms(duration_ms);
    absolute_time_t last_sample = get_absolute_time();
    
    while (get_absolute_time() < cal_end) {
        if (absolute_time_diff_us(last_sample, get_absolute_time()) >= (UPDATE_TIME_MS * 1000)) {
            lsm6ds3_read_all();
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
            sum_ax += ax;
            sum_az += az;
            sample_count++;
            last_sample = get_absolute_time();
        }
        tight_loop_contents();
    }
    
    if (sample_count > 0) {
        gx_zero = sum_gx / sample_count;
        gy_zero = sum_gy / sample_count;
        gz_zero = sum_gz / sample_count;
        
        // Compute initial angle from accelerometer
        // Angle = 0 when vertical (AX = max, AZ = 0)
        // Angle = 90° when lying flat (AX = 0, AZ = max)
        int32_t avg_ax = sum_ax / sample_count;
        int32_t avg_az = sum_az / sample_count;
        angle = atan2f((float)avg_az, (float)avg_ax) * 180.0f / M_PI;
        
        // Reset control state
        pid_integral_error = 0;
        motorSpeed = 0;
        angleRate = 0.0f;
        distanceLeft = 0;
        distanceRight = 0;
        
        // Reset encoder counts to zero
        left_total = 0;
        right_total = 0;
        
        printf("INIT: Gyro zeros: GX=%d, GY=%d, GZ=%d\n", gx_zero, gy_zero, gz_zero);
        printf("INIT: Initial angle: %.2f deg (AX=%ld, AZ=%ld)\n", angle, avg_ax, avg_az);
        printf("INIT: Encoder counts reset to zero\n");
        bt_printf("INIT: Calibrated. Angle=%.1f deg\n", angle);
    }
}

// Integrate gyro to update angle estimate
void integrateGyro() {
    // Convert gyro reading to deg/s
    // LSM6DS3 at ±1000 dps: 1000 dps = 29000 counts (from LSM6 library scaling)
    angleRate = (float)(gy - gy_zero) / 29.0f;
    
    // Integrate to get angle (UPDATE_TIME_MS = 10ms)
    angle += angleRate * (UPDATE_TIME_MS / 1000.0f);
    
    // Drift compensation: 0.01% per millisecond = 0.1% per 10ms cycle
    // Multiply by 0.999 (decay toward zero)
    angle = angle * 0.999f;
}

// Track encoder positions for differential correction
void integrateEncoders() {
    static int32_t lastCountsLeft = 0;
    static int32_t lastCountsRight = 0;
    
    int32_t countsLeft = left_total;
    int32_t countsRight = right_total;
    
    // Update accumulated distances
    distanceLeft += (countsLeft - lastCountsLeft);
    distanceRight += (countsRight - lastCountsRight);
    
    lastCountsLeft = countsLeft;
    lastCountsRight = countsRight;
}

// Balance Control Loop - POLOLU-STYLE WITH GYRO INTEGRATION
void balance_update() {
    // 1. Update angle and encoder tracking
    integrateGyro();
    integrateEncoders();
    
    // 2. Calculate rising angle offset (trajectory error)
    // This represents how far we are from a stable trajectory to vertical
    // Units: degrees (angleRate is deg/s, ANGLE_RATE_RATIO converts to deg)
    risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;
    
    // 3. Incremental motor speed update (Pololu-style)
    // Note: No distance/speed feedback per user request, only trajectory stabilization
    motorSpeed += (int16_t)((ANGLE_RESPONSE * risingAngleOffset) / 100.0f / GEAR_RATIO);
    
    // 4. Apply saturation
    if (motorSpeed > MOTOR_SPEED_LIMIT) motorSpeed = MOTOR_SPEED_LIMIT;
    if (motorSpeed < -MOTOR_SPEED_LIMIT) motorSpeed = -MOTOR_SPEED_LIMIT;
    
    // 5. Add differential correction for turning
    // Prevents rotation while rocking back and forth
    int32_t distanceDiff = distanceLeft - distanceRight;
    int16_t leftMotor = motorSpeed + (int16_t)((distanceDiff * DISTANCE_DIFF_RESPONSE) / 100);
    int16_t rightMotor = motorSpeed - (int16_t)((distanceDiff * DISTANCE_DIFF_RESPONSE) / 100);
    
    // 6. Send commands to motors
    balboa_set_speeds(leftMotor, rightMotor);
    actual_motor_cmd = motorSpeed;
    
    // Store components for debugging/recording
    pid_p = (int16_t)risingAngleOffset;  // Trajectory error in degrees
    pid_d = (int16_t)angleRate;          // Angular rate in deg/s
    
    // 7. Fall detection based on angle
    static int fall_angle_count = 0;
    if (fabsf(angle) > 70.0f) {  // Beyond 70 degrees from vertical
        if (++fall_angle_count > 5) {  // 50ms of being too far
            is_balancing = false;
            is_recording = false;  // Stop recording on fall
            motorSpeed = 0;
            balboa_set_speeds(0, 0);
            actual_motor_cmd = 0;
            fall_angle_count = 0;
            printf("FALL DETECTED: Angle=%.1f degrees. Recording stopped.\n", angle);
            bt_printf("FALL DETECTED: Angle too large\n");
        }
    } else {
        fall_angle_count = 0;
    }
}

// Stand up routine - Pololu-style with angle-based triggering
void balance_stand_up(int16_t speed1, int16_t speed2, int32_t trigger_threshold) {
    if (!is_recording) {
        printf("STAND: Pololu-style stand-up (s1=%d, s2=%d)...\n", speed1, speed2);
        bt_printf("STAND: Starting stand-up\n");
    }
    
    // Reset control state
    motorSpeed = 0;
    distanceLeft = 0;
    distanceRight = 0;
    
    // Phase 1: Backward kick (400ms at negative full speed)
    balboa_set_speeds(-MOTOR_SPEED_LIMIT, -MOTOR_SPEED_LIMIT);
    actual_motor_cmd = -MOTOR_SPEED_LIMIT;
    absolute_time_t phase1_end = make_timeout_time_ms(400);
    absolute_time_t last_update = get_absolute_time();
    
    while (get_absolute_time() < phase1_end) {
        absolute_time_t t_now = get_absolute_time();
        if (absolute_time_diff_us(last_update, t_now) >= (UPDATE_TIME_MS * 1000)) {
            lsm6ds3_read_all();
            update_counts();
            integrateGyro();
            integrateEncoders();
            
            // Calculate trajectory error for recording
            risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;
            
            // Calculate actual motor commands
            int32_t distanceDiff = distanceLeft - distanceRight;
            int16_t leftMotor = motorSpeed + (distanceDiff * DISTANCE_DIFF_RESPONSE) / 100;
            int16_t rightMotor = motorSpeed - (distanceDiff * DISTANCE_DIFF_RESPONSE) / 100;
            
            if (is_recording && record_index < MAX_RECORD_SAMPLES) {
                DataPoint *p = &record_buffer[record_index];
                p->timestamp_us = to_us_since_boot(t_now);
                p->ax = ax;                         // Accelerometer X
                p->ay = az;                         // Accelerometer Z
                p->az = gy;                         // Gyro Y (raw)
                p->gx = leftMotor;                  // Left motor command (actual)
                p->gy = rightMotor;                 // Right motor command (actual)
                p->gz = (int16_t)angle;             // Angle (degrees)
                p->left_enc = left_total;
                p->right_enc = right_total;
                p->motor_cmd = motorSpeed;          // Base motor command
                p->trajectory_error = (int16_t)risingAngleOffset;  // Trajectory error
                record_index++;
            }
            last_update = t_now;
        }
        tight_loop_contents();
    }
    
    // Phase 2: Forward drive (up to 200ms or until angle < 60°)
    balboa_set_speeds(150, 150);
    actual_motor_cmd = 150;
    absolute_time_t phase2_end = make_timeout_time_ms(200);
    last_update = get_absolute_time();
    
    while (get_absolute_time() < phase2_end) {
        absolute_time_t t_now = get_absolute_time();
        if (absolute_time_diff_us(last_update, t_now) >= (UPDATE_TIME_MS * 1000)) {
            lsm6ds3_read_all();
            update_counts();
            integrateGyro();
            integrateEncoders();
            
            // Calculate trajectory error for recording
            risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;
            
            // Calculate actual motor commands
            int32_t distanceDiff = distanceLeft - distanceRight;
            int16_t leftMotor = motorSpeed + (distanceDiff * DISTANCE_DIFF_RESPONSE) / 100;
            int16_t rightMotor = motorSpeed - (distanceDiff * DISTANCE_DIFF_RESPONSE) / 100;
            
            // Check if close enough to vertical (angle < 45°)
            if (fabsf(angle) < 45.0f) {
                if (!is_recording) {
                    printf("STAND: Triggered at angle=%.1f degrees\n", angle);
                    bt_printf("STAND: Balancing started\n");
                }
                is_balancing = true;
                motorSpeed = 150;  // Pre-load motor speed
                distanceLeft = 0;   // Reset for differential correction
                distanceRight = 0;
                break;
            }
            
            if (is_recording && record_index < MAX_RECORD_SAMPLES) {
                DataPoint *p = &record_buffer[record_index];
                p->timestamp_us = to_us_since_boot(t_now);
                p->ax = ax;                         // Accelerometer X
                p->ay = az;                         // Accelerometer Z
                p->az = gy;                         // Gyro Y (raw)
                p->gx = leftMotor;                  // Left motor command (actual)
                p->gy = rightMotor;                 // Right motor command (actual)
                p->gz = (int16_t)angle;             // Angle (degrees)
                p->left_enc = left_total;
                p->right_enc = right_total;
                p->motor_cmd = motorSpeed;          // Base motor command
                p->trajectory_error = (int16_t)risingAngleOffset;  // Trajectory error
                record_index++;
            }
            last_update = t_now;
        }
        tight_loop_contents();
    }
    
    if (!is_recording) {
        printf("STAND: Complete. Mode: %s, Angle: %.1f deg\n", 
               is_balancing ? "ACTIVE" : "inactive", angle);
        bt_printf("STAND: Complete\n");
    }
}

void balance_move_to(int32_t left_target, int32_t right_target) {
    printf("MOVE: Placeholder (L=%ld, R=%ld)\n", left_target, right_target);
    bt_printf("MOVE: Placeholder\n");
}

void balance_stop() {
    printf("STOP: Balance stopped\n");
    bt_printf("STOP: Balance stopped\n");
    is_balancing = false;
    balboa_set_speeds(0, 0);
}

void run_blocking_sequence() {
    if (sequence_length == 0) return;

    printf("SEQ: Starting sequence...\n");
    is_recording = false;  // Don't record during INIT
    
    for (int i = 0; i < sequence_length; i++) {
        Step s = sequence[i];
        
        switch (s.cmd_type) {
            case CMD_INIT:
                calibrate_gyros(s.duration_ms);
                // Print all control parameters before recording starts
                printf("\n=== CONTROL PARAMETERS ===\n");
                printf("Initial angle: %.2f deg\n", angle);
                printf("Gyro zeros: GX=%d, GY=%d, GZ=%d\n", gx_zero, gy_zero, gz_zero);
                printf("ANGLE_RESPONSE: %d\n", ANGLE_RESPONSE);
                printf("ANGLE_RATE_RATIO: %d\n", ANGLE_RATE_RATIO);
                printf("DISTANCE_DIFF_RESPONSE: %d\n", DISTANCE_DIFF_RESPONSE);
                printf("GEAR_RATIO: %d\n", GEAR_RATIO);
                printf("MOTOR_SPEED_LIMIT: %d\n", MOTOR_SPEED_LIMIT);
                printf("Update rate: %d ms (%.1f Hz)\n", UPDATE_TIME_MS, 1000.0f/UPDATE_TIME_MS);
                printf("==========================\n\n");
                
                // Start recording AFTER parameter dump
                record_index = 0;
                is_recording = true;
                printf("SEQ: Recording started\n");
                break;
            case CMD_STAND:
                balance_stand_up(s.left_speed, s.right_speed, s.stand_trigger);
                break;
            case CMD_MOVE:
                balance_move_to(s.move_left, s.move_right);
                break;
            case CMD_STOP:
                balance_stop();
                break;
            case CMD_DRIVE:
            default:
                balboa_set_speeds(s.left_speed, s.right_speed);
                actual_motor_cmd = s.left_speed;
                absolute_time_t step_end = make_timeout_time_ms(s.duration_ms);
                absolute_time_t last_update = get_absolute_time();
                
                while (get_absolute_time() < step_end) {
                    absolute_time_t t_now = get_absolute_time();
                    if (absolute_time_diff_us(last_update, t_now) >= (UPDATE_TIME_MS * 1000)) {
                        lsm6ds3_read_all();
                        update_counts();
                        
                        // Update angle estimate even if not balancing
                        integrateGyro();
                        integrateEncoders();
                        
                        if (is_balancing) {
                            balance_update();  // This calculates risingAngleOffset
                        } else {
                            // Calculate trajectory error for recording even if not balancing
                            risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;
                        }
                        
                        // Calculate actual motor commands for recording
                        int32_t distanceDiff = distanceLeft - distanceRight;
                        int16_t leftMotor = motorSpeed + (distanceDiff * DISTANCE_DIFF_RESPONSE) / 100;
                        int16_t rightMotor = motorSpeed - (distanceDiff * DISTANCE_DIFF_RESPONSE) / 100;
                        
                        // Only record if recording is active
                        if (is_recording && record_index < MAX_RECORD_SAMPLES) {
                            DataPoint *p = &record_buffer[record_index];
                            p->timestamp_us = to_us_since_boot(t_now);
                            p->ax = ax;                         // Accelerometer X
                            p->ay = az;                         // Accelerometer Z
                            p->az = gy;                         // Gyro Y (raw)
                            p->gx = leftMotor;                  // Left motor command (actual)
                            p->gy = rightMotor;                 // Right motor command (actual)
                            p->gz = (int16_t)angle;             // Angle (degrees)
                            p->left_enc = left_total;
                            p->right_enc = right_total;
                            p->motor_cmd = motorSpeed;          // Base motor command
                            p->trajectory_error = (int16_t)risingAngleOffset;  // Trajectory error
                            record_index++;
                        }
                        last_update = t_now;
                    }
                    tight_loop_contents();
                }
                break;
        }
    }
    balboa_set_speeds(0, 0);
    is_balancing = false;
    is_recording = false;  // Stop recording at end
    printf("SEQ: Complete. Recorded %d samples.\n", record_index);
    bt_printf("SEQ: Complete.\n");
}

int main() {
    stdio_init_all();
    gpio_init(ENABLE_PIN); gpio_set_dir(ENABLE_PIN, GPIO_OUT); gpio_put(ENABLE_PIN, 1);
    gpio_init(L_X_PIN); gpio_set_dir(L_X_PIN, GPIO_IN); gpio_pull_down(L_X_PIN);
    gpio_init(L_B_PIN); gpio_set_dir(L_B_PIN, GPIO_IN); gpio_pull_down(L_B_PIN);
    gpio_init(R_X_PIN); gpio_set_dir(R_X_PIN, GPIO_IN); gpio_pull_down(R_X_PIN);
    gpio_init(R_B_PIN); gpio_set_dir(R_B_PIN, GPIO_IN); gpio_pull_down(R_B_PIN);

    setup_pio_encoders();
    setup_dma_encoders();
    lsm6ds3_init();
    init_motor_i2c();
    balboa_set_speeds(0, 0);
    
    sequence[0] = (Step){CMD_DRIVE, 100, 100, 100, 0, 0};
    sequence[1] = (Step){CMD_DRIVE, 1000, 0, 0, 0, 0};
    sequence_length = 2;
    
    if (cyw43_arch_init()) return -1;
    l2cap_init();
    rfcomm_init();
    rfcomm_register_service(packet_handler, 1, 0xffff); 
    sdp_init();
    spp_create_sdp_record((uint8_t*)spp_service_buffer, 0x10001, 1, "Pico_Balboa");
    sdp_register_service((uint8_t*)spp_service_buffer);
    gap_set_local_name("Pico_Balboa");
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_discoverable_control(1);
    hci_power_control(HCI_POWER_ON);
    
    printf("--- RP2035 Ready ---\n");

    while (true) {
        // Skip USB/BT during critical recording period
        if (!is_recording) {
            check_usb_input();
            cyw43_arch_poll();
        }
        
        if (start_run_request) { start_run_request = false; run_blocking_sequence(); }
        if (start_read_request) { start_read_request = false; read_sequence(); }
        if (start_dma_dump_request) { start_dma_dump_request = false; dma_dump_sequence(); }
        if (start_show_seq_request) { start_show_seq_request = false; show_sequence(); }
        if (start_help_request) { start_help_request = false; print_help(); }
        sleep_ms(1);
    }
    return 0;
}