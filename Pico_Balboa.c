#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h" 
#include "hardware/i2c.h" // Added for LSM6DS3
#include "encoders.pio.h" 
#include "pico/time.h"

// --- PIN ASSIGNMENTS ---
#define L_X_PIN 0   
#define L_B_PIN 1   
#define R_X_PIN 26  
#define R_B_PIN 27  
#define ENABLE_PIN 20

// --- I2C / SENSOR PIN ASSIGNMENTS ---
#define I2C_PORT i2c1
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19
#define LSM6DS3_ADDR 0x6B

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

PIO pio = pio0;
uint sm_left = 0;
uint sm_right = 1;

int dma_chan_left;
int dma_chan_right;

// Buffers aligned for Ring Wrap
volatile int32_t left_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096)));
volatile int32_t right_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096)));

uint32_t left_read_index = 0;
uint32_t right_read_index = 0;

// --- APP STATE ---
volatile int32_t left_total = 0;
volatile int32_t right_total = 0;
volatile int32_t left_errors = 0;
volatile int32_t right_errors = 0;

// Sensor Data Storage
int16_t ax, ay, az;
int16_t gx, gy, gz;

// --- LSM6DS3 FUNCTIONS ---

void lsm6ds3_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_write_blocking(I2C_PORT, LSM6DS3_ADDR, buf, 2, false);
}

void lsm6ds3_init() {
    // Initialize I2C peripheral at 400kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    sleep_ms(10); // Short delay for sensor power up

    // Configuration sequence from lsm6ds3.py
    // Set Gyro to 208Hz
    lsm6ds3_write_reg(CTRL2_G, NORMAL_MODE_208HZ);
    // Set Accel to 208Hz
    lsm6ds3_write_reg(CTRL1_XL, NORMAL_MODE_208HZ);
    // Reset steps (pedometer)
    lsm6ds3_write_reg(CTRL10_C, RESET_STEPS);
    // Enable functional blocks
    lsm6ds3_write_reg(CTRL10_C, SET_FUNC_EN);
}

void lsm6ds3_read_all() {
    uint8_t reg = OUTX_L_G;
    uint8_t buffer[12];

    // Write register address (blocking, no stop bit)
    i2c_write_blocking(I2C_PORT, LSM6DS3_ADDR, &reg, 1, true);
    // Read 12 bytes of data
    i2c_read_blocking(I2C_PORT, LSM6DS3_ADDR, buffer, 12, false);

    // Combine bytes (Little Endian) into signed 16-bit integers
    // Note: C casts to int16_t automatically handle 2's complement for 16-bit values
    gx = (int16_t)((buffer[1] << 8) | buffer[0]);
    gy = (int16_t)((buffer[3] << 8) | buffer[2]);
    gz = (int16_t)((buffer[5] << 8) | buffer[4]);
    ax = (int16_t)((buffer[7] << 8) | buffer[6]);
    ay = (int16_t)((buffer[9] << 8) | buffer[8]);
    az = (int16_t)((buffer[11] << 8) | buffer[10]);
}

// --- ENCODER FUNCTIONS ---

void setup_dma_encoders() {
    // LEFT
    dma_chan_left = dma_claim_unused_channel(true);
    dma_channel_config c_left = dma_channel_get_default_config(dma_chan_left);
    channel_config_set_transfer_data_size(&c_left, DMA_SIZE_32); 
    channel_config_set_read_increment(&c_left, false); 
    channel_config_set_write_increment(&c_left, true); 
    channel_config_set_dreq(&c_left, pio_get_dreq(pio, sm_left, false)); 
    channel_config_set_ring(&c_left, true, 12); 
    dma_channel_configure(dma_chan_left, &c_left, left_dma_buffer, &pio->rxf[sm_left], 0xFFFFFFFF, true);

    // RIGHT
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
    // --- Process Left Buffer ---
    uint32_t l_curr_addr = (uint32_t)dma_hw->ch[dma_chan_left].write_addr;
    uint32_t l_write_idx = (l_curr_addr - (uint32_t)left_dma_buffer) / 4;
    l_write_idx &= DMA_BUFFER_MASK;

    while (left_read_index != l_write_idx) {
        int32_t val = left_dma_buffer[left_read_index] & 0x3; 
        
        if (val == 0) {         // 00 -> Increment
            left_total++;
        } else if (val == 2) {  // 10 -> Decrement
            left_total--;
        } else {
            left_errors++;      // Error
        }
        
        left_read_index = (left_read_index + 1) & DMA_BUFFER_MASK;
    }

    // --- Process Right Buffer ---
    uint32_t r_curr_addr = (uint32_t)dma_hw->ch[dma_chan_right].write_addr;
    uint32_t r_write_idx = (r_curr_addr - (uint32_t)right_dma_buffer) / 4;
    r_write_idx &= DMA_BUFFER_MASK;

    while (right_read_index != r_write_idx) {
        int32_t val = right_dma_buffer[right_read_index] & 0x3;
        
        if (val == 0) {         // 00 -> Increment
            right_total++;
        } else if (val == 2) {  // 10 -> Decrement
            right_total--;
        } else {
            right_errors++;     // Error
        }
        
        right_read_index = (right_read_index + 1) & DMA_BUFFER_MASK;
    }
}

void setup_pio_encoders() {
    uint offset = pio_add_program(pio, &simple_quad_counter_program);
    simple_quad_counter_program_init(pio, sm_left, offset, L_X_PIN, L_B_PIN);
    simple_quad_counter_program_init(pio, sm_right, offset, R_X_PIN, R_B_PIN);
}

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("RP2035 Sensor + Encoder Integration\n");

    // Power Enable
    gpio_init(ENABLE_PIN); gpio_set_dir(ENABLE_PIN, GPIO_OUT); gpio_put(ENABLE_PIN, 1);
    
    // Encoder Pins
    gpio_init(L_X_PIN); gpio_set_dir(L_X_PIN, GPIO_IN); gpio_pull_down(L_X_PIN);
    gpio_init(L_B_PIN); gpio_set_dir(L_B_PIN, GPIO_IN); gpio_pull_down(L_B_PIN);
    gpio_init(R_X_PIN); gpio_set_dir(R_X_PIN, GPIO_IN); gpio_pull_down(R_X_PIN);
    gpio_init(R_B_PIN); gpio_set_dir(R_B_PIN, GPIO_IN); gpio_pull_down(R_B_PIN);

    // Initialize Subsystems
    setup_pio_encoders();
    setup_dma_encoders();
    lsm6ds3_init();
    
    printf("--- Loop Started (Cycle Time Reporting) ---\n");

    absolute_time_t t_start, t_end;
    uint64_t cycle_duration;
    int report_decimator = 0;

    while (true) {
        // 1. Mark Start Time
        t_start = get_absolute_time();

        // 2. Task A: Read Sensors (Blocking I2C)
        lsm6ds3_read_all();

        // 3. Task B: Process Encoders (Memory Read)
        update_counts();

        // 4. Mark End Time & Calculate Duration
        t_end = get_absolute_time();
        cycle_duration = absolute_time_diff_us(t_start, t_end);

        // 5. Report Cycle Time (Decimated to ~2Hz for readability)
        // Note: We run as fast as possible, no artificial delays here.
        report_decimator++;
        if (report_decimator >= 2000) { 
            // If loop is fast (e.g., 500us), 2000 iters = ~1 sec
            printf("Cycle Time: %llu us\n", cycle_duration);
            report_decimator = 0;
        }
    }
    return 0;
}