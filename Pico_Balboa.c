#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h" 
#include "encoders.pio.h" 
#include "pico/time.h"

// --- PIN ASSIGNMENTS ---
#define L_X_PIN 0   
#define L_B_PIN 1   
#define R_X_PIN 26  
#define R_B_PIN 27  
#define ENABLE_PIN 20  

// --- DMA BUFFER CONFIGURATION ---
#define DMA_BUFFER_SIZE 1024 
#define DMA_BUFFER_MASK (DMA_BUFFER_SIZE - 1) 

// --- GLOBALS ---
PIO pio = pio0;
uint sm_left = 0;
uint sm_right = 1;

// DMA Channels
int dma_chan_left;
int dma_chan_right;

// DMA Buffers - CRITICAL FIX HERE
// 1. Aligned to 4096 bytes for hardware ring wrapping.
// 2. Placed in ".non_cached_bss" section so CPU sees DMA writes immediately.
// 3. Attributes placed at the END to satisfy GCC syntax requirements.
volatile int32_t left_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096), section(".non_cached_bss")));
volatile int32_t right_dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(4096), section(".non_cached_bss")));

// CPU Read Indices
uint32_t left_read_index = 0;
uint32_t right_read_index = 0;

// Total Counts
volatile int32_t left_count = 0;
volatile int32_t right_count = 0;

// --- DMA SETUP ---
void setup_dma_encoders() {
    // --- LEFT ENCODER DMA ---
    dma_chan_left = dma_claim_unused_channel(true);
    dma_channel_config c_left = dma_channel_get_default_config(dma_chan_left);
    
    channel_config_set_transfer_data_size(&c_left, DMA_SIZE_32); 
    channel_config_set_read_increment(&c_left, false); 
    channel_config_set_write_increment(&c_left, true); 
    channel_config_set_dreq(&c_left, pio_get_dreq(pio, sm_left, false)); 
    
    channel_config_set_ring(&c_left, true, 12); // Wrap every 4096 bytes

    dma_channel_configure(
        dma_chan_left,
        &c_left,
        left_dma_buffer,        
        &pio->rxf[sm_left],     
        0xFFFFFFFF,             
        true                    
    );

    // --- RIGHT ENCODER DMA ---
    dma_chan_right = dma_claim_unused_channel(true);
    dma_channel_config c_right = dma_channel_get_default_config(dma_chan_right);
    
    channel_config_set_transfer_data_size(&c_right, DMA_SIZE_32);
    channel_config_set_read_increment(&c_right, false);
    channel_config_set_write_increment(&c_right, true);
    channel_config_set_dreq(&c_right, pio_get_dreq(pio, sm_right, false));
    channel_config_set_ring(&c_right, true, 12); 

    dma_channel_configure(
        dma_chan_right,
        &c_right,
        right_dma_buffer,       
        &pio->rxf[sm_right],    
        0xFFFFFFFF,             
        true                    
    );
}

// --- HARVESTER FUNCTION ---
void update_encoder_counts() {
    // --- Update Left ---
    uint32_t current_write_addr = (uint32_t)dma_hw->ch[dma_chan_left].write_addr;
    uint32_t left_write_index = (current_write_addr - (uint32_t)left_dma_buffer) / 4;
    left_write_index &= DMA_BUFFER_MASK; 

    while (left_read_index != left_write_index) {
        left_count += left_dma_buffer[left_read_index];
        left_read_index = (left_read_index + 1) & DMA_BUFFER_MASK;
    }

    // --- Update Right ---
    uint32_t current_write_addr_r = (uint32_t)dma_hw->ch[dma_chan_right].write_addr;
    uint32_t right_write_index = (current_write_addr_r - (uint32_t)right_dma_buffer) / 4;
    right_write_index &= DMA_BUFFER_MASK;

    while (right_read_index != right_write_index) {
        right_count += right_dma_buffer[right_read_index];
        right_read_index = (right_read_index + 1) & DMA_BUFFER_MASK;
    }
}

void setup_pio_encoders() {
    uint offset = pio_add_program(pio, &simple_quad_counter_program);
    simple_quad_counter_program_init(pio, sm_left, offset, L_X_PIN, L_B_PIN);
    simple_quad_counter_program_init(pio, sm_right, offset, R_X_PIN, R_B_PIN);
}

// --- MAIN APPLICATION ---

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("RP2035 Encoder Application (Fixed Cache Syntax)\n");

    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 1);
    
    gpio_init(L_X_PIN); gpio_set_dir(L_X_PIN, GPIO_IN); gpio_pull_down(L_X_PIN);
    gpio_init(L_B_PIN); gpio_set_dir(L_B_PIN, GPIO_IN); gpio_pull_down(L_B_PIN);
    gpio_init(R_X_PIN); gpio_set_dir(R_X_PIN, GPIO_IN); gpio_pull_down(R_X_PIN);
    gpio_init(R_B_PIN); gpio_set_dir(R_B_PIN, GPIO_IN); gpio_pull_down(R_B_PIN);

    setup_pio_encoders();
    setup_dma_encoders();
    
    printf("--- Setup Complete ---\n");
    printf("Reading @ 100Hz, Printing @ 10Hz\n");

    absolute_time_t next_loop_time = get_absolute_time();
    int64_t loop_interval_us = 10000; // 10 milliseconds (100 Hz)
    int print_decimator = 0;

    while (true) {
        next_loop_time = delayed_by_us(next_loop_time, loop_interval_us);
        best_effort_wfe_or_timeout(next_loop_time);

        update_encoder_counts();

        print_decimator++;
        if (print_decimator >= 10) {
            printf("Left Count: %8d | Right Count: %8d\n", left_count, right_count);
            print_decimator = 0;
        }
    }

    return 0;
}