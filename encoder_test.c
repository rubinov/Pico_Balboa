// Generated: November 21, 2024, 5:45 AM CST
// Encoder Test - Raw Pin State Sampler for Balboa Robot
// Pico 2 W quadrature encoder diagnostic tool

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Pin definitions - Balboa encoder connections
#define LEFT_ENC_B_PIN  0   // GP0 - Left encoder B channel (5V level-shifted)
#define LEFT_ENC_X_PIN  1   // GP1 - Left encoder A channel (5V level-shifted)
#define RIGHT_ENC_B_PIN 26  // GP26 - Right encoder B channel (5V level-shifted)
#define RIGHT_ENC_X_PIN 27  // GP27 - Right encoder A channel (5V level-shifted)
#define ENABLE_PIN      20  // GP20 - External power enable

int main() {
    // Initialize USB serial (baud rate handled by host terminal at 115200)
    stdio_init_all();
    
    // Wait for USB serial connection
    sleep_ms(2000);
    
    printf("--- STARTING RAW QUADRATURE SAMPLER (C Version) ---\n");
    printf("Baud rate: 115200 (set on host terminal)\n");
    
    // Enable external power via GP20
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 1);
    printf("GP20 set HIGH (power enabled).\n");
    
    // Initialize encoder pins as inputs with pull-down
    gpio_init(LEFT_ENC_B_PIN);
    gpio_set_dir(LEFT_ENC_B_PIN, GPIO_IN);
    gpio_pull_down(LEFT_ENC_B_PIN);
    
    gpio_init(LEFT_ENC_X_PIN);
    gpio_set_dir(LEFT_ENC_X_PIN, GPIO_IN);
    gpio_pull_down(LEFT_ENC_X_PIN);
    
    gpio_init(RIGHT_ENC_B_PIN);
    gpio_set_dir(RIGHT_ENC_B_PIN, GPIO_IN);
    gpio_pull_down(RIGHT_ENC_B_PIN);
    
    gpio_init(RIGHT_ENC_X_PIN);
    gpio_set_dir(RIGHT_ENC_X_PIN, GPIO_IN);
    gpio_pull_down(RIGHT_ENC_X_PIN);
    
    printf("\n--- Running Raw Pin State Sampler Loop ---\n");
    printf("T(ms) | L_B | L_X | R_B | R_X\n");
    printf("---------------------------------\n");
    
    // Store previous pin states for change detection
    uint8_t prev_LB = gpio_get(LEFT_ENC_B_PIN);
    uint8_t prev_LX = gpio_get(LEFT_ENC_X_PIN);
    uint8_t prev_RB = gpio_get(RIGHT_ENC_B_PIN);
    uint8_t prev_RX = gpio_get(RIGHT_ENC_X_PIN);
    
    while (true) {
        // Sample all encoder pins
        uint8_t curr_LB = gpio_get(LEFT_ENC_B_PIN);
        uint8_t curr_LX = gpio_get(LEFT_ENC_X_PIN);
        uint8_t curr_RB = gpio_get(RIGHT_ENC_B_PIN);
        uint8_t curr_RX = gpio_get(RIGHT_ENC_X_PIN);
        
        // Print only when any pin changes state
        if (curr_LB != prev_LB || curr_LX != prev_LX ||
            curr_RB != prev_RB || curr_RX != prev_RX) {
            
            printf("T: %7lu | %3d | %3d | %3d | %3d\n",
                   to_ms_since_boot(get_absolute_time()),
                   curr_LB, curr_LX, curr_RB, curr_RX);
            
            // Update state tracking
            prev_LB = curr_LB;
            prev_LX = curr_LX;
            prev_RB = curr_RB;
            prev_RX = curr_RX;
        }
        
        // Fast polling - 100us sample period
        sleep_us(100);
    }
    
    return 0;
}