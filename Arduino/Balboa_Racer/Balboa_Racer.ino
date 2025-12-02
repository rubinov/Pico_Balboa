// Version: Nov 20, 2:09 AM

// This sketch sets the Pololu Balboa 32U4 (ATmega32U4) as an I2C Slave.
// It is configured for BIDIRECTIONAL communication and motor command execution.
// *** Motor logic is simplified to ignore Balboa's internal balancing state. ***

#include <Wire.h>
#include <Balboa32U4.h> 

// Define the I2C address for this slave device
const int SLAVE_ADDRESS = 0x42;

// Balboa components
Balboa32U4Motors motors;
Balboa32U4Encoders encoders; 

// Volatile global variables (Minimal use, only for command transport)
volatile int16_t received_val1 = 0;
volatile int16_t received_val2 = 0;
volatile uint8_t received_cmd = 0x00;

// Command 0x81 is used by Pico to signal motor activation
const uint8_t CMD_ACTIVATE_MOTORS = 0x81;
const uint8_t CMD_STATUS_ONLY = 0x01; 

void setup() {
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent); 
    
    // Safety setup
    motors.setSpeeds(0, 0); 
    ledRed(0);
    ledGreen(0);
    ledYellow(0);
    
    // Initial encoder reset
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
}

void loop() {
    // The loop is empty, ensuring maximum speed for ISR execution and no interference with motors.
}

// Function executes when Master (Pico) WRITES data (5 bytes expected)
void receiveEvent(int byteCount) {
    // 1. Check byte count and read data
    if (byteCount == 5 && Wire.available()) {
        received_cmd = Wire.read(); 

        // 2. Read and assemble two 16-bit motor speed values
        uint8_t v1_lsb = Wire.read();
        uint8_t v1_msb = Wire.read();
        received_val1 = (int16_t)((v1_msb << 8) | v1_lsb);
        
        uint8_t v2_lsb = Wire.read();
        uint8_t v2_msb = Wire.read();
        received_val2 = (int16_t)((v2_msb << 8) | v2_lsb);

        // MOTOR COMMAND LOGIC 
        // Directly set motor speeds if the command is activation (Bypasses Balboa safety checks)
        if (received_cmd == CMD_ACTIVATE_MOTORS) {
            motors.setSpeeds(received_val1, received_val2); 
            ledGreen(1); // GREEN LED indicates motor command received/executed
        } else {
            motors.setSpeeds(0, 0); 
            ledGreen(0);
        }
    } else {
        // ERROR: Incorrect byte count or wire available
        ledRed(1); 
    }
    
    // CRITICAL FIX: Consume ALL remaining bytes to clear the buffer
    while (Wire.available()) {
        Wire.read();
    }
}

// Function executes when Master (Pico) requests data (4 bytes to be sent)
void requestEvent() {
    
    ledYellow(1); // YELLOW LED indicates data request received

    // CRITICAL: Read and Reset the counts (atomic read and reset)
    int16_t counts_L = encoders.getCountsAndResetLeft(); 
    int16_t counts_R = encoders.getCountsAndResetRight();  
    
    // --- Assemble and Send Response (Total 4 bytes) ---
    byte response_buffer[4];

    // 1. Counts LEFT (2 bytes: Little-Endian)
    response_buffer[0] = (byte)(counts_L & 0xFF);      // LSB
    response_buffer[1] = (byte)((counts_L >> 8) & 0xFF); // MSB
    
    // 2. Counts RIGHT (2 bytes: Little-Endian)
    response_buffer[2] = (byte)(counts_R & 0xFF);     // LSB
    response_buffer[3] = (byte)((counts_R >> 8) & 0xFF); // MSB
    
    // Send the entire 4-byte buffer
    Wire.write(response_buffer, 4); 
    
    // Clear LED immediately after transmission
    ledYellow(0);
}