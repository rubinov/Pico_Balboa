# Build Instructions for Encoder Debug Version

## Files Needed

Place these files in your project directory:
1. `Pico_Balboa_encoder_debug.c` - Main debug program
2. `CMakeLists.txt` - Build configuration (the debug version)
3. `encoders.pio` - PIO assembly (unchanged from original)
4. `pico_sdk_import.cmake` - SDK import file (from original project)

## Build Steps

### From Command Line

```bash
# Create build directory
mkdir build
cd build

# Configure
cmake ..

# Build
make -j4

# Flash to Pico
# Method 1: Hold BOOTSEL, drag and drop the .uf2 file
# Method 2: Using picotool
picotool load Pico_Balboa_encoder_debug.uf2
picotool reboot
```

### From VS Code with Pico Extension

1. Open project folder
2. Click "Configure CMake" (if needed)
3. Click "Build" 
4. Hold BOOTSEL button on Pico, connect USB
5. Click "Flash" or drag `.uf2` file to mounted drive

## Serial Monitor Connection

### USB Serial
```bash
# Linux/Mac
screen /dev/ttyACM0 115200

# Windows (PowerShell with putty)
putty -serial COM3 -serspeed 115200

# Or use VS Code serial monitor extension
```

### Bluetooth Serial
1. Pair with "Pico_Balboa_Debug"
2. Connect using Serial Bluetooth Terminal app or similar
3. Android: "Serial Bluetooth Terminal" by Kai Morich
4. iOS: "Bluetooth Terminal" or similar

## Testing Procedure

1. **Connect and verify**:
   ```
   Type: HELP
   ```

2. **Check default sequence**:
   ```
   Type: SHOW_SEQ
   ```
   Should show 4 steps: FWD → STOP → REV → STOP

3. **Run test sequence**:
   ```
   Type: RUN
   ```
   Watch the robot move forward, stop, reverse, stop.

4. **Download debug data**:
   ```
   Type: READ
   ```
   Save the CSV output to a file for analysis.

5. **Analyze raw values**:
   Look at the `L_Raw` and `R_Raw` columns:
   - Should only contain values 0 or 1
   - Forward motion: probably all 0s
   - Reverse motion: probably all 1s
   - If counts only go up, this confirms the decoder bug!

## Optional: Custom Test Sequence

Create your own test:
```
STEP 0, 1000, 50, 50      # 1 second at speed 50 (both motors)
STEP 1, 500, 0, 0         # 0.5 second stop
STEP 2, 1000, -50, -50    # 1 second reverse at speed -50
SHOW_SEQ                   # Verify
RUN                        # Execute
READ                       # Download data
```

## Expected Output Format

```csv
Idx,Time_us,L_Raw,R_Raw,L_Total,R_Total,MotorCmd
0,1234567,0,0,0,0,100
1,1235567,0,0,1,1,100
2,1236567,0,0,2,2,100
...
```

## Troubleshooting

**No serial output:**
- Verify USB cable supports data (not just power)
- Check COM port is correct
- Try resetting the Pico

**Bluetooth won't connect:**
- Check device is discoverable ("Pico_Balboa_Debug")
- Try forgetting and re-pairing
- Ensure no other device is already connected

**Motors don't move:**
- Check ENABLE_PIN (GPIO 20) is connected
- Verify I2C connections to motor controller
- Test with lower speeds first

**DMA buffer fills too fast:**
- Reduce motor speed
- Shorten test duration
- The buffer is circular, so it won't overflow, but you might miss transitions

## Files Generated

After a successful build, you'll find in `build/`:
- `Pico_Balboa_encoder_debug.uf2` - Flash this to your Pico
- `Pico_Balboa_encoder_debug.elf` - Debug symbols
- `Pico_Balboa_encoder_debug.bin` - Raw binary
