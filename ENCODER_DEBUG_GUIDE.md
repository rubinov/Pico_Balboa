# Encoder Debug Output Analysis Guide

## What This Debug Program Does

This modified version records the **raw 2-bit values** from the PIO before they're converted to counts. This will show you exactly what the hardware is seeing.

## Key Changes from Original

1. **Simplified Data Structure**: Instead of recording full IMU data, we only record:
   - `timestamp_us`: Time of sample
   - `left_raw`: Raw 2-bit value from left encoder PIO (0-3)
   - `right_raw`: Raw 2-bit value from right encoder PIO (0-3)
   - `left_total`: Running count after decoding
   - `right_total`: Running count after decoding
   - `motor_cmd`: Motor command for context

2. **Default Test Sequence**: 
   - Forward for 2 seconds
   - Stop for 0.5 seconds
   - Reverse for 2 seconds
   - Stop for 0.5 seconds

3. **CSV Output Format**:
   ```
   Idx,Time_us,L_Raw,R_Raw,L_Total,R_Total,MotorCmd
   ```

## What to Look For

### Expected Behavior for Quadrature Decoding

Since your PIO captures on the **falling edge of X**, at that instant X=0 always.
The B channel determines direction:

| Raw Value | Binary | X | B | Interpretation |
|-----------|--------|---|---|----------------|
| 0         | 0b00   | 0 | 0 | Forward motion |
| 1         | 0b01   | 0 | 1 | Reverse motion |
| 2         | 0b10   | 1 | 0 | **IMPOSSIBLE** (X can't be 1 at falling edge) |
| 3         | 0b11   | 1 | 1 | **IMPOSSIBLE** (X can't be 1 at falling edge) |

### What You'll Probably See (Current Bug)

Given that you're always counting up regardless of direction, I predict you'll see:

**Forward Motion:**
- Raw values: All 0s (0b00)
- Counts: Incrementing

**Reverse Motion:**
- Raw values: All 1s (0b01)  ← This should decrement but doesn't!
- Counts: Still incrementing (because code checks for val==2, not val==1)

### The Bug in Your Current Code

Lines 260-263 of your original code:
```c
if (val == 0) left_total++;      // 0b00 → increment
else if (val == 2) left_total--; // 0b10 → decrement
else left_errors++;
```

This is wrong because `val` can NEVER be 2 when sampling on the falling edge of X!

### The Fix (Don't Apply Yet)

The correct logic should be:
```c
if (val == 0) left_total++;      // 0b00 (B=0) → forward
else if (val == 1) left_total--; // 0b01 (B=1) → reverse
// val==2 or val==3 are impossible with falling-edge sampling
```

## How to Use This Debug Version

1. Compile and flash `Pico_Balboa_encoder_debug.c`
2. Connect via USB serial or Bluetooth
3. Type `RUN` to execute the default sequence
4. Type `READ` to download the CSV data
5. Analyze the `L_Raw` and `R_Raw` columns:
   - Are they only 0s and 1s? (Expected)
   - Do you see any 2s or 3s? (Would be strange)
   - Does `L_Raw` change between forward and reverse motion?
   - Does `R_Raw` change between forward and reverse motion?

## Mathematical Theory (Advanced Undergraduate Level)

### Quadrature Encoding State Machine

A quadrature encoder produces two square waves (A and B) 90° out of phase:

```
Forward:  A: ¯|_¯|_¯|_     State sequence: 00 → 10 → 11 → 01 → 00
          B: __|¯¯|__|     (clockwise Gray code)

Reverse:  A: ¯|_¯|_¯|_     State sequence: 00 → 01 → 11 → 10 → 00
          B: ¯¯|__|¯¯|     (counter-clockwise Gray code)
```

### Gray Code Property

The key insight is that consecutive states differ by only one bit (Gray code).
This property allows direction determination from any edge on either channel.

### Your Implementation (1X Decoding)

By sampling only on **falling edges of A (your X channel)**:

- You see only 2 of the 4 possible states
- You get 1 count per full cycle (1X decoding)
- Maximum count rate = encoder frequency / 4

At the falling edge of A:
- A must be 0 (by definition of falling edge)
- B's value determines direction:
  - B=0 → State 0b00 → Forward
  - B=1 → State 0b01 → Reverse

### Full 4X Decoding (For Reference)

To get 4X resolution, you'd need to:
1. Detect both edges of both channels
2. Track state transitions: {00→10, 10→11, 11→01, 01→00} = forward
3. This requires storing previous state (costs 1 PIO register)

Example PIO for 4X (for future reference):
```
.program quad_4x
    mov x, pins      ; Read current state
    mov y, isr       ; Get previous state from ISR
    jmp x!=y edge    ; Jump if state changed
    jmp loop         ; No change, loop
edge:
    ; Determine direction from old→new state transition
    ; (Requires lookup table or conditional logic)
```

This would consume more PIO instructions but give 4× the counts.

## Next Steps

After analyzing the debug output:

1. Confirm raw values are only 0 and 1
2. Note which value appears during forward vs reverse
3. Apply the corrected decoder logic:
   - Change `val == 2` to `val == 1` in update_counts()
4. Re-test to verify bidirectional counting

## Questions to Answer

1. Do both encoders show the same raw value patterns?
2. Is there any noise (unexpected transitions)?
3. What's the count rate at a given motor speed?
4. Do you need higher resolution (4X decoding)?
