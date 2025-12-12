# Debug System Commands

**Baud Rate:** 921600  
**Format:** JSON  
**Terminator:** Newline (`\n`)

## Configuration Commands

Send these JSON strings to the Serial port to toggle specific data streams.

### FFT Mode (Exclusive)
*   `{"fft": true}` - Enables exclusive FFT streaming mode. (Pauses other metrics)
*   `{"fft": false}` - Disables FFT mode and returns to normal telemetry.

### Telemetry Toggles
*   `{"mag_lowpass": true}` / `false` - Low-Pass Filtered Magnetic Data (`mlx`, `mly`, `mlz`, `mag`). Alias: `mag_filtered`
*   `{"mag_highpass": true}` / `false` - High-Pass Filtered Magnetic Data (`mhx`, `mhy`, `mhz`)
*   `{"mag_raw": true}` / `false` - Raw Magnetic Data (`rmx`, `rmy`, `rmz`)
*   `{"current": true}` / `false` - Current Sensor (`cur`)
*   `{"slip": true}` / `false` - Slip Detection (`slip`, `s_ind`)
*   `{"servo": true}` / `false` - Servo & Mode (`srv`, `grp`)
*   `{"system": true}` / `false` - System Timing (`t`)

## Output Formats

### Normal Telemetry
Returns a JSON object with enabled fields:
```json
{"mx":12.3,"my":45.6,"mz":78.9,"mag":90.1,"cur":10.5,"slip":0,"t":500}
```

### FFT Data
Streamed when FFT mode is enabled:
```json
{"type":"fft","data":[10.5, 2.3, 0.1, ...]}
```
*   **data**: Array of magnitude values (High-Pass Filtered)

