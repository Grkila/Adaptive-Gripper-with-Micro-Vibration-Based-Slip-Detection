#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================
// PIN DEFINITIONS
// ============================================
constexpr int SERVO_PIN = 12;
constexpr int ANALOG_FEEDBACK_PIN = 13;
constexpr int MAGNETIC_SENSOR_POWER_PIN = 14;
constexpr int BUTTON_1_PIN = 16;
constexpr int BUTTON_2_PIN = 23;
constexpr int BUTTON_3_PIN = 22;
constexpr int BUTTON_4_PIN = 21;
constexpr int BUTTON_5_PIN = 19;
constexpr int SDA_PIN=26;
constexpr int SCL_PIN=27;

// ============================================
// SERVO CONFIGURATION
// ============================================
constexpr int SERVO_FULLY_OPEN = 180;
constexpr int SERVO_FULLY_CLOSED = 0;

// ============================================
// TIMING CONFIGURATION
// ============================================
constexpr unsigned long SCAN_INTERVAL_US = 500;  // 500us scan cycle (2kHz)
constexpr unsigned long CURRENT_READ_INTERVAL_MS = 10; // 10ms (100Hz)
constexpr unsigned long BUTTON_READ_INTERVAL_MS = 50;  // 50ms (20Hz) for button debounce/polling

// ============================================
// SAMPLING AND FFT CONFIGURATION
// ============================================
constexpr uint16_t FFT_SAMPLES = 128;  // Must be power of 2
constexpr double MAGNETIC_SENSOR_SAMPLING_FREQUENCY = 1000000.0 / (double)SCAN_INTERVAL_US;

// ============================================
// FILTER CONFIGURATION
// ============================================
#ifndef PI
#define PI 3.14159
#endif

// 30 Hz low-pass filter cutoff
constexpr double FILTER_30HZ_CUTOFF_FREQ = 30.0;

// 500 Hz low-pass filter cutoff (main filter)
constexpr double FILTER_500HZ_CUTOFF_FREQ = 500.0;

// Current filter (5 Hz at 100 Hz sampling)
constexpr double FILTER_CURRENT_CUTOFF_FREQ = 5.0;
constexpr double FILTER_CURRENT_SAMPLE_RATE = 100.0;


// ============================================
// SLIP DETECTION CONFIGURATION
// ============================================
constexpr float SLIP_THRESHOLD = 250.0f;
constexpr uint16_t SLIP_FREQ_START_HZ = 40;
constexpr uint16_t SLIP_FREQ_END_HZ = 300;

// ============================================
// GRIPPING STATE MACHINE TIMING
// ============================================
constexpr unsigned long REACTION_COOLDOWN_MS = 74;
constexpr unsigned long BACKOFF_DELAY_MS = 2000;
constexpr unsigned long BACKOFF_INTERVAL_MS = 1000;

// ============================================
// GRIPPING THRESHOLDS
// ============================================
constexpr float GRIP_CURRENT_THRESHOLD_MA = 8.0f;
constexpr float GRIP_MAGNITUDE_THRESHOLD = 5.0f;

// ============================================
// MOTOR SPEED SETTINGS
// ============================================
constexpr int GRASPING_STEP = 2;
constexpr int OPENING_STEP = 5;
constexpr int BACKOFF_STEP = 1;

// ============================================
// DEBUG TASK CONFIGURATION
// ============================================
constexpr unsigned long DEBUG_PRINT_INTERVAL_MS = 120;
constexpr uint32_t DEBUG_TASK_STACK_SIZE = 8192;
constexpr UBaseType_t DEBUG_TASK_PRIORITY = 1;
constexpr BaseType_t DEBUG_TASK_CORE = 0;

// ============================================
// I2C CONFIGURATION
// ============================================
constexpr uint32_t MAGNETIC_I2C_CLOCK_SPEED = 1000000;  // 400 kHz Fast Mode (reduced from 1MHz for stability)

// ============================================
// TMC2209 MOTOR CONFIGURATION
// ============================================
constexpr int TMC_RX_PIN = 5;    // TMC2209 PDN_UART
constexpr int TMC_TX_PIN = 17;    // Connect to RX via 1k Resistor
//constexpr int TMC_DIAG_PIN = 15;  // StallGuard output
constexpr int TMC_EN_PIN = 18;    // Enable pin

constexpr float TMC_R_SENSE = 0.11f;
constexpr uint8_t TMC_DRIVER_ADDR = 0b00;
constexpr int TMC_STALL_VALUE = 3; // Stall Sensitivity (0-255)

constexpr int TMC_RUN_CURRENT = 600; // mA
constexpr int TMC_MICROSTEPS = 1;     // 1/16 if not set, but user example says 1? user example: driver.microsteps(1); // 1/16 Microstepping. Wait, 1 means full step usually? No, in TMCStepper library logic depends. 0=microstepping disabled? No. 
// Library: microsteps(uint16_t msteps). 0=256, 1=128?? No, standard is 0, 2, 4, 8, 16 etc.
// The user example says: driver.microsteps(1); // 1/16 Microstepping. This comment might be wrong or using some specific override. I will copy the user example value.

// Speed Settings
constexpr int TMC_MAX_SPEED = 100000;      // VACTUAL units
constexpr int TMC_ACCELERATION = 2000;     // Speed increment per loop cycle
constexpr int TMC_TASK_DELAY_MS = 10;      // Update loop frequency (approx 100Hz)
constexpr BaseType_t TMC_TASK_CORE = 0;    // Run on Core 1

#endif // CONFIG_H

