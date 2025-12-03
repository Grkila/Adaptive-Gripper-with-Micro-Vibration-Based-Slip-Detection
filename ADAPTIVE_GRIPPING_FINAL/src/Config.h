#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================
// PIN DEFINITIONS
// ============================================
constexpr int SERVO_PIN = 12;
constexpr int ANALOG_FEEDBACK_PIN = 13;
constexpr int MAGNETIC_SENSOR_POWER_PIN = 14;
constexpr int BUTTON_1_PIN = 25;
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
constexpr float SLIP_THRESHOLD =40.0f;
constexpr uint16_t SLIP_FREQ_START_HZ = 80;
constexpr uint16_t SLIP_FREQ_END_HZ = 200;
constexpr int SLIP_DETECTION_IGNORE_CYCLES = 100; // Ignore slip for 50 cycles (25ms) after movement to filter vibration
constexpr float GRIP_SLIP_MARGIN_FALSE_POSITIVE=6.0f;
// ============================================
// GRIPPING STATE MACHINE TIMING
// ============================================
constexpr unsigned long REACTION_COOLDOWN_MS = 64;
constexpr unsigned long BACKOFF_DELAY_MS = 2000;
constexpr unsigned long BACKOFF_INTERVAL_MS = 1000;

// ============================================
// GRIPPING THRESHOLDS
// ============================================
constexpr float GRIP_CURRENT_THRESHOLD_MA = 0.0f;
constexpr float GRIP_MAGNITUDE_THRESHOLD = 1.2f;
constexpr float GRIP_MAGNITUDE_DROP_MARGIN = 0.2f;

// ============================================
// MOTOR SPEED SETTINGS
// ============================================
constexpr int GRASPING_STEP = 1;
constexpr int OPENING_STEP = 5;
constexpr int BACKOFF_STEP = 1;
constexpr int MAX_REACTION_STEPS = 4;
// ============================================
// DEBUG TASK CONFIGURATION
// ============================================
constexpr unsigned long DEBUG_PRINT_INTERVAL_MS = 1;
constexpr uint32_t DEBUG_TASK_STACK_SIZE = 8192;
constexpr UBaseType_t DEBUG_TASK_PRIORITY = 1;
constexpr BaseType_t DEBUG_TASK_CORE = 0;

// ============================================
// I2C CONFIGURATION
// ============================================
constexpr uint32_t MAGNETIC_I2C_CLOCK_SPEED = 1000000;  
// ============================================
// TMC2209 MOTOR CONFIGURATION
// ============================================
constexpr int TMC_RX_PIN = 5;    // TMC2209 PDN_UART
constexpr int TMC_TX_PIN = 17;    // Connect to RX via 1k Resistor
constexpr int TMC_EN_PIN = 18;    // Enable pin
constexpr int TMC_STEP_PIN = 16;
constexpr int TMC_DIR_PIN = 15;
constexpr float TMC_R_SENSE = 0.11f;
constexpr uint8_t TMC_DRIVER_ADDR = 0b00;
constexpr int TMC_STALL_VALUE = 3; // Stall Sensitivity (0-255)

constexpr int TMC_RUN_CURRENT = 1000; // mA
constexpr int TMC_MICROSTEPS = 16;     

// Speed Settings
constexpr int TMC_MAX_SPEED = 6000;      // Steps per second
constexpr int TMC_ACCELERATION = 5000;   // Steps per second^2

// Automatic Homing Configuration
constexpr int TMC_HOMING_CURRENT = 500;    // mA
constexpr int TMC_HOMING_SPEED = 5000;    // Steps per second
constexpr int TMC_HOMING_THRESHOLD = 3;   // StallGuard threshold for homing
constexpr int TMC_HOMING_CONSECUTIVE_STALLS = 3; // Number of consecutive stalls to confirm stall
constexpr int TMC_HOMING_DIRECTION = -1;    // 1 for forward/up, -1 for backward/down
constexpr int TMC_HOMING_TIMEOUT_MS = 1000000; // Safety timeout

constexpr float TMC_STEPS_PER_MM = 2560.0f; // https://blog.prusa3d.com/calculator_3416/#MotorStuffSPML 1.8deg motor m8 metric screw

#endif // CONFIG_H
