#include "Globals.h"

// ============================================
// GLOBAL VARIABLE DEFINITIONS
// ============================================

// Timing
volatile bool newDataAvailable = false;
unsigned long lastSampleTime = 0;
unsigned long currentSampleTime = 0;
int measuredInterval = 0;

// Magnetic field data
MagneticData magData = {0};

// Calibration data
CalibrationData calData = {0};

// Current measurement
float current_mA = 0.0f;
unsigned long last_current_read = 0;

// Button state
ButtonState buttons = {false, false, false, false, false};

// Gripping state
GrippingMode gripping_mode = GRIPPING_MODE_OPEN;
int servo_position = SERVO_FULLY_OPEN;
unsigned long last_reaction_time = 0;
unsigned long last_slip_or_entry_time = 0;
unsigned long last_backoff_time = 0;

// Slip detection
bool slip_flag = false;
float slip_indicator = 0.0f;

// FFT instances
AxisFFT fftX_high_pass("X_high_pass");
AxisFFT fftY_high_pass("Y_high_pass");
AxisFFT fftZ_high_pass("Z_high_pass");
AxisFFT fftMagnitude_high_pass("Magnitude_high_pass");
AxisFFT fftX_low_pass("X_low_pass");
AxisFFT fftY_low_pass("Y_low_pass");
AxisFFT fftZ_low_pass("Z_low_pass");
AxisFFT fftMagnitude_low_pass("Magnitude_low_pass");

// Debug data
volatile DebugData debugData = {false, 0.0f, 0, false, false};

// FreeRTOS synchronization
SemaphoreHandle_t mutexSlipData = NULL;
SemaphoreHandle_t mutexFFTData = NULL;
volatile SemaphoreHandle_t timerSemaphore = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Debug task handle
TaskHandle_t debugTaskHandle = NULL;

// Timer
hw_timer_t* timer = NULL;

