#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "Config.h"
#include "Types.h"

// ============================================
// EXTERN DECLARATIONS FOR SHARED STATE
// ============================================

// Timing
extern volatile bool newDataAvailable;
extern unsigned long lastSampleTime;
extern unsigned long currentSampleTime;
extern int measuredInterval;

// Magnetic field data
extern MagneticData magData;

// Calibration data
extern CalibrationData calData;

// Current measurement
extern float current_mA;
extern unsigned long last_current_read;

// Button state
extern ButtonState buttons;

// Gripping state
extern GrippingMode gripping_mode;
extern int servo_position;
extern unsigned long last_reaction_time;
extern unsigned long last_slip_or_entry_time;
extern unsigned long last_backoff_time;

// Slip detection
extern bool slip_flag;
extern float slip_indicator;

// FFT instances
extern AxisFFT fftX_high_pass;
extern AxisFFT fftY_high_pass;
extern AxisFFT fftZ_high_pass;
extern AxisFFT fftMagnitude_high_pass;
extern AxisFFT fftX_low_pass;
extern AxisFFT fftY_low_pass;
extern AxisFFT fftZ_low_pass;
extern AxisFFT fftMagnitude_low_pass;

// Debug data (volatile for ISR safety)
extern volatile DebugData debugData;

// FreeRTOS synchronization
extern SemaphoreHandle_t mutexSlipData;
extern SemaphoreHandle_t mutexFFTData;
extern volatile SemaphoreHandle_t timerSemaphore;
extern portMUX_TYPE timerMux;

// Debug task handle
extern TaskHandle_t debugTaskHandle;

// Timer
extern hw_timer_t* timer;

#endif // GLOBALS_H

