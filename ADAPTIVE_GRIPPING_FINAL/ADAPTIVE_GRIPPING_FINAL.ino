// ============================================
// ADAPTIVE GRIPPING FINAL
// ============================================
// Modular architecture following Arduino IDE 2.0 "src" folder method
// All modules are in the src/ folder for clean organization
//
// Directory Structure:
// ADAPTIVE_GRIPPING_FINAL/
// ├── ADAPTIVE_GRIPPING_FINAL.ino  (this file - main loop)
// └── src/
//     ├── Config.h          - Pin definitions and constants
//     ├── Types.h           - Data structures
//     ├── Globals.h/cpp     - Shared state variables
//     ├── Drivers/          - Hardware abstraction
//     │   ├── MagneticSensor.h/cpp
//     │   ├── CurrentSensor.h/cpp
//     │   ├── ServoDriver.h/cpp
//     │   └── Buttons.h/cpp
//     └── Logic/            - Algorithms
//         ├── Filters.h/cpp
//         ├── FFTProcessor.h/cpp
//         ├── SlipDetection.h/cpp
//         ├── GrippingFSM.h/cpp
//         └── DebugTask.h/cpp
// ============================================

#include <Wire.h>
#include <WiFi.h>

// Include all modules
#include "src/Config.h"
#include "src/Types.h"
#include "src/Globals.h"
#include "src/Drivers/MagneticSensor.h"
#include "src/Drivers/CurrentSensor.h"
#include "src/Drivers/ServoDriver.h"
#include "src/Drivers/Buttons.h"
#include "src/Drivers/MotorDriver.h"
#include "src/Logic/Filters.h"
#include "src/Logic/FFTProcessor.h"
#include "src/Logic/SlipDetection.h"
#include "src/Logic/GrippingFSM.h"
#include "src/Logic/DebugTask.h"

// ============================================
// TIMER ISR
// ============================================
void ARDUINO_ISR_ATTR magneticSensor_ISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  newDataAvailable = true;
  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

// ============================================
// SETUP
// ============================================
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Disable WiFi and Bluetooth to save power
  WiFi.mode(WIFI_OFF);
  btStop();
  
  // Create I2C mutex FIRST
  mutexI2C = xSemaphoreCreateMutex();
  if (mutexI2C == NULL) {
    Serial.println("[FATAL] Failed to create I2C mutex!");
    while(1) delay(1000);
  }
  
  Serial.println("\n=== ADAPTIVE GRIPPING SYSTEM ===");
  Serial.println("Initializing modules...\n");
  
  // Initialize drivers
  ServoDriver::init();
  if (!MagneticSensor::init()) {
    Serial.println("[FATAL] Magnetic sensor init failed!");
    while (1) {Serial.println("Magnetic sensor init failed!"); delay(1000); }
  }

  if (!CurrentSensor::init()) {
    Serial.println("[FATAL] Current sensor init failed!");
    while (1) {Serial.println("Current sensor init failed!"); delay(1000); }
  }
  
  
  
  Buttons::init();
  MotorDriver::init();
  
  // Automatic Z Homing
  Serial.println("Starting Automatic Z Homing...");
  MotorDriver::runHomingRoutine();
  Serial.println("Z Homing Complete.");
  
  // Initialize logic modules
  Filters::init();
  
  // Calibrate magnetic sensor
  MagneticSensor::calibrate(calData);
  
  // Setup hardware timer for deterministic scan cycle
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(1000000);  // 1 MHz timer
  timerAttachInterrupt(timer, &magneticSensor_ISR);
  timerAlarm(timer, SCAN_INTERVAL_US, true, 0); 

  
  // Initialize debug task (runs on Core 0)
  DebugTask::init();
  
  // Clear serial buffer and sync
  Serial.flush();
  delay(500);
  
  Serial.println("\n=== System Ready ===");
  Serial.println("Scan cycle: " + String(SCAN_INTERVAL_US) + " us (" + String(MAGNETIC_SENSOR_SAMPLING_FREQUENCY, 0) + " Hz)");
  Serial.println();
}

// ============================================
// MAIN LOOP - DETERMINISTIC SCAN CYCLE
// ============================================
void loop() {
  // Execute scan cycle at fixed interval
  readInputs();
  processLogic();
  writeOutputs();
  
  // Update debug data (non-blocking - actual print runs on Core 0)
  DebugTask::updateData();
  
  // Yield to avoid starving other tasks/watchdog
  yield();
}

// ============================================
// SCAN CYCLE: READ INPUTS
// ============================================
void readInputs() {
  // Check if timer triggered new sample
  if (timerSemaphore != NULL && xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    currentSampleTime = micros();
    
    if (lastSampleTime > 0) {
      measuredInterval = currentSampleTime - lastSampleTime;
    }
    
    // Read magnetic sensor
    double raw_x, raw_y, raw_z;
    MagneticSensor::read(raw_x, raw_y, raw_z);
    
    // Apply calibration
    MagneticSensor::applyCalibration(raw_x, raw_y, raw_z, calData);
    
    // Store in magnetic data structure
    magData.x = raw_x;
    magData.y = raw_y;
    magData.z = raw_z;
    magData.magnitude = MagneticSensor::calculateMagnitude(raw_x, raw_y, raw_z);
    
    // Apply 500 Hz low-pass filter
    Filters::applyMainFilterMagneticSensor(magData);
    
    // Apply 30 Hz low-pass and calculate high-pass
    Filters::applyBandSplitFilterMagneticSensor(magData);
    
    newDataAvailable = false;
    
    // Update scan timing for debug
    unsigned long scan_duration = currentSampleTime - lastSampleTime;
    bool time_exceeded = (scan_duration >= SCAN_INTERVAL_US + SCAN_INTERVAL_US / 4);
    
    // Guard against null mutex (created in DebugTask::init)
    if (mutexSlipData != NULL && xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(0)) == pdTRUE) {
      debugData.scan_time_us = scan_duration;
      debugData.scan_time_exceeded = time_exceeded;
      xSemaphoreGive(mutexSlipData);
    }
    
    lastSampleTime = currentSampleTime;
  }
  // Run background tasks when not processing high-speed data
  else {
    readLowPriorityInputs();
  }
}

// ============================================
// HELPER: LOW PRIORITY INPUTS (POLLING)
// ============================================
void readLowPriorityInputs() {
  unsigned long now = millis();
  
  // 1. Current Sensor (100Hz)
  if (now - last_current_read > CURRENT_READ_INTERVAL_MS) {
    last_current_read = now;
    float raw_current = CurrentSensor::readCurrent_mA();
    current_mA = Filters::filterCurrent(raw_current);
  }

  // 2. Buttons (20Hz)
  static unsigned long last_button_read = 0;
  if (now - last_button_read > BUTTON_READ_INTERVAL_MS) {
    last_button_read = now;
    buttons = Buttons::read();
  }
}

// ============================================
// SCAN CYCLE: PROCESS LOGIC
// ============================================
void processLogic() {
  // Process FFT on high-pass filtered data
  FFTProcessor::process(magData);
  
  // Detect slip from FFT results
  SlipDetection::detect();
  
  // Process gripping state machine
  GrippingFSM::process(buttons, current_mA, magData.magnitude);

  // Process Lift Control (TMC2209) - Speed Mode
  // Only call if button state changed to avoid flooding with calls
  static bool lastBtn3 = false, lastBtn4 = false, lastBtn5 = false;
  
  // Button 3: Up
  if (buttons.button_3 && !lastBtn3) {
      MotorDriver::moveToMM(100);
  }
  lastBtn3 = buttons.button_3;
 
  // Button 4: Down
  if (buttons.button_4 && !lastBtn4) {
      MotorDriver::moveToMM(1);
  }
  lastBtn4 = buttons.button_4;
  
  // Button 5: Stop
  if (buttons.button_5 && !lastBtn5) {
      MotorDriver::setTargetSpeed(0);
  }
  lastBtn5 = buttons.button_5;
}

// ============================================
// SCAN CYCLE: WRITE OUTPUTS
// ============================================
void writeOutputs() {
  // Write motor position (only if changed)
  ServoDriver::writePositionIfChanged(servo_position);
}

