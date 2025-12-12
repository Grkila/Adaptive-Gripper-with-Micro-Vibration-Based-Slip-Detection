/*
 * ADAPTIVE GRIPPING SYSTEM
 * Main control loop for the adaptive gripper project.
 */

#include <Wire.h>
#include <WiFi.h>

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

unsigned long cycleCounter = 0;
// Sampling dividers (base frequency 2kHz)
const int CURRENT_READ_DIVIDER = 20;  // ~100Hz
const int BUTTON_READ_DIVIDER = 100;  // ~20Hz

// Timer interrupt for sampling synchronization
void ARDUINO_ISR_ATTR magneticSensor_ISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  newDataAvailable = true;
  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(2000000);
  
  WiFi.mode(WIFI_OFF);
  btStop();
  
  mutexI2C = xSemaphoreCreateMutex();
  if (mutexI2C == NULL) {
    Serial.println("Error: Failed to create I2C mutex");
    while(1) delay(1000);
  }
  
  Serial.println("\n--- ADAPTIVE GRIPPING SYSTEM ---");
  
  // Hardware initialization
  ServoDriver::init();
  if (!MagneticSensor::init()) {
    Serial.println("Error: Magnetic sensor init failed");
    while (1) delay(1000);
  }

  if (!CurrentSensor::init()) {
    Serial.println("Error: Current sensor init failed");
    while (1) delay(1000);
  }
  
  Buttons::init();
  MotorDriver::init();
  
  Serial.println("Homing Z-axis...");
  MotorDriver::runHomingRoutine();
  Serial.println("Homing complete.");
  
  Filters::init();
  MagneticSensor::calibrate(calData);
  
  // Configure timer for determinstic loop
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &magneticSensor_ISR);
  timerAlarm(timer, SCAN_INTERVAL_US, true, 0); 

  DebugTask::init();
  
  Serial.flush();
  delay(500);
  Serial.println("System Ready.");
}

void readInputsSequentially() {
  // High priority: Magnetic sensor (2kHz)
  double raw_x=0, raw_y=0, raw_z=0;
  MagneticSensor::read(raw_x, raw_y, raw_z);
  MagneticSensor::applyCalibration(raw_x, raw_y, raw_z, calData);
  
  magData.x = raw_x;
  magData.y = raw_y;
  magData.z = raw_z;
  magData.magnitude = MagneticSensor::calculateMagnitude(raw_x, raw_y, raw_z);
  
  Filters::applyMainFilterMagneticSensor(magData);
  Filters::applyBandSplitFilterMagneticSensor(magData);

  // Low priority: Current sensing (sliced)
  if (cycleCounter % CURRENT_READ_DIVIDER == 0) {
      float raw_current = CurrentSensor::readCurrent_mA();
      current_mA = Filters::filterCurrent(raw_current);
  }

  // Low priority: UI Buttons
  if (cycleCounter % BUTTON_READ_DIVIDER == 0) {
      buttons = Buttons::read();
  }
}

void processLogic() {
  FFTProcessor::process(magData);
  SlipDetection::detect();
  GrippingFSM::process(buttons, current_mA, magData.magnitude);

  // Manual lift control
  static bool lastBtn3 = false, lastBtn4 = false, lastBtn5 = false;
  
  if (buttons.button_3 && !lastBtn3) {
      MotorDriver::moveToMM(150);
  }
  lastBtn3 = buttons.button_3;
 
  if (buttons.button_4 && !lastBtn4) {
      MotorDriver::moveToMM(0);
  }
  lastBtn4 = buttons.button_4;
  
  if (buttons.button_5 && !lastBtn5) {
      MotorDriver::setTargetSpeed(0);
  }
  lastBtn5 = buttons.button_5;
}

void writeOutputs() {
  ServoDriver::writePositionIfChanged(servo_position);
}

void loop() {
  // Wait for timer trigger to start cycle
  if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
    readInputsSequentially();
    processLogic();
    writeOutputs();

    cycleCounter++;
    DebugTask::updateData();
  }
}
