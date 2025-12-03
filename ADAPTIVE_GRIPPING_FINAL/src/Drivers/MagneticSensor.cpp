#include "MagneticSensor.h"
#include "../Globals.h"
#include <Arduino.h>

namespace MagneticSensor {
  
  // Static sensor instance
  static TLx493D_t sensor;
  
  bool init() {
    // Power up sensor
    pinMode(MAGNETIC_SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(MAGNETIC_SENSOR_POWER_PIN, LOW);
    delay(100);
    digitalWrite(MAGNETIC_SENSOR_POWER_PIN, HIGH);
    delay(100);
    
    // Initialize I2C
    if (mutexI2C != NULL && xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
      Wire.begin(SDA_PIN, SCL_PIN);
      Wire.setClock(MAGNETIC_I2C_CLOCK_SPEED);
      xSemaphoreGive(mutexI2C);
      Serial.println("[I2C] Initialized at 400kHz");
    } else {
       Serial.println("[I2C] Failed to take mutex for init!");
       return false;
    }
    
    // Initialize sensor structure
    if (!tlx493d_init(&sensor, TLx493D_A1B6_e)) {
      Serial.println("[SENSOR] ERROR: tlx493d_init failed!");
      return false;
    }
    Serial.println("[SENSOR] Structure initialized");
    
    // Initialize communication
    bool comm_ok = false;
    if (mutexI2C != NULL && xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        comm_ok = ifx::tlx493d::initCommunication(&sensor, Wire, TLx493D_IIC_ADDR_A0_e, true);
        xSemaphoreGive(mutexI2C);
    }
    if (!comm_ok) {
      Serial.println("[SENSOR] ERROR: initCommunication failed!");
      return false;
    }
    Serial.println("[SENSOR] Communication initialized");
    
    // Set default config
    bool config_ok = false;
    if (mutexI2C != NULL && xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        config_ok = tlx493d_setDefaultConfig(&sensor);
        xSemaphoreGive(mutexI2C);
    }
    if (!config_ok) {
      Serial.println("[SENSOR] ERROR: setDefaultConfig failed!");
      return false;
    }
    Serial.println("[SENSOR] Default config set");
    
    // Configure for fast mode
    if (mutexI2C != NULL && xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        tlx493d_setPowerMode(&sensor, TLx493D_FAST_MODE_e);
        tlx493d_setMeasurement(&sensor, TLx493D_BxByBz_e);
        xSemaphoreGive(mutexI2C);
    }
    
    delay(100);
    Serial.println("[SENSOR] ✓ Ready");
    return true;
  }
  
  bool read(double& x, double& y, double& z) {
    if (mutexI2C == NULL) return false;
    
    bool result = false;
    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(2)) == pdTRUE) {
      result = tlx493d_getMagneticField(&sensor, &x, &y, &z);
      xSemaphoreGive(mutexI2C);
    }
    return result;
  }
  
  void calibrate(CalibrationData& calData) {
    Serial.println("\n=== Starting Calibration ===");
    Serial.println("Reading sensor for 1 second...");
    
    double x, y, z;
    bool first_reading = true;
    unsigned long start_time = millis();
    int reading_count = 0;
    
    while (millis() - start_time < 1000) {
      if (read(x, y, z)) {
        reading_count++;
        
        if (first_reading) {
          calData.x_min = calData.x_max = x;
          calData.y_min = calData.y_max = y;
          calData.z_min = calData.z_max = z;
          first_reading = false;
        } else {
          if (x < calData.x_min) calData.x_min = x;
          if (x > calData.x_max) calData.x_max = x;
          if (y < calData.y_min) calData.y_min = y;
          if (y > calData.y_max) calData.y_max = y;
          if (z < calData.z_min) calData.z_min = z;
          if (z > calData.z_max) calData.z_max = z;
        }
      }
    }
    
    // Calculate offset as average of min/max
    calData.x_offset = (calData.x_min + calData.x_max) / 2.0;
    calData.y_offset = (calData.y_min + calData.y_max) / 2.0;
    calData.z_offset = (calData.z_min + calData.z_max) / 2.0;
    
    Serial.println("=== Calibration Complete ===");
    Serial.print("Readings taken: ");
    Serial.println(reading_count);
    Serial.print("X offset: ");
    Serial.print(calData.x_offset, 3);
    Serial.print(" mT (range: ");
    Serial.print(calData.x_min, 3);
    Serial.print(" to ");
    Serial.print(calData.x_max, 3);
    Serial.println(")");
    Serial.print("Y offset: ");
    Serial.print(calData.y_offset, 3);
    Serial.print(" mT (range: ");
    Serial.print(calData.y_min, 3);
    Serial.print(" to ");
    Serial.print(calData.y_max, 3);
    Serial.println(")");
    Serial.print("Z offset: ");
    Serial.print(calData.z_offset, 3);
    Serial.print(" mT (range: ");
    Serial.print(calData.z_min, 3);
    Serial.print(" to ");
    Serial.print(calData.z_max, 3);
    Serial.println(")");
    Serial.println();
  }
  
  void applyCalibration(double& x, double& y, double& z, const CalibrationData& calData) {
    x -= calData.x_offset;
    y -= calData.y_offset;
    z -= calData.z_offset;
  }
  
  double calculateMagnitude(double x, double y, double z) {
    return sqrt(x * x + y * y + z * z);
  }
  
  TLx493D_t& getSensor() {
    return sensor;
  }
}

