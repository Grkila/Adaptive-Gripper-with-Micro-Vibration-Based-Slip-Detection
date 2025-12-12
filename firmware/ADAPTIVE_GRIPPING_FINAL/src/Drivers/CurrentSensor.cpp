#include "CurrentSensor.h"
#include "../Globals.h"
#include <Arduino.h>

namespace CurrentSensor {
  
  // Static sensor instance
  static Adafruit_INA219 ina219;
  
  bool init() {
    Serial.println("[INA219] Initializing...");
    
    bool success = false;
    if (mutexI2C != NULL && xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
        success = ina219.begin();
        xSemaphoreGive(mutexI2C);
    }
    
    if (!success) {
      Serial.println("[INA219] ✗ FAILED - Check wiring!");
      return false;
    }
    Serial.println("[INA219] ✓ Ready");
    return true;
  }
  
  float readCurrent_mA() {
    if (mutexI2C == NULL) return 0.0f;
    
    float result = 0.0f;
    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(5)) == pdTRUE) {
        result = ina219.getCurrent_mA();
        xSemaphoreGive(mutexI2C);
    }
    return result;
  }
  
  Adafruit_INA219& getSensor() {
    return ina219;
  }
}

