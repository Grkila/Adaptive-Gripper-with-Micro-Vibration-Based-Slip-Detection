#include "CurrentSensor.h"
#include <Arduino.h>

namespace CurrentSensor {
  
  // Static sensor instance
  static Adafruit_INA219 ina219;
  
  bool init() {
    Serial.println("[INA219] Initializing...");
    if (!ina219.begin()) {
      Serial.println("[INA219] ✗ FAILED - Check wiring!");
      return false;
    }
    Serial.println("[INA219] ✓ Ready");
    return true;
  }
  
  float readCurrent_mA() {
    return ina219.getCurrent_mA();
  }
  
  Adafruit_INA219& getSensor() {
    return ina219;
  }
}

