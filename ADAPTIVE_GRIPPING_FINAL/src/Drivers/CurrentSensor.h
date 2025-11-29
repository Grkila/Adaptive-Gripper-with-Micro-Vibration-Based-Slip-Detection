#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <Adafruit_INA219.h>
#include "../Config.h"
#include "../Types.h"

// ============================================
// CURRENT SENSOR DRIVER (INA219)
// ============================================

namespace CurrentSensor {
  // Initialize the INA219 sensor
  bool init();
  
  // Read current in mA (raw, unfiltered)
  float readCurrent_mA();
  
  // Get sensor reference (for advanced usage)
  Adafruit_INA219& getSensor();
}

#endif // CURRENT_SENSOR_H

