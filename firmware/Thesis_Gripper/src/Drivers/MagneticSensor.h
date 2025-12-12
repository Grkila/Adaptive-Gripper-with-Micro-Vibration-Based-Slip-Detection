#ifndef MAGNETIC_SENSOR_H
#define MAGNETIC_SENSOR_H

#include <Wire.h>
#include "TLx493D_inc.hpp"
#include "../Config.h"
#include "../Types.h"

// ============================================
// MAGNETIC SENSOR DRIVER
// ============================================

namespace MagneticSensor {
  // Initialize the TLx493D sensor
  bool init();
  
  // Read raw magnetic field values (x, y, z in mT)
  bool read(double& x, double& y, double& z);
  
  // Calibrate sensor (determines offsets)
  void calibrate(CalibrationData& calData);
  
  // Apply calibration offsets to raw readings
  void applyCalibration(double& x, double& y, double& z, const CalibrationData& calData);
  
  // Calculate magnitude from x, y, z
  double calculateMagnitude(double x, double y, double z);
  
  // Get sensor reference (for advanced usage)
  TLx493D_t& getSensor();
}

#endif // MAGNETIC_SENSOR_H

