#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <ESP32Servo.h>
#include "../Config.h"

// ============================================
// SERVO DRIVER
// ============================================

namespace ServoDriver {
  // Initialize the servo
  void init();
  
  // Write position to servo (0-180)
  void writePosition(int position);
  
  // Write position only if changed (optimized)
  void writePositionIfChanged(int position);
  
  // Get current written position
  int getCurrentPosition();
}

#endif // SERVO_DRIVER_H

