#include "ServoDriver.h"
#include <Arduino.h>

namespace ServoDriver {
  
  // Static servo instance
  static Servo servo;
  static int lastWrittenPosition = -999;
  
  void init() {
    Serial.println("[SERVO] Initializing...");
    servo.attach(SERVO_PIN);
    servo.write(SERVO_FULLY_OPEN);
    lastWrittenPosition = SERVO_FULLY_OPEN;
    Serial.println("[SERVO] ✓ Ready");
  }
  
  void writePosition(int position) {
    servo.write(position);
    lastWrittenPosition = position;
  }
  
  void writePositionIfChanged(int position) {
    if (position != lastWrittenPosition) {
      servo.write(position);
      lastWrittenPosition = position;
    }
  }
  
  int getCurrentPosition() {
    return lastWrittenPosition;
  }
}

