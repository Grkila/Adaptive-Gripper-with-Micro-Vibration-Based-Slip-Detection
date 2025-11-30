#include "GrippingFSM.h"
#include "SlipDetection.h"
#include <Arduino.h>

namespace GrippingFSM {
  
  void process(const ButtonState& buttons, float current_mA, float magnitude) {
    switch (gripping_mode) {
      
      case GRIPPING_MODE_OPEN:
        // Only start grasping when button is pressed
        if (buttons.button_1) {
          gripping_mode = GRIPPING_MODE_GRASPING;
        }
        break;
        
      case GRIPPING_MODE_GRASPING:
        // Gradually close the gripper
        if (millis() - last_reaction_time > REACTION_COOLDOWN_MS) {
          last_reaction_time = millis();
          servo_position -= GRASPING_STEP;
          
          if (servo_position < SERVO_FULLY_CLOSED) {
            servo_position = SERVO_FULLY_CLOSED;
          }
        }
        
        // Check if we've reached sufficient grip force
        if (current_mA > GRIP_CURRENT_THRESHOLD_MA && magnitude > GRIP_MAGNITUDE_THRESHOLD) {
          gripping_mode = GRIPPING_MODE_HOLDING;
          last_slip_or_entry_time = millis();
          last_backoff_time = millis();
        }
        // Allow opening while grasping
        else if (buttons.button_2) {
          gripping_mode = GRIPPING_MODE_OPENING;
        }
        break;
        
      case GRIPPING_MODE_HOLDING:
        // React to slip detection
        if (slip_flag) {
          if (millis() - last_reaction_time > REACTION_COOLDOWN_MS) {
            last_reaction_time = millis();
            last_slip_or_entry_time = millis();
            last_backoff_time = millis();
            slip_flag = false;
            gripping_mode = GRIPPING_MODE_REACTING;
          }
        }
        // Gradual backoff after no slip for BACKOFF_DELAY
        else if (millis() - last_slip_or_entry_time > BACKOFF_DELAY_MS) {
          if (millis() - last_backoff_time > BACKOFF_INTERVAL_MS) {
            last_backoff_time = millis();
            // motor_position += BACKOFF_STEP;  // Commented out as in original
            
            if (servo_position > SERVO_FULLY_OPEN) {
              servo_position = SERVO_FULLY_OPEN;
            }
          }
        }
        
        // Allow user to override and grasp tighter
        if (buttons.button_1) {
          gripping_mode = GRIPPING_MODE_GRASPING;
        }
        // Allow user to open
        else if (buttons.button_2) {
          gripping_mode = GRIPPING_MODE_OPENING;
        }
        break;
        
      case GRIPPING_MODE_REACTING:
       { // Tighten grip in response to slip
        int slip_u= round(slip_indicator / SLIP_THRESHOLD);
        if (slip_u>MAX_REACTION_STEPS) slip_u=MAX_REACTION_STEPS;
        servo_position -= slip_u;
        
        if (servo_position < SERVO_FULLY_CLOSED) {
          servo_position = SERVO_FULLY_CLOSED;
        }
        
        // Return to holding state after reaction
        gripping_mode = GRIPPING_MODE_HOLDING;
        last_slip_or_entry_time = millis();
        last_backoff_time = millis();
        break;
      }
      case GRIPPING_MODE_OPENING:
        // Move towards fully open position
        if (servo_position < SERVO_FULLY_OPEN) {
          servo_position += OPENING_STEP;
          
          if (servo_position > SERVO_FULLY_OPEN) {
            servo_position = SERVO_FULLY_OPEN;
          }
        }
        
        // Check if we've reached fully open
        if (servo_position >= SERVO_FULLY_OPEN) {
          gripping_mode = GRIPPING_MODE_OPEN;
        }
        break;
    }
  }
  
  GrippingMode getState() {
    return gripping_mode;
  }
  
  int getServoPosition() {
    return servo_position;
  }
  
  void reset() {
    gripping_mode = GRIPPING_MODE_OPEN;
    servo_position = SERVO_FULLY_OPEN;
    last_reaction_time = 0;
    last_slip_or_entry_time = 0;
    last_backoff_time = 0;
  }
  
  const char* getStateName() {
    switch (gripping_mode) {
      case GRIPPING_MODE_OPEN: return "OPEN";
      case GRIPPING_MODE_GRASPING: return "GRASPING";
      case GRIPPING_MODE_HOLDING: return "HOLDING";
      case GRIPPING_MODE_REACTING: return "REACTING";
      case GRIPPING_MODE_OPENING: return "OPENING";
      default: return "UNKNOWN";
    }
  }
}

