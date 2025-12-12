#ifndef GRIPPING_FSM_H
#define GRIPPING_FSM_H

#include "../Config.h"
#include "../Types.h"
#include "../Globals.h"

// ============================================
// GRIPPING FINITE STATE MACHINE MODULE
// ============================================

namespace GrippingFSM {
  // Process the gripping state machine
  void process(const ButtonState& buttons, float current_mA, float magnitude);
  
  // Get current state
  GrippingMode getState();
  
  // Get motor position
  int getServoPosition();
  
  // Reset to initial state
  void reset();
  
  // Get state name as string (for debugging)
  const char* getStateName();
}

#endif // GRIPPING_FSM_H

