#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include "../Config.h"
#include "../Types.h"

// ============================================
// BUTTON INPUT DRIVER
// ============================================

namespace Buttons {
  // Initialize button pins
  void init();
  
  // Read button states
  ButtonState read();
}

#endif // BUTTONS_H

