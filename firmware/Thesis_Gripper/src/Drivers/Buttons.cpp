#include "Buttons.h"

namespace Buttons {
  
  void init() {
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    pinMode(BUTTON_5_PIN, INPUT_PULLUP);
    Serial.println("[BUTTONS] ✓ Ready");
  }
  
  ButtonState read() {
    ButtonState state;
    state.button_1 = !digitalRead(BUTTON_1_PIN);
    state.button_2 = !digitalRead(BUTTON_2_PIN);
    state.button_3 = !digitalRead(BUTTON_3_PIN);
    state.button_4 = !digitalRead(BUTTON_4_PIN);
    state.button_5 = !digitalRead(BUTTON_5_PIN);
    return state;
  }
}

