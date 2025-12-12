#ifndef SLIP_DETECTION_H
#define SLIP_DETECTION_H

#include "../Config.h"
#include "../Types.h"
#include "../Globals.h"

// ============================================
// SLIP DETECTION MODULE
// ============================================

namespace SlipDetection {
  // Detect slip from FFT data
  // Updates global slip_flag and slip_indicator
  void detect();
  
  // Get current slip status
  bool isSlipDetected();
  
  // Get slip intensity
  float getSlipIndicator();
  
  // Reset slip detection state
  void reset();

  // Ignore slip detection for a number of scan cycles
  void ignoreFor(int cycles);
}

#endif // SLIP_DETECTION_H

