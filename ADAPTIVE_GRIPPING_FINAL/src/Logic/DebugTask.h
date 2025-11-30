#ifndef DEBUG_TASK_H
#define DEBUG_TASK_H

#include "../Config.h"
#include "../Types.h"
#include "../Globals.h"

// ============================================
// DEBUG TASK MODULE (Runs on Core 0)
// ============================================

namespace DebugTask {
  struct DebugConfig {
    bool stream_mag_raw = false;
    bool stream_mag_filtered = true; 
    bool stream_current = true;
    bool stream_servo = true;
    bool stream_slip = true;
    bool stream_fft = false; // Exclusive mode
    bool stream_system = true; // Scan time, mode
  };

  extern DebugConfig config;

  // Initialize and start the debug task on Core 0
  void init();
  
  // Update shared debug data (called from main loop)
  void updateData();
  
  // Task function (runs on Core 0)
  void taskFunction(void* parameter);
}

#endif // DEBUG_TASK_H
