#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include "arduinoFFT.h"
#include "Config.h"

// ============================================
// GRIPPING STATE MACHINE ENUM
// ============================================
enum GrippingMode {
  GRIPPING_MODE_OPEN,
  GRIPPING_MODE_GRASPING,
  GRIPPING_MODE_HOLDING,
  GRIPPING_MODE_REACTING,
  GRIPPING_MODE_OPENING
};

// ============================================
// IIR FILTER STRUCTURE
// ============================================
struct IIRFilter {
  double previousOutput;
  double alpha;

  IIRFilter(double filterAlpha)
    : previousOutput(0.0), alpha(filterAlpha) {}

  // Apply filter: y[n] = α * x[n] + (1 - α) * y[n-1]
  double filter(double input) {
    double output = alpha * input + (1.0 - alpha) * previousOutput;
    previousOutput = output;
    return output;
  }

  void reset() {
    previousOutput = 0.0;
  }
};

// ============================================
// FFT AXIS STRUCTURE
// ============================================
struct AxisFFT {
  double vReal[FFT_SAMPLES];
  double vImag[FFT_SAMPLES];
  int index;
  ArduinoFFT<double> fft;
  const char* name;
  bool FFT_complete;

  AxisFFT(const char* axisName)
    : index(0),
      fft(ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, MAGNETIC_SENSOR_SAMPLING_FREQUENCY)),
      name(axisName),
      FFT_complete(false) {
    for (int i = 0; i < FFT_SAMPLES; i++) {
      vReal[i] = 0;
      vImag[i] = 0;
    }
  }
};

// ============================================
// DEBUG DATA STRUCTURE (Thread-safe)
// ============================================
struct DebugData {
  bool slip_flag;
  float slip_indicator;
  unsigned long scan_time_us;
  bool scan_time_exceeded;
  bool fft_ready_to_print;
  
  // Added metrics for debug
  double mag_x;
  double mag_y;
  double mag_z;
  double mag_magnitude;
  double mag_x_filtered; // Low pass
  double mag_y_filtered;
  double mag_z_filtered;
  
  float current_mA;
  int servo_position;
  int gripping_mode;
};

// ============================================
// MAGNETIC FIELD DATA STRUCTURE
// ============================================
struct MagneticData {
  double x;
  double y;
  double z;
  double magnitude;
  
  // Low-pass filtered values
  double x_low_pass;
  double y_low_pass;
  double z_low_pass;
  double magnitude_low_pass;
  
  // High-pass filtered values
  double x_high_pass;
  double y_high_pass;
  double z_high_pass;
  double magnitude_high_pass;
};

// ============================================
// CALIBRATION DATA STRUCTURE
// ============================================
struct CalibrationData {
  double x_min, x_max;
  double y_min, y_max;
  double z_min, z_max;
  double x_offset, y_offset, z_offset;
};

// ============================================
// BUTTON STATE STRUCTURE
// ============================================
struct ButtonState {
  bool button_1;  // Grasp
  bool button_2;  // Open
  bool button_3;  // Lifter up
  bool button_4;  // Lifter down
  bool button_5;  // automatic mode
};

#endif // TYPES_H

