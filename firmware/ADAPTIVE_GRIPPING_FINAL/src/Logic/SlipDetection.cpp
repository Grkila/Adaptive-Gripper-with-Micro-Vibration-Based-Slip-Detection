#include "SlipDetection.h"
#include <Arduino.h>

namespace SlipDetection {
  
  static int ignore_counter = 0;

  void detect() {
    // Handle ignore counter (vibration filtering)
    

    // Check if FFT data is ready
    if (fftY_high_pass.FFT_complete) {
      
      // Calculate frequency bins
      const uint16_t start_bin = round((float)SLIP_FREQ_START_HZ * FFT_SAMPLES / MAGNETIC_SENSOR_SAMPLING_FREQUENCY);
      const uint16_t end_bin = round((float)SLIP_FREQ_END_HZ * FFT_SAMPLES / MAGNETIC_SENSOR_SAMPLING_FREQUENCY);
      
      float max_power = 0;
      int peak_freq = 0;
      
      // Find peak in frequency range
      for (int i = start_bin; i < end_bin; i++) {
        float power = (fftY_high_pass.vReal[i] * fftY_high_pass.vReal[i] + 
                       fftY_high_pass.vImag[i] * fftY_high_pass.vImag[i]) / FFT_SAMPLES;
        if (power > max_power) {
          max_power = power;
          peak_freq = i * (MAGNETIC_SENSOR_SAMPLING_FREQUENCY / FFT_SAMPLES);
        }
      }
      
      // Calculate slip indicator
      slip_indicator = max_power * peak_freq;
      
      // Determine slip flag
      if (slip_indicator > SLIP_THRESHOLD) {
        slip_flag = true;
      } else {
        slip_flag = false;
      }
      
      // Signal that new data is ready
      new_slip_data_ready = true;

      // Update shared debug data
      if (mutexSlipData != NULL && xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(5)) == pdTRUE) {
        debugData.slip_flag = slip_flag;
        debugData.slip_indicator = slip_indicator;
        xSemaphoreGive(mutexSlipData);
      }
      
      // Reset FFT for next cycle
      fftY_high_pass.FFT_complete = false;
    }
  }
  
  bool isSlipDetected() {
    return slip_flag;
  }
  
  float getSlipIndicator() {
    return slip_indicator;
  }
  
  void reset() {
    slip_flag = false;
   // slip_indicator = 0.0f;
    new_slip_data_ready = false;
  }

  void ignoreFor(int cycles) {
    ignore_counter = cycles;
    reset();
  }
}
