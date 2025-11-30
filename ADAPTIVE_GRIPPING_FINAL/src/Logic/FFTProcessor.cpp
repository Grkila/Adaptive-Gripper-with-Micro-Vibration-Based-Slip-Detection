#include "FFTProcessor.h"
#include <Arduino.h>

namespace FFTProcessor {
  
  static int counter = 0;
  
  bool processSingleAxis(AxisFFT& axisData, double value) {
    // Protect FFT data access with mutex
    if (mutexFFTData != NULL && xSemaphoreTake(mutexFFTData, 0) == pdTRUE) {
      
      // Don't add new samples if FFT is complete and waiting to be printed
      if (!axisData.FFT_complete) {
        if (axisData.index < FFT_SAMPLES) {
          axisData.vReal[axisData.index] = value;
          axisData.vImag[axisData.index] = 0;
          axisData.index++;
        } else {
          // Buffer full - compute FFT
          axisData.fft.compute(FFTDirection::Forward);
          axisData.fft.complexToMagnitude();
          axisData.index = 0;
          axisData.FFT_complete = true;
        }
      }
      
      xSemaphoreGive(mutexFFTData);
    }
    
    return axisData.FFT_complete;
  }
  
  void process(const MagneticData& data) {
    // Process FFT for high-pass filtered components
    bool x_complete = processSingleAxis(fftX_high_pass, data.x_high_pass);
    processSingleAxis(fftY_high_pass, data.y_high_pass);
    
    // Signal debug task when FFT is ready
    if (true) {
      if (x_complete) {
        counter++;
        if (counter >= 1) {
          if (xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(1)) == pdTRUE) {
            debugData.fft_ready_to_print = true;
            xSemaphoreGive(mutexSlipData);
          }
          counter = 0;
        }
      }
    }
  }
  
  void printCombinedFFT(double* lowPass, double* highPass) {
    uint16_t dataSize = FFT_SAMPLES >> 1;
    
    Serial.println("{");
    Serial.print("  \"fs\": ");
    Serial.print(MAGNETIC_SENSOR_SAMPLING_FREQUENCY, 2);
    Serial.println(",");
    Serial.print("  \"samples\": ");
    Serial.print(dataSize);
    Serial.println(",");
    
    // Frequency array
    Serial.print("  \"freq\": [");
    for (uint16_t i = 0; i < dataSize; i++) {
      Serial.print((i * MAGNETIC_SENSOR_SAMPLING_FREQUENCY) / FFT_SAMPLES, 2);
      if (i < dataSize - 1) Serial.print(",");
    }
    Serial.println("],");
    
    // Low-pass data
    Serial.print("  \"low_pass\": [");
    for (uint16_t i = 0; i < dataSize; i++) {
      Serial.print(lowPass[i], 4);
      if (i < dataSize - 1) Serial.print(",");
    }
    Serial.println("],");
    
    // High-pass data
    Serial.print("  \"high_pass\": [");
    for (uint16_t i = 0; i < dataSize; i++) {
      Serial.print(highPass[i], 4);
      if (i < dataSize - 1) Serial.print(",");
    }
    Serial.println("]");
    
    Serial.println("}");
  }
  
  bool isReady(const AxisFFT& axisData) {
    return axisData.FFT_complete;
  }
  
  void resetAxis(AxisFFT& axisData) {
    axisData.FFT_complete = false;
    axisData.index = 0;
  }
}

