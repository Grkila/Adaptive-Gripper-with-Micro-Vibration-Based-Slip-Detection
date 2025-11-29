#include "DebugTask.h"
#include "FFTProcessor.h"
#include <Arduino.h>

namespace DebugTask {
  
  void init() {
    // Initialize mutexes
    mutexSlipData = xSemaphoreCreateMutex();
    if (mutexSlipData == NULL) {
      Serial.println("[DEBUG] ERROR: Failed to create mutex for slip data!");
    }
    
    mutexFFTData = xSemaphoreCreateMutex();
    if (mutexFFTData == NULL) {
      Serial.println("[DEBUG] ERROR: Failed to create mutex for FFT data!");
    }
    
    // Create debug task on Core 0
    xTaskCreatePinnedToCore(
      taskFunction,
      "DebugPrintTask",
      DEBUG_TASK_STACK_SIZE,
      NULL,
      DEBUG_TASK_PRIORITY,
      &debugTaskHandle,
      DEBUG_TASK_CORE
    );
    
    Serial.println("[DEBUG] ✓ Debug print task started on Core 0");
  }
  
  void updateData() {
    // Update shared debug data with mutex protection
    if (mutexSlipData != NULL && xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(5)) == pdTRUE) {
      debugData.slip_flag = slip_flag;
      debugData.slip_indicator = slip_indicator;
      xSemaphoreGive(mutexSlipData);
    }
  }
  
  void taskFunction(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(DEBUG_PRINT_INTERVAL_MS);
    
    for (;;) {
      // Wait for the next cycle
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      
      // Local copies of debug data
      bool local_slip_flag;
      float local_slip_indicator;
      unsigned long local_scan_time;
      bool local_scan_exceeded;
      bool local_fft_ready;
      
      // Read shared data with mutex protection
      if (mutexSlipData != NULL && xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(10)) == pdTRUE) {
        local_slip_flag = debugData.slip_flag;
        local_slip_indicator = debugData.slip_indicator;
        local_scan_time = debugData.scan_time_us;
        local_scan_exceeded = debugData.scan_time_exceeded;
        local_fft_ready = debugData.fft_ready_to_print;
        debugData.fft_ready_to_print = false;
        xSemaphoreGive(mutexSlipData);
      } else {
        continue;
      }
      
      // Print FFT data if ready (currently disabled)
      if (false) {
        if (local_fft_ready) {
          double local_fft_vReal[FFT_SAMPLES];
          double local_fft_vImag[FFT_SAMPLES];
          
          if (mutexFFTData != NULL && xSemaphoreTake(mutexFFTData, pdMS_TO_TICKS(20)) == pdTRUE) {
            memcpy(local_fft_vReal, fftX_high_pass.vReal, FFT_SAMPLES * sizeof(double));
            memcpy(local_fft_vImag, fftX_high_pass.vImag, FFT_SAMPLES * sizeof(double));
            
            fftX_high_pass.FFT_complete = false;
            fftX_high_pass.index = 0;
            
            xSemaphoreGive(mutexFFTData);
            
            FFTProcessor::printCombinedFFT(local_fft_vReal, local_fft_vReal);
          }
        }
      }
      
      // Arduino Serial Plotter format
      Serial.print(" slip_number:");
      Serial.print(local_slip_indicator);
      Serial.println();
    }
  }
}

