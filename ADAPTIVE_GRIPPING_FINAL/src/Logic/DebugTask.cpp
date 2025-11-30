#include "DebugTask.h"
#include "FFTProcessor.h"
#include <Arduino.h>

namespace DebugTask {
  
  DebugConfig config;

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
    // This is called from the main loop (Core 1)
    if (mutexSlipData != NULL && xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(2)) == pdTRUE) {
      debugData.slip_flag = slip_flag;
      debugData.slip_indicator = slip_indicator;
      
      debugData.mag_x = magData.x;
      debugData.mag_y = magData.y;
      debugData.mag_z = magData.z;
      debugData.mag_magnitude = magData.magnitude;
      
      debugData.mag_x_filtered = magData.x_low_pass;
      debugData.mag_y_filtered = magData.y_low_pass;
      debugData.mag_z_filtered = magData.z_low_pass;
      
      debugData.current_mA = current_mA;
      debugData.servo_position = servo_position;
      debugData.gripping_mode = (int)gripping_mode;
      
      debugData.scan_time_us = measuredInterval; 
      
      xSemaphoreGive(mutexSlipData);
    }
  }

  void processSerialInput() {
    static char cmdBuffer[64];
    static int cmdIndex = 0;
    
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (cmdIndex > 0) {
          cmdBuffer[cmdIndex] = 0; // Null terminate
          String line = String(cmdBuffer);
          cmdIndex = 0;
          
          line.trim();
          if (line.length() == 0) continue;

          // Remove all spaces/tabs for easier parsing (handle {"key": value} vs {"key":value})
          line.replace(" ", "");
          line.replace("\t", "");

          bool commandFound = false;

          // Simple JSON command parsing
          // Example: {"fft":true} or {"mag":false}
          
          // Toggle FFT (Exclusive mode)
          if (line.indexOf("\"fft\":true") >= 0) {
            config.stream_fft = true; 
            Serial.println("{\"status\":\"FFT_ENABLED\"}");
            commandFound = true;
          }
          else if (line.indexOf("\"fft\":false") >= 0) {
            config.stream_fft = false;
            Serial.println("{\"status\":\"FFT_DISABLED\"}");
            commandFound = true;
          }

          if (line.indexOf("\"mag_raw\":true") >= 0) { config.stream_mag_raw = true; commandFound = true; }
          if (line.indexOf("\"mag_raw\":false") >= 0) { config.stream_mag_raw = false; commandFound = true; }

          if (line.indexOf("\"mag_filtered\":true") >= 0) { config.stream_mag_filtered = true; commandFound = true; }
          if (line.indexOf("\"mag_filtered\":false") >= 0) { config.stream_mag_filtered = false; commandFound = true; }
          
          if (line.indexOf("\"current\":true") >= 0) { config.stream_current = true; commandFound = true; }
          if (line.indexOf("\"current\":false") >= 0) { config.stream_current = false; commandFound = true; }
          
          if (line.indexOf("\"slip\":true") >= 0) { config.stream_slip = true; commandFound = true; }
          if (line.indexOf("\"slip\":false") >= 0) { config.stream_slip = false; commandFound = true; }

          if (line.indexOf("\"servo\":true") >= 0) { config.stream_servo = true; commandFound = true; }
          if (line.indexOf("\"servo\":false") >= 0) { config.stream_servo = false; commandFound = true; }
          
          if (line.indexOf("\"system\":true") >= 0) { config.stream_system = true; commandFound = true; }
          if (line.indexOf("\"system\":false") >= 0) { config.stream_system = false; commandFound = true; }

          // Ack for other commands to verify reception
          if (commandFound && !config.stream_fft) { // Don't spam in FFT mode
             // Optional: echo back current config or just OK
             // Serial.println("{\"status\":\"CMD_OK\"}");
          } else if (!commandFound) {
             // Debug help: print what we received if it didn't match
             Serial.print("{\"log\":\"Unknown cmd: ");
             Serial.print(line);
             Serial.println("\"}");
          }
        }
      } else {
        if (cmdIndex < 63) {
          cmdBuffer[cmdIndex++] = c;
        }
      }
    }
  }
  
  void taskFunction(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(DEBUG_PRINT_INTERVAL_MS);
    
    DebugData localData;
    
    for (;;) {
      // Check for commands
      processSerialInput();

      if (config.stream_fft) {
        // EXCLUSIVE FFT MODE
        
        bool local_fft_ready = false;
        if (mutexSlipData != NULL && xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(10)) == pdTRUE) {
             local_fft_ready = debugData.fft_ready_to_print;
             if (local_fft_ready) debugData.fft_ready_to_print = false;
             xSemaphoreGive(mutexSlipData);
        }

        if (local_fft_ready) {
           // We need to access the FFT arrays which are global
           
           if (mutexFFTData != NULL && xSemaphoreTake(mutexFFTData, pdMS_TO_TICKS(50)) == pdTRUE) {
              
              // Helper to print array as JSON: "fft_x":[v1,v2,...]
              
              Serial.print("{\"type\":\"fft\",\"data\":[");
              for(int i=0; i<FFT_SAMPLES/2; i++) { // Only first half is useful usually
                 Serial.print(fftX_high_pass.vReal[i], 2); 
                 if(i < (FFT_SAMPLES/2)-1) Serial.print(",");
              }
              Serial.println("]}");
              
              // Reset flag in FFT processor
              fftX_high_pass.FFT_complete = false;
              fftX_high_pass.index = 0;
              
              xSemaphoreGive(mutexFFTData);
           }
        }
        
        // Yield to allow other tasks
        vTaskDelay(pdMS_TO_TICKS(10)); 
      } 
      else {
        // NORMAL DEBUG MODE
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Copy shared data
        if (mutexSlipData != NULL && xSemaphoreTake(mutexSlipData, pdMS_TO_TICKS(10)) == pdTRUE) {
          // Use const_cast to cast away volatile for the copy, 
          // which is safe because we hold the mutex.
          localData = const_cast<DebugData&>(debugData);
          xSemaphoreGive(mutexSlipData);
        } else {
          continue;
        }

        // Build JSON String
        // Using manual string building for speed and no dynamic allocation overhead
        Serial.print("{");
        bool first = true;

        if (config.stream_mag_filtered) {
           if(!first) Serial.print(",");
           Serial.printf("\"mx\":%.2f,\"my\":%.2f,\"mz\":%.2f,\"mag\":%.2f", 
              localData.mag_x_filtered, localData.mag_y_filtered, localData.mag_z_filtered, localData.mag_magnitude);
           first = false;
        }

        if (config.stream_mag_raw) {
           if(!first) Serial.print(",");
           Serial.printf("\"rmx\":%.2f,\"rmy\":%.2f,\"rmz\":%.2f", 
              localData.mag_x, localData.mag_y, localData.mag_z);
           first = false;
        }

        if (config.stream_current) {
           if(!first) Serial.print(",");
           Serial.printf("\"cur\":%.2f", localData.current_mA);
           first = false;
        }
        
        if (config.stream_slip) {
           if(!first) Serial.print(",");
           Serial.printf("\"slip\":%d,\"s_ind\":%.2f", localData.slip_flag ? 1 : 0, localData.slip_indicator);
           first = false;
        }

        if (config.stream_servo) {
           if(!first) Serial.print(",");
           Serial.printf("\"srv\":%d,\"grp\":%d", localData.servo_position, localData.gripping_mode);
           first = false;
        }

        if (config.stream_system) {
           if(!first) Serial.print(",");
           Serial.printf("\"t\":%lu", localData.scan_time_us);
           first = false;
        }

        Serial.println("}");
      }
    }
  }
}
