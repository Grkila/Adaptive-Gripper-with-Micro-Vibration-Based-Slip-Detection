#ifndef FFT_PROCESSOR_H
#define FFT_PROCESSOR_H

#include "../Config.h"
#include "../Types.h"
#include "../Globals.h"

// ============================================
// FFT PROCESSING MODULE
// ============================================

namespace FFTProcessor {
  // Process single axis FFT (DRY principle)
  // Returns true if FFT computation completed
  bool processSingleAxis(AxisFFT& axisData, double value);
  
  // Process all axes FFT
  void process(const MagneticData& data);
  
  // Print combined FFT data (JSON format)
  void printCombinedFFT(double* lowPass, double* highPass);
  
  // Check if FFT is ready for processing
  bool isReady(const AxisFFT& axisData);
  
  // Reset FFT state after processing
  void resetAxis(AxisFFT& axisData);
}

#endif // FFT_PROCESSOR_H

