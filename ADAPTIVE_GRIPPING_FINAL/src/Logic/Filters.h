#ifndef FILTERS_H
#define FILTERS_H

#include "../Config.h"
#include "../Types.h"

// ============================================
// SIGNAL FILTERING MODULE
// ============================================

namespace Filters {
  // Calculate IIR filter alpha from cutoff frequency
  double calculateAlpha(double cutoffFreq, double sampleFreq);
  
  // Initialize all filter instances
  void init();
  
  // Apply main low-pass filter (500 Hz) to magnetic data
  void applyMainFilterMagneticSensor(MagneticData& data);
  
  // Apply 30 Hz low-pass filter and calculate high-pass
  void applyBandSplitFilterMagneticSensor(MagneticData& data);
  
  // Apply filter to current reading
  float filterCurrent(float raw_current_mA);
  
  // Reset all filters
  void reset();
}

#endif // FILTERS_H

