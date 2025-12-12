#include "Filters.h"
#include <Arduino.h>
#include <math.h>

namespace Filters {
  
  // Calculate alpha value for IIR filter
  static double alpha30Hz;
  static double alpha500Hz;
  static double alphaCurrent;
  
  // Filter instances for main 500 Hz low-pass
  static IIRFilter filterX(0);
  static IIRFilter filterY(0);
  static IIRFilter filterZ(0);
  static IIRFilter filterMagnitude(0);
  
  // Filter instances for 30 Hz low-pass (for band splitting)
  static IIRFilter filter30Hz_X(0);
  static IIRFilter filter30Hz_Y(0);
  static IIRFilter filter30Hz_Z(0);
  static IIRFilter filter30Hz_Magnitude(0);
  
  // Filter for current
  static IIRFilter filterCurrentMA(0);
  
  double calculateAlpha(double cutoffFreq, double sampleFreq) {
    return 1.0 - exp(-2.0 * PI * cutoffFreq / sampleFreq);
  }
  
  void init() {
    // Calculate alpha values
    alpha500Hz = calculateAlpha(FILTER_500HZ_CUTOFF_FREQ, MAGNETIC_SENSOR_SAMPLING_FREQUENCY);
    alpha30Hz = calculateAlpha(FILTER_30HZ_CUTOFF_FREQ, MAGNETIC_SENSOR_SAMPLING_FREQUENCY);
    alphaCurrent = calculateAlpha(FILTER_CURRENT_CUTOFF_FREQ, FILTER_CURRENT_SAMPLE_RATE);
    
    // Initialize filter coefficients
    filterX.alpha = alpha500Hz;
    filterY.alpha = alpha500Hz;
    filterZ.alpha = alpha500Hz;
    filterMagnitude.alpha = alpha500Hz;
    
    filter30Hz_X.alpha = alpha30Hz;
    filter30Hz_Y.alpha = alpha30Hz;
    filter30Hz_Z.alpha = alpha30Hz;
    filter30Hz_Magnitude.alpha = alpha30Hz;
    
    filterCurrentMA.alpha = alphaCurrent;
    
    Serial.println("[FILTERS] ✓ Initialized");
    Serial.print("  500Hz alpha: ");
    Serial.println(alpha500Hz, 6);
    Serial.print("  30Hz alpha: ");
    Serial.println(alpha30Hz, 6);
  }
  
  void applyMainFilterMagneticSensor(MagneticData& data) {
    data.x = filterX.filter(data.x);
    data.y = filterY.filter(data.y);
    data.z = filterZ.filter(data.z);
    data.magnitude = filterMagnitude.filter(data.magnitude);
  }
  
  void applyBandSplitFilterMagneticSensor(MagneticData& data) {
    // Apply 30 Hz low-pass filter
    data.x_low_pass = filter30Hz_X.filter(data.x);
    data.y_low_pass = filter30Hz_Y.filter(data.y);
    data.z_low_pass = filter30Hz_Z.filter(data.z);
    data.magnitude_low_pass = filter30Hz_Magnitude.filter(data.magnitude);
    
    // Calculate high-pass as original - low_pass
    data.x_high_pass = data.x - data.x_low_pass;
    data.y_high_pass = data.y - data.y_low_pass;
    data.z_high_pass = data.z - data.z_low_pass;
    data.magnitude_high_pass = data.magnitude - data.magnitude_low_pass;
  }
  
  float filterCurrent(float raw_current_mA) {
    return (float)filterCurrentMA.filter((double)raw_current_mA);
  }
  
  void reset() {
    filterX.reset();
    filterY.reset();
    filterZ.reset();
    filterMagnitude.reset();
    
    filter30Hz_X.reset();
    filter30Hz_Y.reset();
    filter30Hz_Z.reset();
    filter30Hz_Magnitude.reset();
    
    filterCurrentMA.reset();
  }
}

