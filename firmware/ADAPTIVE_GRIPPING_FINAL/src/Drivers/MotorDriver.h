#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "../Config.h"

class MotorDriver {
public:
    static void init();
    
    // Position Control API
    static void moveTo(long absolutePosition); // Changed float to long for steps
    static void moveRelative(long relativePosition); 
    static long getPosition(); // Changed float to long
    static long getTargetPosition(); // Changed float to long
    static long mmToSteps(float mm);
    static void moveToMM(float mm);
    static void moveRelativeMM(float mm);
    // Manual/Speed Control (Overrides Position Control)
    static void setTargetSpeed(int32_t speed);
    
    // Low level
    static void stop();
    static void enable();
    static void disable();
    static int32_t getLoad();

    // Homing
    static void runHomingRoutine();

private:
    static volatile bool enabled;
    static volatile bool initialized;  // True after successful init
    
    // TMC2209 driver pointer (initialized in init())
    static TMC2209Stepper* driver;
    
    // FastAccelStepper objects
    static FastAccelStepperEngine engine;
    static FastAccelStepper* stepper;
    
    // Mutex for driver access
    static SemaphoreHandle_t driverMutex;
};

#endif // MOTOR_DRIVER_H
