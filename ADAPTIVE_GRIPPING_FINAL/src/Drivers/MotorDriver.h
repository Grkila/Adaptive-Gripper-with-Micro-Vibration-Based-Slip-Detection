#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "../Config.h"

class MotorDriver {
public:
    static void init();
    
    // Position Control API
    static void moveTo(float absolutePosition);
    static void moveRelative(float relativePosition);
    static float getPosition();
    static float getTargetPosition();
    
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
    static void motorTask(void* parameter);
    
    // Internal state
    static volatile float targetPosition;
    static volatile float currentPosition;
    static volatile int32_t targetSpeed; // For manual control or ramp
    static volatile int32_t currentSpeed;
    static volatile bool enabled;
    static volatile bool positionMode; // True if under position control
    static volatile bool initialized;  // True after successful init
    
    // TMC2209 driver pointer (initialized in init())
    static TMC2209Stepper* driver;
    
    // Mutex for driver access
    static SemaphoreHandle_t driverMutex;
};

#endif // MOTOR_DRIVER_H
