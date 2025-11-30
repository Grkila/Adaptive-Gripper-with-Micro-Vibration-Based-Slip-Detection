#include "MotorDriver.h"
#include <HardwareSerial.h>
#include <freertos/semphr.h>

// Static member initialization
volatile bool MotorDriver::enabled = false;
volatile bool MotorDriver::initialized = false;  // Track init state
TMC2209Stepper* MotorDriver::driver = nullptr;
FastAccelStepperEngine MotorDriver::engine = FastAccelStepperEngine();
FastAccelStepper* MotorDriver::stepper = nullptr;
SemaphoreHandle_t MotorDriver::driverMutex = NULL;

// Constants
constexpr float VACTUAL_TO_HZ = 0.715f; // Approx conversion 1 VACTUAL unit ~ 0.715 steps/sec

void MotorDriver::init() {
    Serial.println("[MTR] init() starting...");
    
    // 1. Create mutex FIRST before anything else
    Serial.println("[MTR] Creating mutex...");
    driverMutex = xSemaphoreCreateRecursiveMutex();
    if (driverMutex == NULL) {
        Serial.println("[MTR-FATAL] Failed to create mutex!");
        return;
    }

    // 2. Initialize Serial
    Serial.println("[MTR] Initializing Serial2...");
    Serial2.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
    delay(100); 

    // 3. Setup Pins (EN pin handled by FastAccelStepper, but we init here too)
    pinMode(TMC_EN_PIN, OUTPUT);
    digitalWrite(TMC_EN_PIN, HIGH); // Disable initially

    // 4. Create driver instance
    Serial.println("[MTR] Creating TMC2209Stepper object...");
    driver = new TMC2209Stepper(&Serial2, TMC_R_SENSE, TMC_DRIVER_ADDR);
    if (driver == nullptr) {
        Serial.println("[MTR-FATAL] Failed to allocate driver!");
        return;
    }
    
    // 5. Driver Setup via UART
    Serial.println("[MTR] Configuring driver...");
    driver->begin();
    driver->toff(5);
    driver->microsteps(TMC_MICROSTEPS);
    driver->rms_current(TMC_RUN_CURRENT);
    driver->iholddelay(10);
    driver->en_spreadCycle(false); // StealthChop
    driver->pwm_autoscale(true);
    driver->TCOOLTHRS(0xFFFFF);
    driver->SGTHRS(TMC_STALL_VALUE);
    
    // IMPORTANT: Set VACTUAL to 0 to enable STEP/DIR control
    driver->VACTUAL(0);
    Serial.println("[MTR] Driver configuration complete (STEP/DIR mode)");

    // 6. FastAccelStepper Setup
    Serial.println("[MTR] Initializing FastAccelStepper...");
    engine.init();
    stepper = engine.stepperConnectToPin(TMC_STEP_PIN);
    
    if (stepper) {
        stepper->setDirectionPin(TMC_DIR_PIN);
        stepper->setEnablePin(TMC_EN_PIN);
        stepper->setAutoEnable(true); // Automatically manage EN pin
        
        // Convert old acceleration units to steps/s^2 if needed, or use directly
        // Assuming TMC_ACCELERATION in Config.h is suitable for FastAccelStepper directly
        stepper->setAcceleration(TMC_ACCELERATION); 
        Serial.println("[MTR] FastAccelStepper initialized");
    } else {
        Serial.println("[MTR-FATAL] Failed to create stepper!");
    }

    initialized = true;
    Serial.println("[MTR] init() complete");
}

void MotorDriver::moveTo(long absolutePosition) {
    if (stepper) stepper->moveTo(absolutePosition);
}

void MotorDriver::moveRelative(long relativePosition) {
    if (stepper) stepper->move(relativePosition);
}

long MotorDriver::getPosition() {
    if (stepper) return stepper->getCurrentPosition();
    return 0;
}

long MotorDriver::getTargetPosition() {
    if (stepper) return stepper->targetPos();
    return 0;
}

void MotorDriver::setTargetSpeed(int32_t speed) {
    if (!stepper) return;
    
    // Critical section if needed, but FAS is thread-safeish for simple calls
    // Speed control logic
    if (speed == 0) {
        stepper->stopMove(); // Ramps down to stop
        enabled = false;
    } else {
        // Use TMC_MAX_SPEED from Config.h as the cap
        float speedHz = abs(speed) * VACTUAL_TO_HZ;
        if (speedHz > TMC_MAX_SPEED) speedHz = TMC_MAX_SPEED;
        if (speedHz < 100) speedHz = 100; // Minimum speed
        
        stepper->setSpeedInHz((uint32_t)speedHz);
        
        if (speed > 0) {
            stepper->runForward();
        } else {
            stepper->runBackward();
        }
        enabled = true;
    }
}

void MotorDriver::stop() {
    if (stepper) stepper->stopMove();
}

void MotorDriver::enable() {
    if (stepper) stepper->enableOutputs();
}

void MotorDriver::disable() {
    if (stepper) stepper->disableOutputs();
}

int32_t MotorDriver::getLoad() {
    if (!initialized || !driver) return 0;
    
    int32_t result = 0;
    if (xSemaphoreTakeRecursive(driverMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        result = driver->SG_RESULT();
        xSemaphoreGiveRecursive(driverMutex);
    }
    return result;
}

void MotorDriver::runHomingRoutine() {
    Serial.println("[MTR] Homing routine starting (STEP/DIR mode)...");
    if (!initialized || !stepper) {
        Serial.println("[MTR] Not initialized, cannot home.");
        return;
    }

    // 1. Setup for homing
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        driver->rms_current(TMC_HOMING_CURRENT); 
        driver->en_spreadCycle(false);
        driver->SGTHRS(TMC_HOMING_THRESHOLD);
        xSemaphoreGiveRecursive(driverMutex);
    }

    // 1.5 Move AWAY first
    Serial.println("[MTR] Moving AWAY from home...");
    
    float homingSpeedHz = TMC_HOMING_SPEED * VACTUAL_TO_HZ;
    stepper->setSpeedInHz((uint32_t)homingSpeedHz);
    
    if (TMC_HOMING_DIRECTION > 0) {
        stepper->runBackward(); // Opposite to homing
    } else {
        stepper->runForward();
    }
    
    delay(5000); // Run for 5 seconds
    stepper->forceStop(); // Stop immediately
    delay(1000);

    // 2. Move TOWARDS home and check Stall
    Serial.println("[MTR] Moving TOWARDS home...");
    if (TMC_HOMING_DIRECTION > 0) {
        stepper->runForward();
    } else {
        stepper->runBackward();
    }

    unsigned long startTime = millis();
    bool stalled = false;
    
    delay(2000); // Skip acceleration spike

    while (millis() - startTime < TMC_HOMING_TIMEOUT_MS) {
        int32_t load = getLoad(); 
        if ((millis() % 200) == 0) Serial.printf("[MTR] Load: %d\n", load);

        if (load < TMC_HOMING_THRESHOLD) { 
             Serial.printf("[MTR] Stall detected! Load: %d < %d\n", load, TMC_HOMING_THRESHOLD);
             stalled = true;
             break;
        }
        
        // Also check if motor stopped for some other reason
        if (!stepper->isRunning()) break;
        
        delay(10);
    }

    // 3. Stop
    stepper->forceStop();
    
    if (!stalled) {
        Serial.println("[MTR] Homing timed out without stall.");
    }

    // 4. Reset Position and Config
    stepper->setCurrentPosition(0);
    
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        driver->rms_current(TMC_RUN_CURRENT);
        driver->SGTHRS(TMC_STALL_VALUE);
        xSemaphoreGiveRecursive(driverMutex);
    }
    
    Serial.println("[MTR] Homing routine complete.");
}
long MotorDriver::mmToSteps(float mm) {
    return (long)round(mm * TMC_STEPS_PER_MM);
}
void MotorDriver::moveToMM(float mm) {
     moveTo(mmToSteps(mm));
}
void MotorDriver::moveRelativeMM(float mm) {
    moveRelative(mmToSteps(mm));
}   