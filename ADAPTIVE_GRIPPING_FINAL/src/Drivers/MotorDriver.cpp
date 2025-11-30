#include "MotorDriver.h"
#include <HardwareSerial.h>
#include <freertos/semphr.h>

// Static member initialization
volatile bool MotorDriver::enabled = false;
volatile bool MotorDriver::initialized = false;
TMC2209Stepper* MotorDriver::driver = nullptr;
FastAccelStepperEngine MotorDriver::engine = FastAccelStepperEngine();
FastAccelStepper* MotorDriver::stepper = nullptr;
SemaphoreHandle_t MotorDriver::driverMutex = NULL;

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

    // 3. Setup Pins
    // FastAccelStepper handles pins, but we ensure EN is initially high (disabled)
    pinMode(TMC_EN_PIN, OUTPUT);
    digitalWrite(TMC_EN_PIN, HIGH);

    // 4. Create and Configure TMC2209 Driver (UART)
    Serial.println("[MTR] Creating TMC2209Stepper object...");
    driver = new TMC2209Stepper(&Serial2, TMC_R_SENSE, TMC_DRIVER_ADDR);
    if (driver == nullptr) {
        Serial.println("[MTR-FATAL] Failed to allocate driver!");
        return;
    }
    
    Serial.println("[MTR] Configuring driver...");
    driver->begin();
    driver->toff(5);
    driver->mstep_reg_select(true); // 1. Tell driver to IGNORE physical MS1/MS2 pins
    driver->microsteps(TMC_MICROSTEPS);
    driver->rms_current(TMC_RUN_CURRENT);
    driver->iholddelay(10);
    driver->en_spreadCycle(true); // StealthChop
    driver->pwm_autoscale(true);
    driver->TCOOLTHRS(0xFFFFF);
    driver->SGTHRS(TMC_STALL_VALUE);
    
    // IMPORTANT: Set VACTUAL to 0 to enable STEP/DIR control
    driver->VACTUAL(0);
    Serial.print("[MTR] Driver sees Microsteps: ");
    Serial.println(driver->microsteps()); 
    Serial.println("[MTR] Driver configuration complete (STEP/DIR mode)");

    // 5. FastAccelStepper Setup
    Serial.println("[MTR] Initializing FastAccelStepper...");
    engine.init();
    stepper = engine.stepperConnectToPin(TMC_STEP_PIN);
    
    if (stepper) {
        stepper->setDirectionPin(TMC_DIR_PIN);
        stepper->setEnablePin(TMC_EN_PIN);
        stepper->setAutoEnable(true); // Automatically manage EN pin
        
        // Configuration from Config.h
        stepper->setAcceleration(TMC_ACCELERATION); 
        stepper->setSpeedInHz(TMC_MAX_SPEED);
        
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
    
    if (speed == 0) {
        stepper->stopMove();
        enabled = false;
    } else {
        // speed is in steps/s
        uint32_t speedAbs = abs(speed);
        if (speedAbs > TMC_MAX_SPEED) speedAbs = TMC_MAX_SPEED;
        if (speedAbs < 100) speedAbs = 100; // Minimum speed safety?
        
        stepper->setSpeedInHz(speedAbs);
        
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
    Serial.println("[MTR] Homing routine starting...");
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
    
    stepper->setSpeedInHz(TMC_HOMING_SPEED);
    
    if (TMC_HOMING_DIRECTION > 0) {
        stepper->runBackward(); 
    } else {
        stepper->runForward();
    }
    
    delay(5000); // Run for 5 seconds
    stepper->forceStop(); 
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

    int consecutiveStalls = 0;

    while (millis() - startTime < TMC_HOMING_TIMEOUT_MS) {
        int32_t load = getLoad(); 
        if ((millis() % 200) == 0) Serial.printf("[MTR] Load: %d\n", load);

        if (load < TMC_HOMING_THRESHOLD) { 
             consecutiveStalls++;
             if (consecutiveStalls >= TMC_HOMING_CONSECUTIVE_STALLS) {
                 Serial.printf("[MTR] Stall detected! Load: %d < %d (Consecutive: %d)\n", load, TMC_HOMING_THRESHOLD, consecutiveStalls);
                 stalled = true;
                 break;
             }
        } else {
             consecutiveStalls = 0;
        }
        
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
        stepper->setSpeedInHz(TMC_MAX_SPEED);
        stepper->setAcceleration(TMC_ACCELERATION);
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
