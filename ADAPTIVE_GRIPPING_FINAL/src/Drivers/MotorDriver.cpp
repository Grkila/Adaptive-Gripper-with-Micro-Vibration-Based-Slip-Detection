#include "MotorDriver.h"
#include <HardwareSerial.h>
#include <freertos/semphr.h>

// Static member initialization
volatile float MotorDriver::targetPosition = 0.0f;
volatile float MotorDriver::currentPosition = 0.0f;
volatile int32_t MotorDriver::targetSpeed = 0;
volatile int32_t MotorDriver::currentSpeed = 0;
volatile bool MotorDriver::enabled = false;
volatile bool MotorDriver::positionMode = false;
volatile bool MotorDriver::initialized = false;  // Track init state
TMC2209Stepper* MotorDriver::driver = nullptr;
SemaphoreHandle_t MotorDriver::driverMutex = NULL;

// Constants
constexpr float STEPS_PER_VACTUAL = 0.715f; // Approx conversion 1 VACTUAL unit ~ 0.715 steps/sec
constexpr float POSITION_DEADZONE = 100.0f; // Deadzone in "VACTUAL-ticks" or steps

void MotorDriver::init() {
    Serial.println("[MTR] init() starting...");
    
    // 1. Create mutex FIRST before anything else
    Serial.println("[MTR] Creating mutex...");
    driverMutex = xSemaphoreCreateRecursiveMutex();
    if (driverMutex == NULL) {
        Serial.println("[MTR-FATAL] Failed to create mutex!");
        return;
    }
    Serial.println("[MTR] Mutex created OK");

    // 2. Initialize Serial
    Serial.println("[MTR] Initializing Serial2...");
    Serial2.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
    delay(100); // Give serial time to stabilize
    Serial.println("[MTR] Serial2 initialized");

    // 3. Setup Pins
    Serial.println("[MTR] Setting up EN pin...");
    pinMode(TMC_EN_PIN, OUTPUT);
    digitalWrite(TMC_EN_PIN, HIGH); // Disable initially

    // 4. Create driver instance AFTER Serial2 is initialized
    Serial.println("[MTR] Creating TMC2209Stepper object...");
    Serial.print("[MTR] Free heap before new: ");
    Serial.println(ESP.getFreeHeap());
    
    driver = new TMC2209Stepper(&Serial2, TMC_R_SENSE, TMC_DRIVER_ADDR);
    if (driver == nullptr) {
        Serial.println("[MTR-FATAL] Failed to allocate driver!");
        return;
    }
    Serial.println("[MTR] TMC2209Stepper object created");
    
    // 5. Driver Setup
    Serial.println("[MTR] Configuring driver...");
    driver->begin();
    Serial.println("[MTR] driver->begin() done");
    driver->toff(5);
    driver->microsteps(TMC_MICROSTEPS);
    driver->rms_current(TMC_RUN_CURRENT);
    driver->iholddelay(10);
    driver->en_spreadCycle(true);
    driver->pwm_autoscale(true);
    driver->TCOOLTHRS(0xFFFFF);
    driver->SGTHRS(TMC_STALL_VALUE);
    Serial.println("[MTR] Driver configuration complete");

    // Print free heap for debugging memory issues
    Serial.print("[MTR] Free heap after TMC init: ");
    Serial.println(ESP.getFreeHeap());

    // Mark as initialized before creating task
    initialized = true;
    Serial.println("[MTR] initialized = true");

    // 6. Create Task LAST after everything is ready
    Serial.println("[MTR] Creating motor task...");
    xTaskCreatePinnedToCore(
        motorTask,          // Function
        "MotorTask",        // Name
        4096,               // Stack size
        NULL,               // Parameters
        1,                  // Priority (Low priority)
        NULL,               // Task handle
        TMC_TASK_CORE       // Core
    );
    Serial.println("[MTR] init() complete");
}

void MotorDriver::moveTo(float absolutePosition) {
    if (!initialized) return;  // Guard against uninitialized state
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        targetPosition = absolutePosition;
        positionMode = true;
        
        // Auto-enable if needed (direct, no nested mutex)
        if (!enabled) {
            digitalWrite(TMC_EN_PIN, LOW);
            enabled = true;
        }
        xSemaphoreGiveRecursive(driverMutex);
    }
}

void MotorDriver::moveRelative(float relativePosition) {
    if (!initialized) return;
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        targetPosition = currentPosition + relativePosition;
        positionMode = true;
        
        // Auto-enable if needed (direct, no nested mutex)
        if (!enabled) {
            digitalWrite(TMC_EN_PIN, LOW);
            enabled = true;
        }
        xSemaphoreGiveRecursive(driverMutex);
    }
}

float MotorDriver::getPosition() {
    if (!initialized) return 0.0f;
    float pos = 0.0f;
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        pos = currentPosition;
        xSemaphoreGiveRecursive(driverMutex);
    }
    return pos;
}

float MotorDriver::getTargetPosition() {
    if (!initialized) return 0.0f;
    float pos = 0.0f;
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        pos = targetPosition;
        xSemaphoreGiveRecursive(driverMutex);
    }
    return pos;
}

void MotorDriver::setTargetSpeed(int32_t speed) {
    Serial.println("[MTR] setTargetSpeed enter");
    if (!initialized) {
        Serial.println("[MTR] setTargetSpeed: not initialized!");
        return;
    }
    Serial.println("[MTR] setTargetSpeed: taking mutex...");
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("[MTR] setTargetSpeed: mutex acquired");
        positionMode = false;
        targetSpeed = speed;
        
        if (speed != 0 && !enabled) {
            Serial.println("[MTR] setTargetSpeed: enabling motor");
            digitalWrite(TMC_EN_PIN, LOW);
            enabled = true;
        } else if (speed == 0 && currentSpeed == 0) {
            Serial.println("[MTR] setTargetSpeed: disabling motor");
            digitalWrite(TMC_EN_PIN, HIGH);
            enabled = false;
        }
        Serial.println("[MTR] setTargetSpeed: releasing mutex");
        xSemaphoreGiveRecursive(driverMutex);
        Serial.println("[MTR] setTargetSpeed: done");
        //Serial.flush(); // Ensure output is sent
    } else {
        Serial.println("[MTR] setTargetSpeed: mutex FAILED!");
    }
}

void MotorDriver::stop() {
    Serial.println("[MTR] stop() called");
    if (!initialized) return;
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        positionMode = false;
        targetSpeed = 0;
        xSemaphoreGiveRecursive(driverMutex);
    }
}

void MotorDriver::enable() {
    Serial.println("[MTR] enable() called");
    if (!initialized) return;
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        if (!enabled) {
            digitalWrite(TMC_EN_PIN, LOW);
            enabled = true;
        }
        xSemaphoreGiveRecursive(driverMutex);
    }
}

void MotorDriver::disable() {
    Serial.println("[MTR] disable() called");
    if (!initialized) return;
    if (xSemaphoreTakeRecursive(driverMutex, portMAX_DELAY) == pdTRUE) {
        if (enabled) {
            digitalWrite(TMC_EN_PIN, HIGH);
            enabled = false;
        }
        xSemaphoreGiveRecursive(driverMutex);
    }
}

int32_t MotorDriver::getLoad() {
    if (!initialized) return 0;
    
    int32_t result = 0;
    // Protect access from external calls
    if (xSemaphoreTakeRecursive(driverMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        result = driver->SG_RESULT();
        xSemaphoreGiveRecursive(driverMutex);
    }
    return result;
}

void MotorDriver::motorTask(void* parameter) {
    Serial.println("[MTR-TASK] Starting motor task...");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(TMC_TASK_DELAY_MS);
    
    // Safety delay to ensure system stability and init completion
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println("[MTR-TASK] Delay complete, entering loop");

    xLastWakeTime = xTaskGetTickCount();
    uint32_t loopCount = 0;

    // Loop
    while (true) {
        // Guard against uninitialized state
        if (!initialized) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Take mutex for the ENTIRE logic update to ensure consistency
        if (xSemaphoreTakeRecursive(driverMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            int32_t desiredSpeed = 0;
            
            // --- 1. Position Control Logic ---
            if (positionMode) {
                float error = targetPosition - currentPosition;
                
                // Check if arrived
                if (abs(error) < POSITION_DEADZONE) {
                    desiredSpeed = 0;
                    // If we are stopped and at target, disable (direct, no nested mutex)
                    if (currentSpeed == 0 && enabled) {
                        digitalWrite(TMC_EN_PIN, HIGH);
                        enabled = false;
                    }
                } else {
                    // Determine direction based on sign of error
                    if (error > 0) {
                        desiredSpeed = (int32_t)TMC_MAX_SPEED;
                    } else {
                        desiredSpeed = (int32_t)-TMC_MAX_SPEED;
                    }
                    
                    // Ensure enabled (direct, no nested mutex)
                    if (!enabled) {
                        digitalWrite(TMC_EN_PIN, LOW);
                        enabled = true;
                    }
                }
                targetSpeed = desiredSpeed;
            }
            
            // --- 2. Speed Ramp Logic (Shared) ---
            // Smoothly adjust currentSpeed towards targetSpeed
            if (currentSpeed < targetSpeed) {
                currentSpeed += TMC_ACCELERATION;
                if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
            } else if (currentSpeed > targetSpeed) {
                currentSpeed -= TMC_ACCELERATION;
                if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
            }
            else if(currentSpeed==0)
            {
                currentSpeed = 0;
                digitalWrite(TMC_EN_PIN, HIGH);
                enabled = false;
            }

            // --- 3. Hardware Control ---
            if (driver != nullptr) {
                // Debug: print every 100 loops when motor is active
                if (loopCount % 100 == 0 && (enabled || currentSpeed != 0)) {
                    Serial.print("[MTR-TASK] VACTUAL: ");
                    Serial.println(currentSpeed);
                }
                if (enabled) {
                    driver->VACTUAL(currentSpeed);
                } else {
                    driver->VACTUAL(0);
                    currentSpeed = 0; // Force sync
                }
            }
            
            // --- 4. Position Estimation (Integration) ---
            // Position += Speed * Time
            if (enabled && currentSpeed != 0) {
                 currentPosition += (float)currentSpeed * (TMC_TASK_DELAY_MS / 1000.0f);
            }
            
            xSemaphoreGiveRecursive(driverMutex);
        }

        loopCount++;
        // Delay for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
