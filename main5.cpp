/*
 * ============================================================================
 * ROCKET FLIGHT COMPUTER - APOGEE CONTROL SYSTEM
 * ============================================================================
 * 
 * Hardware:
 * - Teensy 4.1
 * - MTi IMU (I2C address 0x6B, DRDY on pin 20)
 * - BMP390 Barometer (I2C)
 * - GPS (TODO)
 * - Servo/Airbrake actuator (TODO)
 * - SD Card (built-in)
 * 
 * Architecture:
 * - Multi-rate sensor reading with hardware timers
 * - Two separate buffers: logging (SD) and phase detection (analysis)
 * - State machine for flight phase management
 */

#include <Wire.h>
#include <SD.h>
#include <Adafruit_BMP3XX.h>
#include "MTi.h"
#include "State.h"
#include "Debug.h"
#include "KalmanFilter.h"
#include <ArduinoEigenDense.h>
using namespace Eigen;


// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Pin definitions
#define IMU_DRDY_PIN 20
#define BUZZER_PIN 33
// #define GPS_SERIAL Serial8     // TODO: GPS connected to Serial8
// #define SERVO_PIN 9            // TODO: Airbrake servo pin

// I2C addresses
#define IMU_ADDRESS 0x6B

// TODO: GPS Configuration
// #define GPS_BAUD_INITIAL 9600
// #define GPS_BAUD_OPERATING 115200
// #define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
// #define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
// #define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F\r\n"

// ============================================================================
// TIMING CONFIGURATION (microseconds)
// ============================================================================

#define IMU_INTERVAL      5000      // 200 Hz
#define BARO_INTERVAL     10000     // 100 Hz
#define CONTROL_INTERVAL  10000     // 100 Hz
#define FLUSH_INTERVAL    1000000   // 1 Hz

// ============================================================================
// LOGGING CONFIGURATION
// ============================================================================

#define LOG_BUFFER_SIZE 200         // 2 seconds at 100 Hz
#define LOG_FILENAME "flight_log.csv"
#define DEBUG_FILENAME "debug_log.txt"

// ============================================================================
// PHASE DETECTION CONFIGURATION
// ============================================================================

#define PHASE_BUFFER_SIZE 50        // 0.5 seconds at 100 Hz

// Liftoff detection (ARMED → POWERED_ASCENT)
#define LIFTOFF_ACCEL_THRESHOLD 2.0f      // m/s² - adjust from motor specs  (Recheck!!!!!!!!!!!!!!!) (set to 2 for elavator test)
#define LIFTOFF_ALT_THRESHOLD 1.0f        // meters AGL  (set to 1m for elevator)

// Burnout detection (POWERED_ASCENT → COASTING)
#define BURNOUT_ACCEL_THRESHOLD 1.5f      // m/s² - below motor thrust
#define BURNOUT_TIME_MIN 1500000            // 3 seconds (microseconds)   (Set to 1.5 for testing first)
#define BURNOUT_SAMPLES 10                  // Average over N samples

// Apogee detection (COASTING → DESCENT)
#define APOGEE_VELOCITY_THRESHOLD 2.0f     // m/s (velocity near zero)
#define APOGEE_ALT_MIN 1.5f                // meters AGL (safety check)

// Landing detection (DESCENT → LANDED)
#define LANDING_ALT_THRESHOLD 20.0f        // meters AGL
#define LANDING_VELOCITY_THRESHOLD 1.0f    // m/s
#define LANDING_ACCEL_THRESHOLD 2.0f       // m/s² (near 1G when stationary)
#define LANDING_TIME_THRESHOLD 2000000     // 3 seconds (microseconds)

// ============================================================================
// CONTROL CONFIGURATION
// ============================================================================

// TODO: PID gains
// #define PID_KP 1.0
// #define PID_KI 0.1
// #define PID_KD 0.05
// #define TARGET_APOGEE 4600.0  // meters AGL

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Flight phases
enum FlightPhase {
    GROUND_IDLE,
    ARMED,
    POWERED_ASCENT,
    COASTING,        // Motor burned out, airbrakes active
    DESCENT,         // Past apogee, airbrakes retracted
    LANDED
};

const char* phaseNames[] = {
    "GROUND_IDLE",
    "ARMED",
    "POWERED_ASCENT",
    "COASTING",
    "DESCENT",
    "LANDED"
};

// Complete log entry (for SD card)
struct LogEntry {
    uint32_t timestamp;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float baroAltitude, baroPressure, baroTemperature;
    float altitudeAGL;
    float estimatedAltitude, estimatedVelocity;
    // TODO: Add GPS fields
    // float gpsLat, gpsLon, gpsAltitude, gpsSpeed;
    // uint8_t gpsSatellites;
    // uint8_t gpsHasFix;
    uint8_t phase;
} __attribute__((packed));

// Phase detection data (for analysis only)
struct PhaseDetectionData {
    float altitude;
    float accelMagnitude;
    unsigned long timestamp;
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Sensors
MTi *imu = NULL;
Adafruit_BMP3XX baro;

// TODO: GPS variables
// #ifdef ENABLE_GPS
// String nmeaBuffer = "";
// unsigned long lastGPSRead = 0;
// #endif

// State
State currentState = {0};

// Flight phase
FlightPhase currentPhase = GROUND_IDLE;
unsigned long phaseStartTime = 0;
float loggingStartTime = 0.000f;
unsigned long liftoffTime = 0;
float groundAltitude = 0.0f;
float maxAltitude = 0.0f;
unsigned long lastMovementTime = 0;

// Timing flags
volatile bool imuReady = false;
volatile bool baroReady = false;
volatile bool controlReady = false;

// Timers
IntervalTimer imuTimer;
IntervalTimer baroTimer;
IntervalTimer controlTimer;

// Logging buffer
LogEntry logBuffer[LOG_BUFFER_SIZE];
volatile int logWriteIndex = 0;
int logFlushIndex = 0;
File logFile;
unsigned long lastFlush = 0;

// debug log
File debugFile;

// Phase detection buffer (separate from logging buffer)
#ifdef ENABLE_PHASE_BUFFER
PhaseDetectionData phaseBuffer[PHASE_BUFFER_SIZE];
int phaseBufferIndex = 0;
bool phaseBufferFilled = false;
#endif

// IMU tare values (bias removal)
float imuTare[6] = {0.0f};
bool imuTared = false;

// Kalman Filter
KalmanFilter KF;
unsigned long lastKalmanTime = 0;

// ============================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// ============================================================================

void imuISR() {
    imuReady = true;  // Just set flag, don't do work in ISR
}

void baroISR() {
    baroReady = true;
}

void controlISR() {
    controlReady = true;
}

// ============================================================================
// DATA LOGGING
// ============================================================================

void logDebugMessage(const char* message, float value, int decimals = 4) {
    #ifndef DEBUG_NO_SD
    if (debugFile) {
        unsigned long t = micros() - loggingStartTime;
        float time_s = (float)t / 1000000.0f;
        debugFile.print("[");
        debugFile.print(time_s, 6);
        debugFile.print(" s] ");
        debugFile.print(message);
        debugFile.println(value, decimals); // print the float value with specified decimals
        debugFile.flush();  // optional
    }
    #endif
}

void logDebugMessage(const char* message) {
    #ifndef DEBUG_NO_SD
    if (debugFile) {
        unsigned long t = micros() - loggingStartTime;
        float time_s = (float)t / 1000000.0f;
        debugFile.print("[");
        debugFile.print(time_s, 6);
        debugFile.print(" s] ");
        debugFile.println(message);
        debugFile.flush();  // optional
    }
    #endif
}

void flushLogBuffer() {
    if (!logFile) return;
    
    // Write all pending entries
    while (logFlushIndex != logWriteIndex) {
        LogEntry& entry = logBuffer[logFlushIndex];
        
        // Write as CSV
        logFile.print(entry.timestamp);
        logFile.print(',');
        logFile.print(entry.accelX, 6);
        logFile.print(',');
        logFile.print(entry.accelY, 6);
        logFile.print(',');
        logFile.print(entry.accelZ, 6);
        logFile.print(',');
        logFile.print(entry.gyroX, 6);
        logFile.print(',');
        logFile.print(entry.gyroY, 6);
        logFile.print(',');
        logFile.print(entry.gyroZ, 6);
        logFile.print(',');
        logFile.print(entry.baroAltitude, 3);
        logFile.print(',');
        logFile.print(entry.baroPressure, 2);
        logFile.print(',');
        logFile.print(entry.baroTemperature, 2);
        logFile.print(',');
        logFile.print(entry.altitudeAGL, 3);
        logFile.print(',');
        logFile.print(entry.estimatedAltitude, 3);
        logFile.print(',');
        logFile.print(entry.estimatedVelocity, 3);
        logFile.print(',');
        
        // TODO: Add GPS columns when implemented
        // logFile.print(entry.gpsLat, 8);
        // logFile.print(',');
        // logFile.print(entry.gpsLon, 8);
        // logFile.print(',');
        // logFile.print(entry.gpsAltitude, 2);
        // logFile.print(',');
        // logFile.print(entry.gpsSpeed, 2);
        // logFile.print(',');
        // logFile.print(entry.gpsSatellites);
        // logFile.print(',');
        // logFile.print(entry.gpsHasFix);
        // logFile.print(',');
        
        // TODO: Add Kalman estimate columns
        // logFile.print(entry.estimatedAltitude, 3);
        // logFile.print(',');
        // logFile.print(entry.estimatedVelocity, 3);
        // logFile.print(',');
        
        // TODO: Add control output column
        // logFile.print(entry.controlOutput, 2);
        // logFile.print(',');
        
        logFile.print(phaseNames[entry.phase]);
        logFile.println();
        
        logFlushIndex = (logFlushIndex + 1) % LOG_BUFFER_SIZE;
    }
    
    // Commit to SD card (this is the slow part)
    logFile.flush();
    lastFlush = micros();
    
    VERBOSE_PRINTLN("Log flushed to SD");
}

void checkFlushNeeded() {
    unsigned long now = micros();
    
    // Calculate buffer usage
    int pending = (logWriteIndex - logFlushIndex + LOG_BUFFER_SIZE) % LOG_BUFFER_SIZE;
    bool bufferNearFull = pending > (LOG_BUFFER_SIZE * 0.75);
    bool timeToFlush = (now - lastFlush) >= FLUSH_INTERVAL;
    
    if ((timeToFlush || bufferNearFull) && pending > 0) {
        flushLogBuffer();
    }
}

void addLogEntry() {
    LogEntry& entry = logBuffer[logWriteIndex];
    
    entry.timestamp = micros() - loggingStartTime;
    
    // IMU data (latest values)
    entry.accelX = currentState.accelX;
    entry.accelY = currentState.accelY;
    entry.accelZ = currentState.accelZ;
    entry.gyroX = currentState.gyroX;
    entry.gyroY = currentState.gyroY;
    entry.gyroZ = currentState.gyroZ;
    
    // Barometer data (latest values)
    entry.baroAltitude = currentState.baroAltitude;
    entry.baroPressure = currentState.baroPressure;
    entry.baroTemperature = currentState.baroTemperature;

    entry.altitudeAGL = currentState.baroAltitude - groundAltitude;

    // Kalman data
    entry.estimatedAltitude = currentState.estimatedAltitude;
    entry.estimatedVelocity = currentState.estimatedVelocity;
    
    // TODO: GPS data (when implemented)
    // entry.gpsLat = currentState.gpsLat;
    // entry.gpsLon = currentState.gpsLon;
    // entry.gpsAltitude = currentState.gpsAltitude;
    // entry.gpsSpeed = currentState.gpsSpeed;
    // entry.gpsSatellites = currentState.gpsSatellites;
    // entry.gpsHasFix = currentState.gpsHasFix ? 1 : 0;
    
    // TODO: Kalman estimates
    // entry.estimatedAltitude = currentState.estimatedAltitude;
    // entry.estimatedVelocity = currentState.estimatedVelocity;
    
    // TODO: Control output
    // entry.controlOutput = pidController.getOutput();
    
    // Flight phase
    entry.phase = currentPhase;
    
    // Advance write index (circular buffer)
    logWriteIndex = (logWriteIndex + 1) % LOG_BUFFER_SIZE;
    
    // Safety check: buffer overflow
    if (logWriteIndex == logFlushIndex) {
        DEBUG_PRINTLN("WARNING: Log buffer overflow! Forcing flush...");
        flushLogBuffer();
    }
}

// ============================================================================
// PHASE DETECTION BUFFER FUNCTIONS
// ============================================================================

#ifdef ENABLE_PHASE_BUFFER

void addToPhaseBuffer(float altitude, float accelMag) {
    // Add current data to circular buffer for phase detection
    phaseBuffer[phaseBufferIndex].altitude = altitude;
    phaseBuffer[phaseBufferIndex].accelMagnitude = accelMag;
    phaseBuffer[phaseBufferIndex].timestamp = micros();
    
    phaseBufferIndex = (phaseBufferIndex + 1) % PHASE_BUFFER_SIZE;
    
    if (phaseBufferIndex == 0) {
        phaseBufferFilled = true;  // Buffer has wrapped around
    }
}

// Helper function: Get data from N samples ago
PhaseDetectionData getPhaseDataFromPast(int samplesAgo) {
    if (samplesAgo < 0 || samplesAgo >= PHASE_BUFFER_SIZE) {
        // Return current data if out of range
        return phaseBuffer[(phaseBufferIndex - 1 + PHASE_BUFFER_SIZE) % PHASE_BUFFER_SIZE];
    }
    
    int index = (phaseBufferIndex - 1 - samplesAgo + PHASE_BUFFER_SIZE) % PHASE_BUFFER_SIZE;
    return phaseBuffer[index];
}

// Helper function: Check if altitude has been decreasing for N samples
bool isAltitudeDecreasingFor(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        return false;  // Not enough data yet
    }
    
    for (int i = 1; i < numSamples; i++) {
        PhaseDetectionData current = getPhaseDataFromPast(i - 1);
        PhaseDetectionData previous = getPhaseDataFromPast(i);
        
        if (current.altitude >= previous.altitude) {
            return false;  // Altitude increased, not consistently decreasing
        }
    }
    
    return true;  // Altitude decreased for all N samples
}

// Helper function: Calculate average acceleration over N samples
float getAverageAcceleration(int numSamples) {
    if (!phaseBufferFilled && phaseBufferIndex < numSamples) {
        numSamples = phaseBufferIndex;  // Use whatever we have
    }
    
    if (numSamples == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < numSamples; i++) {
        sum += getPhaseDataFromPast(i).accelMagnitude;
    }
    
    return sum / numSamples;
}

#endif  // ENABLE_PHASE_BUFFER

// ============================================================================
// FLIGHT PHASE MANAGER
// ============================================================================

void updateFlightPhase() {
    unsigned long now = micros();
    
    // Calculate derived values
    float AGL = currentState.baroAltitude - groundAltitude;

    // Kalman update
    uint32_t now_us = micros();
    KF.update(AGL, now_us);
    currentState.estimatedAltitude = KF.getAltitude();
    currentState.estimatedVelocity = KF.getVelocity();

    float accelMagnitude = sqrt(currentState.accelX * currentState.accelX +
                                currentState.accelY * currentState.accelY +
                                currentState.accelZ * currentState.accelZ);
    
    #ifdef ENABLE_PHASE_BUFFER
    // Add to phase detection buffer (called every control cycle = 100 Hz)
    addToPhaseBuffer(currentState.estimatedAltitude, accelMagnitude);
    #endif
    
    // Track max altitude
    if (currentState.estimatedAltitude > maxAltitude) {
        maxAltitude = currentState.estimatedAltitude;
    }
    
    // State machine
    switch(currentPhase) {
        case GROUND_IDLE:
            // Should never be here (auto-armed in setup)
            break;
            
        case ARMED: {
            // ================================================================
            // LIFTOFF DETECTION: ARMED → POWERED_ASCENT
            // Require BOTH high acceleration AND altitude gain
            // ================================================================
            bool highAcceleration = (accelMagnitude > LIFTOFF_ACCEL_THRESHOLD);
            bool altitudeGain = (currentState.estimatedAltitude > LIFTOFF_ALT_THRESHOLD);
            
            if (highAcceleration && altitudeGain) {
                currentPhase = POWERED_ASCENT;
                liftoffTime = now;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> LIFTOFF DETECTED <<<");
                DEBUG_PRINT("Accel: "); DEBUG_PRINT2(accelMagnitude, 2);
                DEBUG_PRINT(" m/s² | Alt: "); DEBUG_PRINT2(currentState.estimatedAltitude, 2);
                DEBUG_PRINTLN(" m");
                logDebugMessage(">>> LIFTOFF DETECTED <<<");
                #endif
            }
        }
            break;
            
        case POWERED_ASCENT: {
            // ================================================================
            // BURNOUT DETECTION: POWERED_ASCENT → COASTING
            // Motor burns out, transition to airbrake control
            // ================================================================
            bool burnoutConditionMet = false;
            
            #ifdef ENABLE_PHASE_BUFFER
            // Check average acceleration over last N samples
            float avgAccel = getAverageAcceleration(BURNOUT_SAMPLES);
            
            bool lowAccel = (avgAccel < BURNOUT_ACCEL_THRESHOLD);
            bool minBurnTime = ((now - liftoffTime) > BURNOUT_TIME_MIN);
            bool stillAscending = (currentState.estimatedVelocity > 5.0f);
            
            //burnoutConditionMet = lowAccel && minBurnTime && stillAscending;  (commented out fopr testing)
            burnoutConditionMet = minBurnTime;
            
            #else
            // Simple burnout detection
            bool lowAccel = (accelMagnitude < BURNOUT_ACCEL_THRESHOLD);
            bool minBurnTime = ((now - liftoffTime) > BURNOUT_TIME_MIN);
            bool stillAscending = (currentState.estimatedVelocity > 5.0f);
            
            burnoutConditionMet = lowAccel && minBurnTime && stillAscending;
            #endif
            
            if (burnoutConditionMet) {
                currentPhase = COASTING;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> MOTOR BURNOUT - COASTING <<<");
                DEBUG_PRINT("Velocity: "); DEBUG_PRINT2(currentState.estimatedVelocity, 2);
                DEBUG_PRINT(" m/s | Alt: "); DEBUG_PRINT2(currentState.estimatedAltitude, 2);
                DEBUG_PRINTLN(" m");
                logDebugMessage(">>> MOTOR BURNOUT - COASTING <<<");
                #endif
            }
        }
            break;
            
        case COASTING: {
            // ================================================================
            // APOGEE DETECTION: COASTING → DESCENT
            // Detect when velocity becomes negative (past apogee)
            // Retract airbrakes before descent
            // ================================================================
            
            // Apogee detection criteria:
            // 1. Velocity is negative (descending)
            // 2. Reasonable altitude (not on ground)
            bool descendingVelocity = (currentState.estimatedVelocity < -APOGEE_VELOCITY_THRESHOLD);     // (REVISE FOR BETTER LOGIC)
            bool reasonableAltitude = (currentState.estimatedAltitude > APOGEE_ALT_MIN);
            
            if (descendingVelocity && reasonableAltitude) {
                currentPhase = DESCENT;
                phaseStartTime = now;
                
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> APOGEE DETECTED - DESCENDING <<<");
                DEBUG_PRINT("Max Alt: "); DEBUG_PRINT2(maxAltitude, 2);
                DEBUG_PRINT(" m | Velocity: "); DEBUG_PRINT2(currentState.estimatedVelocity, 2);
                DEBUG_PRINTLN(" m/s");
                logDebugMessage(">>> APOGEE DETECTED - DESCENDING <<<");
                #endif
            }
        }
            break;
            
        case DESCENT: {
            // ================================================================
            // LANDING DETECTION: DESCENT → LANDED
            // Detect when rocket has safely landed
            // ================================================================
            
            bool lowAltitude = (currentState.estimatedAltitude < LANDING_ALT_THRESHOLD);
            bool lowVelocity = (abs(currentState.estimatedVelocity) < LANDING_VELOCITY_THRESHOLD);
            bool nearStationary = (abs(accelMagnitude - 9.81f) < LANDING_ACCEL_THRESHOLD);
            
            if (lowAltitude && lowVelocity && nearStationary) {
                if (lastMovementTime == 0) {
                    lastMovementTime = now;
                } else if ((now - lastMovementTime) > LANDING_TIME_THRESHOLD) {
                    currentPhase = LANDED;
                    phaseStartTime = now;
                    
                    #ifdef DEBUG_PHASE_TRANSITIONS
                    DEBUG_PRINTLN(">>> LANDED <<<");
                    DEBUG_PRINT("Final Alt: "); DEBUG_PRINT2(currentState.estimatedAltitude, 2);
                    DEBUG_PRINT(" m | Max Alt: "); DEBUG_PRINT2(maxAltitude, 2);
                    DEBUG_PRINTLN(" m");
                    logDebugMessage(">>> LANDED <<<");
                    #endif
                }
            } else {
                lastMovementTime = 0;
            }
        }
            break;
            
        case LANDED:
            // Stay in landed state
            break;
    }
}

bool shouldControl() {
    // Only control airbrakes during COASTING phase
    // Airbrakes deploy during coast to target apogee
    // Retract automatically when entering DESCENT
    return (currentPhase == COASTING);
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

void readIMU() {
    if (imu && digitalRead(IMU_DRDY_PIN)) {
        imu->readMessages();
        
        float* acc = imu->getAcceleration();
        float* gyro = imu->getRateOfTurn();
        
        // Apply tare (bias removal)
        if (imuTared) {
            currentState.accelX = acc[0] - imuTare[0];
            currentState.accelY = acc[1] - imuTare[1];
            currentState.accelZ = acc[2] - imuTare[2];
            currentState.gyroX = gyro[0] - imuTare[3];
            currentState.gyroY = gyro[1] - imuTare[4];
            currentState.gyroZ = gyro[2] - imuTare[5];
        } else {
            currentState.accelX = acc[0];
            currentState.accelY = acc[1];
            currentState.accelZ = acc[2];
            currentState.gyroX = gyro[0];
            currentState.gyroY = gyro[1];
            currentState.gyroZ = gyro[2];
        }
        
        VERBOSE_PRINT("IMU: ax=");
        VERBOSE_PRINT(currentState.accelX);
        VERBOSE_PRINT(" ay=");
        VERBOSE_PRINT(currentState.accelY);
        VERBOSE_PRINT(" az=");
        VERBOSE_PRINTLN(currentState.accelZ);
        
        // TODO: Feed IMU data to Kalman filters
        // altitudeKalman.predict(currentState.accelZ);
        // positionKalman.predict(currentState.accelX, currentState.accelY);
    }
}

void readBarometer() {
    if (!baro.performReading()) {
        DEBUG_PRINTLN("Failed to read barometer!");
        return;
    }
    
    currentState.baroAltitude = baro.readAltitude(1013.25);  // Sea level pressure
    currentState.baroPressure = baro.pressure / 100.0f;      // Convert Pa to hPa
    currentState.baroTemperature = baro.temperature;
    
    VERBOSE_PRINT("Baro: ");
    VERBOSE_PRINT(currentState.baroAltitude);
    VERBOSE_PRINTLN(" m");
    
    // TODO: Feed barometer data to Kalman filter
    // altitudeKalman.update(currentState.baroAltitude);
}

// ============================================================================
// GPS FUNCTIONS
// ============================================================================

// TODO: Implement GPS integration
// #ifdef ENABLE_GPS
// 
// bool initializeGPS() {
//     DEBUG_PRINTLN("Initializing GPS...");
//     GPS_SERIAL.begin(GPS_BAUD_INITIAL);
//     delay(200);
//     GPS_SERIAL.print(PMTK_SET_BAUD_115200);
//     delay(200);
//     GPS_SERIAL.end();
//     GPS_SERIAL.begin(GPS_BAUD_OPERATING);
//     delay(200);
//     GPS_SERIAL.print(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//     delay(100);
//     GPS_SERIAL.print(PMTK_SET_NMEA_UPDATE_10HZ);
//     delay(100);
//     return true;
// }
//
// void parseNMEA(String sentence) {
//     // Parse GPGGA and GPRMC sentences
//     // Extract lat, lon, altitude, speed, satellites
// }
//
// void readGPS() {
//     // Read NMEA sentences from Serial
//     // Parse and update currentState.gpsXXX
//     // Feed to position Kalman filter
// }
//
// #endif  // ENABLE_GPS

// ============================================================================
// INITIALIZATION
// ============================================================================

void performIMUTare() {
    DEBUG_PRINTLN("Performing IMU tare (bias removal)...");
    DEBUG_PRINTLN("Keep sensor stationary for 3 seconds...");
    logDebugMessage("Performing IMU tare...");
    
    int samples = 0;
    unsigned long tareStart = millis();
    float sum[6] = {0};
    
    while (millis() - tareStart < 3000) {
        if (digitalRead(IMU_DRDY_PIN)) {  // Check DRDY first!
            imu->readMessages();
            float* acc = imu->getAcceleration();
            float* gyro = imu->getRateOfTurn();
            
            // Only add if valid (not NaN or extreme values)
            if (abs(acc[0]) < 50 && abs(acc[1]) < 50 && abs(acc[2]) < 50) {
                sum[0] += acc[0];
                sum[1] += acc[1];
                sum[2] += acc[2];
                sum[3] += gyro[0];
                sum[4] += gyro[1];
                sum[5] += gyro[2];
                samples++;
            }
        }
        delay(10);  // Wait for next data ready
    }
    
    if (samples > 0) {
        for (int i = 0; i < 6; i++) {
            imuTare[i] = sum[i] / samples;
        }
        imuTared = true;
        DEBUG_PRINTLN("IMU tare complete!");
        DEBUG_PRINT("Bias: ax="); DEBUG_PRINT2(imuTare[0], 4);
        DEBUG_PRINT(" ay="); DEBUG_PRINT2(imuTare[1], 4);
        DEBUG_PRINT(" az="); DEBUG_PRINTLN2(imuTare[2], 4);
        DEBUG_PRINT("Samples collected: "); DEBUG_PRINTLN(samples);

        // log debug message
        logDebugMessage("IMU tare complete!");
        logDebugMessage("Bias: ax=", imuTare[0], 4);
        logDebugMessage("      ay=", imuTare[1], 4);
        logDebugMessage("      az=", imuTare[2], 4);
        logDebugMessage("Samples Collected:  ", samples);

    } else {
        DEBUG_PRINTLN("WARNING: No valid IMU samples collected!");
        logDebugMessage("WARNING: No valid IMU samples collected!");
    }
}

// ============================================================================
// SET UP
// ============================================================================

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait max 3 seconds for serial
    DEBUG_PRINTLN("\n=== ROCKET FLIGHT COMPUTER ===");
    DEBUG_PRINTLN("Initializing...");
    
    // ========================================================================
    // Initialize SD Card
    // ========================================================================
    #ifndef DEBUG_NO_SD
    DEBUG_PRINTLN("Initializing SD card...");

    if (!SD.begin(BUILTIN_SDCARD)) {
        DEBUG_PRINTLN("ERROR: SD card initialization failed!");
        while (1);
    }


    if (SD.exists(DEBUG_FILENAME)) {
        SD.remove(DEBUG_FILENAME);
    }

    // Initialize Debug file.txt
    debugFile = SD.open(DEBUG_FILENAME, FILE_WRITE);
    if(!debugFile) {
        DEBUG_PRINTLN("ERROR: Could not open debug file!");
        while(1);
    }

    // Write TXT header
    debugFile.println("=====================================================");
    debugFile.println("                 USLI Controls 2026                  ");
    debugFile.println("=====================================================");

    // Initialize Logfile.csv
    if (SD.exists(LOG_FILENAME)) {
        SD.remove(LOG_FILENAME);
    }

    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (!logFile) {
        DEBUG_PRINTLN("ERROR: Could not open log file!");
        while (1);
    }

    // Write CSV header
    logFile.print("timestamp,");
    logFile.print("accelX,accelY,accelZ,");
    logFile.print("gyroX,gyroY,gyroZ,");
    logFile.print("baroAlt,baroPressure,baroTemp,");
    logFile.print("AGL,KFalt,KFvel,");
    // TODO: Add GPS columns when implemented
    // logFile.print("gpsLat,gpsLon,gpsAlt,gpsSpeed,gpsSats,gpsFixOK,");
    // TODO: Add Kalman columns
    // logFile.print("estAlt,estVel,");
    // TODO: Add control column
    // logFile.print("control,");
    logFile.println("phase");
    logFile.flush();
    
    DEBUG_PRINTLN("SD card initialized!");
    logDebugMessage("SD card initialized!");
    #endif

    // ========================================================================
    // Initialize I2C
    // ========================================================================
    Wire.begin();
    delay(100);
    
    // ========================================================================
    // Initialize IMU (MTi)
    // ========================================================================
    DEBUG_PRINTLN("Initializing IMU...");
    logDebugMessage("Initializing IMU...");

    pinMode(IMU_DRDY_PIN, INPUT);
    imu = new MTi(IMU_ADDRESS, IMU_DRDY_PIN);
    
    if (!imu->detect(1000)) {
        DEBUG_PRINTLN("ERROR: IMU not detected! Check connections.");
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    
    imu->goToConfig();
    delay(100);
    imu->requestDeviceInfo();
    delay(100);
    imu->configureOutputs();
    delay(100);
    imu->goToMeasurement();
    delay(100);

    // Give IMU time to start producing data
    delay(500);  // Add this!
    
    DEBUG_PRINTLN("IMU initialized!");
    logDebugMessage("IMU initialized!");
    
    // ========================================================================
    // Initialize Barometer (BMP390)
    // ========================================================================
    DEBUG_PRINTLN("Initializing Barometer...");
    logDebugMessage("Initializing Barometer...");
    
    if (!baro.begin_I2C()) {
        DEBUG_PRINTLN("ERROR: BMP390 not detected! Check connections.");
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    
    // Configure BMP390 for high performance
    baro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    baro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    baro.setOutputDataRate(BMP3_ODR_100_HZ);
    
    DEBUG_PRINTLN("Barometer initialized!");
    logDebugMessage("Barometer initialized!");
    
    // ========================================================================
    // TARE SENSORS (while stationary on pad)
    // This must happen BEFORE arming and starting timers
    // ========================================================================
    DEBUG_PRINTLN("\n=== SENSOR TARING ===");
    DEBUG_PRINTLN("Keep rocket stationary on launch pad...");
    logDebugMessage("Sensor Taring Begins...");
    delay(1000);
    
    // Perform IMU tare (bias removal)
    #ifdef ENABLE_IMU_TARE
    performIMUTare();
    #endif
    
    // Read and store ground altitude (barometer "tare")
    #ifdef ENABLE_BARO_TARE
    DEBUG_PRINTLN("Reading ground altitude...");
    logDebugMessage("Reading ground altitude...");

    float altSum = 0.0f;
    int altSamples = 0;

    const int DISCARD_READINGS = 10;
    const int TOTAL_READINGS   = 110;  // 10 discard + 100 used

    int validCount = 0;

    for (int i = 0; i < TOTAL_READINGS; i++) {
        if (baro.performReading()) {
            float alt = baro.readAltitude(1013.25);

            // Skip first few valid readings
            if (validCount >= DISCARD_READINGS) {
                altSum += alt;
                altSamples++;
            }

            validCount++;
        }
        delay(10);
    }  

    if (altSamples > 0) {
        groundAltitude = altSum / altSamples;
        maxAltitude = groundAltitude;

        DEBUG_PRINT("Ground altitude: ");
        DEBUG_PRINT2(groundAltitude, 2);
        DEBUG_PRINTLN(" m");

        logDebugMessage("Barometer tare complete!");
        logDebugMessage("Ground altitude: ", groundAltitude, 2);
    } else {
        DEBUG_PRINTLN("ERROR: Could not read ground altitude!");
        logDebugMessage("ERROR: Could not read ground altitude!");
        while (1);
    }
    #endif

    KF.initialize(0.0f);
    currentState.estimatedAltitude = 0.0f;
    currentState.estimatedVelocity = 0.0f;
    lastKalmanTime = micros();
    
    // ========================================================================
    // TODO: Initialize GPS
    // ========================================================================
    // #ifdef ENABLE_GPS
    // if (!initializeGPS()) {
    //     DEBUG_PRINTLN("WARNING: GPS initialization failed!");
    // }
    // currentState.gpsLat = 0.0f;
    // currentState.gpsLon = 0.0f;
    // currentState.gpsAltitude = 0.0f;
    // currentState.gpsSpeed = 0.0f;
    // currentState.gpsSatellites = 0;
    // currentState.gpsHasFix = false;
    // #endif
    
    // ========================================================================
    // TODO: Initialize Kalman Filters
    // ========================================================================
    // DEBUG_PRINTLN("Initializing Kalman filters...");
    // altitudeKalman.initialize(groundAltitude);
    // positionKalman.initialize(0, 0);
    // DEBUG_PRINTLN("Kalman filters initialized!");
    
    // ========================================================================
    // TODO: Initialize PID Controller
    // ========================================================================
    // DEBUG_PRINTLN("Initializing PID controller...");
    // pidController.setGains(PID_KP, PID_KI, PID_KD);
    // pidController.setTargetApogee(TARGET_APOGEE);
    // DEBUG_PRINTLN("PID controller initialized!");
    
    // ========================================================================
    // TODO: Initialize Airbrake/Servo
    // ========================================================================
    // DEBUG_PRINTLN("Initializing airbrake...");
    // airbrake.attach(SERVO_PIN);
    // airbrake.write(0);  // Retracted position
    // DEBUG_PRINTLN("Airbrake initialized (retracted)!");
    
    // ========================================================================
    // System Ready - Auto-arm and start
    // ========================================================================
    DEBUG_PRINTLN("\n=== SYSTEM READY ===");
    logDebugMessage("\n==================== SYSTEM READY ====================");
    
    // Auto-arm the system
    currentPhase = ARMED;
    phaseStartTime = micros();
    loggingStartTime = micros();
    DEBUG_PRINTLN(">>> SYSTEM ARMED <<<");
    DEBUG_PRINTLN("Waiting for liftoff detection...");
    logDebugMessage(">>> SYSTEM ARMED <<< Waiting for liftoff detection...");
    
    // Beep confirmation
    tone(BUZZER_PIN, 2000, 500);
    delay(1000);
    
    // ========================================================================
    // Start Hardware Timers
    // ========================================================================
    DEBUG_PRINTLN("Starting timers...");
    logDebugMessage("Starting timers...");
    
    imuTimer.begin(imuISR, IMU_INTERVAL);
    baroTimer.begin(baroISR, BARO_INTERVAL);
    controlTimer.begin(controlISR, CONTROL_INTERVAL);
    
    DEBUG_PRINTLN("Flight computer running!");
    DEBUG_PRINTLN("========================================\n");
    logDebugMessage("Flight computer running!");
    
    digitalWrite(LED_BUILTIN, LOW);
}

// ============================================================================
// MAIN LOOP - Multi-rate sensor fusion & control
// ============================================================================

void loop() {
    // ========================================================================
    // PATH 1: IMU (500 Hz)
    // Fast prediction updates for Kalman filters
    // ========================================================================
    if (imuReady) {
        imuReady = false;
        readIMU();
    }
    
    // ========================================================================
    // PATH 2: BAROMETER (100 Hz)
    // Measurement updates for altitude Kalman filter
    // ========================================================================
    if (baroReady) {
        baroReady = false;
        readBarometer();
    }
    
    // ========================================================================
    // PATH 3: GPS (~10 Hz, asynchronous)
    // TODO: Implement when GPS is added
    // ========================================================================
    // #ifdef ENABLE_GPS
    // readGPS();
    // #endif
    
    // ========================================================================
    // PATH 4: CONTROL LOOP (100 Hz)
    // Main control & logging happens here
    // ========================================================================
    if (controlReady) {
        controlReady = false;
        
        // ────────────────────────────────────────────────────────────────────
        // STEP 1: Update flight phase based on current state
        // ────────────────────────────────────────────────────────────────────
        updateFlightPhase();
        
        // ────────────────────────────────────────────────────────────────────
        // STEP 2: Compute control output (if in active control phase)
        // ────────────────────────────────────────────────────────────────────
        //float controlOutput = 0.0f;  // Default: airbrakes retracted
        
        //if (shouldControl()) {
            // TODO: Implement PID control
            // controlOutput = pidController.compute(
            //     currentState.estimatedAltitude,
            //     currentState.estimatedVelocity
            // );
            // controlOutput = constrain(controlOutput, 0.0f, 100.0f);
            
            // Placeholder for now
            //controlOutput = 0.0f;
        }
        
        // ────────────────────────────────────────────────────────────────────
        // STEP 3: Command actuators
        // ────────────────────────────────────────────────────────────────────
        // TODO: Implement actuator control
        // airbrake.write(map(controlOutput, 0, 100, 0, 180));
        
        // ────────────────────────────────────────────────────────────────────
        // STEP 4: Log complete snapshot of current state
        // This is the ONLY place where logging happens (100 Hz)
        // ────────────────────────────────────────────────────────────────────
        #ifndef DEBUG_NO_SD
        addLogEntry();
        #endif
        
        // ────────────────────────────────────────────────────────────────────
        // STEP 5: Print debug info periodically
        // ────────────────────────────────────────────────────────────────────
        #ifdef DEBUG_SENSOR_READINGS
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {                   // set to print every 0.5 sec, but irl it can operate as fast as 31ms
            // TIME
            DEBUG_PRINT("Time: ");
            DEBUG_PRINT2((micros() - loggingStartTime)/1000000, 2);

            // IMU
            //DEBUG_PRINT("Accel: X="); DEBUG_PRINT2(currentState.accelX, 3);
            //DEBUG_PRINT(" Y="); DEBUG_PRINT2(currentState.accelY, 3);
            DEBUG_PRINT(" Z="); DEBUG_PRINT2(currentState.accelZ, 3);

            //DEBUG_PRINT("Gyro:  X="); DEBUG_PRINT2(currentState.gyroX, 3);
            //DEBUG_PRINT(" Y="); DEBUG_PRINT2(currentState.gyroY, 3);
            DEBUG_PRINT(" Z="); DEBUG_PRINT2(currentState.gyroZ, 3);

            // Barometer
            DEBUG_PRINT("Baro: Alt="); DEBUG_PRINT2(currentState.baroAltitude, 2);
            DEBUG_PRINT(" m | AGL="); DEBUG_PRINTLN2(currentState.baroAltitude - groundAltitude, 2);
            //DEBUG_PRINT(" m | Pressure="); DEBUG_PRINT2(currentState.baroPressure, 2);
            //DEBUG_PRINT(" hPa | Temp="); DEBUG_PRINT2(currentState.baroTemperature, 2);
            //DEBUG_PRINTLN(" C");

            // Flight phase
            DEBUG_PRINT("Phase: "); DEBUG_PRINTLN(phaseNames[currentPhase]);

            lastPrint = millis();
            // TODO: Add GPS fields here if enabled
        }
        #endif

    
    // ========================================================================
    // PATH 5: SD CARD FLUSH (~1 Hz)
    // Periodic write to SD card (doesn't block control loop)
    // ========================================================================
    #ifndef DEBUG_NO_SD
    checkFlushNeeded();
    #endif
    
    // ========================================================================
    // PATH 6: SHUTDOWN DETECTION
    // ========================================================================
    if (currentPhase == LANDED) {
        // Force final flush
        DEBUG_PRINTLN("Flight complete. Flushing final data...");
        #ifndef DEBUG_NO_SD
        flushLogBuffer();
        
        // Close log file
        logFile.close();
        #endif
        
        // Beep to indicate complete
        tone(BUZZER_PIN, 1000, 1000);
        
        DEBUG_PRINTLN("Flight computer shutdown. Remove power to reset.");
        
        // Stay in infinite loop (or enter low-power mode)
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(1000);
        }
    }
}

/*
 * ============================================================================
 * TODO: ADDING COMPONENTS LATER
 * ============================================================================
 * 
 * 1. GPS INTEGRATION:
 *    - Uncomment GPS configuration at top (#define ENABLE_GPS)
 *    - Uncomment GPS functions section
 *    - Uncomment GPS reading in main loop
 *    - Uncomment GPS fields in State struct and LogEntry
 *    - Uncomment GPS columns in CSV header and flush function
 *    - GPS code template is already in place (commented out)
 * 
 * 2. KALMAN FILTERS:
 *    - Create AltitudeKalman class
 *      - State: [altitude, velocity]
 *      - Inputs: barometer altitude, IMU accel Z
 *    - Create PositionKalman class
 *      - State: [lat, lon, vel_N, vel_E]
 *      - Inputs: GPS lat/lon, IMU accel X/Y
 *    - Add Kalman state variables to State struct
 *    - Call kalman.predict() in IMU path (500 Hz)
 *    - Call kalman.update() in Baro/GPS paths
 *    - Add Kalman estimates to LogEntry
 *    - Update CSV logging and phase detection to use Kalman outputs
 * 
 * 3. PID CONTROLLER:
 *    - Create PIDController class
 *    - Implement compute() method
 *    - Add anti-windup
 *    - Add derivative filtering
 *    - Tune gains (Kp, Ki, Kd)
 *    - Set target apogee
 *    - Call in control loop when shouldControl() returns true
 * 
 * 4. SERVO/AIRBRAKE:
 *    - Add servo library
 *    - Attach servo to pin
 *    - Map control output (0-100%) to servo angle
 *    - Add position limits
 *    - Test retract/deploy on ground
 * 
 * 5. TESTING & VALIDATION:
 *    - Bench test all sensors
 *    - Validate phase transitions with recorded data
 *    - Test Kalman filters with simulated flight
 *    - Tune PID gains in simulation
 *    - Low-altitude test flights
 *    - Full-scale test flight
 */
