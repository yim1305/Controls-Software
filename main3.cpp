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

// ============================================================================
// DEBUG & CONFIGURATION SWITCHES
// ============================================================================

// Debug flags (comment out to disable)
#define DEBUG_SERIAL              // Print debug messages
#define DEBUG_PHASE_TRANSITIONS   // Print when phases change
#define DEBUG_SENSOR_READINGS     // Print sensor data periodically
// #define DEBUG_VERBOSE          // Very detailed logging (slow!)
// #define DEBUG_NO_SD            // Disable SD card (for bench testing)
// #define DEBUG_SIMULATE_FLIGHT  // Inject fake sensor data for testing

// Feature flags
#define ENABLE_IMU_TARE          // Perform IMU bias removal on startup
#define ENABLE_BARO_TARE         // Record ground altitude
#define ENABLE_PHASE_BUFFER      // Use buffer for phase detection
// #define ENABLE_GPS             // Enable GPS (not yet implemented)
// #define ENABLE_KALMAN          // Enable Kalman filters (not yet implemented)
// #define ENABLE_PID_CONTROL     // Enable PID controller (not yet implemented)

// Debug helper macros
#ifdef DEBUG_SERIAL
  #define DEBUG_PRINT(x)       Serial.print(x)
  #define DEBUG_PRINTLN(x)     Serial.println(x)
  #define DEBUG_PRINT2(x, y)   Serial.print(x, y)
  #define DEBUG_PRINTLN2(x, y) Serial.println(x, y)
  #define DEBUG_PRINTF(...)    Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT2(x, y)
  #define DEBUG_PRINTLN2(x, y)
  #define DEBUG_PRINTF(...)
#endif

#ifdef DEBUG_VERBOSE
  #define VERBOSE_PRINT(x)    Serial.print(x)
  #define VERBOSE_PRINTLN(x)  Serial.println(x)
#else
  #define VERBOSE_PRINT(x)
  #define VERBOSE_PRINTLN(x)
#endif

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

#define IMU_INTERVAL      10000      // 100 Hz
#define BARO_INTERVAL     10000     // 100 Hz
#define CONTROL_INTERVAL  10000     // 100 Hz
#define FLUSH_INTERVAL    1000000   // 1 Hz

// ============================================================================
// LOGGING CONFIGURATION
// ============================================================================

#define LOG_BUFFER_SIZE 200         // 2 seconds at 100 Hz
#define LOG_FILENAME "flight_log.csv"

// ============================================================================
// PHASE DETECTION CONFIGURATION
// ============================================================================

#define PHASE_BUFFER_SIZE 50        // 0.5 seconds at 100 Hz

// Flight detection thresholds
#define LIFTOFF_ACCEL_THRESHOLD 20.0f     // m/s² (2G)
#define LIFTOFF_ALT_THRESHOLD 10.0f       // meters AGL
#define BURNOUT_ACCEL_THRESHOLD 5.0f      // m/s²
#define BURNOUT_TIME_MIN 3000000          // 3 seconds (microseconds)
#define BURNOUT_SAMPLES 20                // Check last 20 samples for burnout
#define APOGEE_VELOCITY_THRESHOLD 2.0f    // m/s
#define APOGEE_SAMPLES 20                 // Check last 20 samples for apogee
#define LANDING_VELOCITY_THRESHOLD 1.0f   // m/s
#define LANDING_TIME_THRESHOLD 3000000    // 3 seconds (microseconds)

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
    COASTING,
    APOGEE_CONTROL,
    BALLISTIC_DESCENT,
    LANDED
};

const char* phaseNames[] = {
    "GROUND_IDLE",
    "ARMED",
    "POWERED_ASCENT",
    "COASTING",
    "APOGEE_CONTROL",
    "BALLISTIC_DESCENT",
    "LANDED"
};

// Current state (holds latest sensor readings)
struct State {
    // IMU
    float accelX, accelY, accelZ;  // m/s²
    float gyroX, gyroY, gyroZ;     // rad/s
    
    // Barometer
    float baroAltitude;            // meters
    float baroPressure;            // hPa
    float baroTemperature;         // °C
    
    // TODO: GPS
    // float gpsLat, gpsLon;
    // float gpsAltitude;
    // float gpsSpeed;
    // uint8_t gpsSatellites;
    // bool gpsHasFix;
    
    // TODO: Kalman estimates
    // float estimatedAltitude;
    // float estimatedVelocity;
};

// Complete log entry (for SD card)
struct LogEntry {
    uint32_t timestamp;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float baroAltitude, baroPressure, baroTemperature;
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
unsigned long loggingStarTime = 0;
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

// Phase detection buffer (separate from logging buffer)
#ifdef ENABLE_PHASE_BUFFER
PhaseDetectionData phaseBuffer[PHASE_BUFFER_SIZE];
int phaseBufferIndex = 0;
bool phaseBufferFilled = false;
#endif

// IMU tare values (bias removal)
float imuTare[6] = {0.0f};
bool imuTared = false;

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
    float accelMagnitude = sqrt(currentState.accelX * currentState.accelX +
                                currentState.accelY * currentState.accelY +
                                currentState.accelZ * currentState.accelZ);
    
    #ifdef ENABLE_PHASE_BUFFER
    // Add to phase detection buffer (called every control cycle = 100 Hz)
    addToPhaseBuffer(currentState.baroAltitude, accelMagnitude);
    #endif
    
    // Track max altitude
    if (currentState.baroAltitude > maxAltitude) {
        maxAltitude = currentState.baroAltitude;
    }
    
    // State machine
    switch(currentPhase) {
        case GROUND_IDLE:
            // Should never be here (auto-armed in setup)
            break;
            
        case ARMED:
            // Detect liftoff
            if (accelMagnitude > LIFTOFF_ACCEL_THRESHOLD || AGL > LIFTOFF_ALT_THRESHOLD) {
                currentPhase = POWERED_ASCENT;
                liftoffTime = now;
                phaseStartTime = now;
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> LIFTOFF DETECTED <<<");
                #endif
            }
            break;
            
        case POWERED_ASCENT:
            {
            #ifdef ENABLE_PHASE_BUFFER
            // Detect burnout: average acceleration over last 20 samples < threshold
            float avgAccel = getAverageAcceleration(BURNOUT_SAMPLES);
            
            if (avgAccel < BURNOUT_ACCEL_THRESHOLD && 
                (now - liftoffTime) > BURNOUT_TIME_MIN) {
                currentPhase = COASTING;
                phaseStartTime = now;
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> MOTOR BURNOUT - COASTING <<<");
                #endif
            }
            #else
            // Simple burnout detection without buffer
            if (accelMagnitude < BURNOUT_ACCEL_THRESHOLD && 
                (now - liftoffTime) > BURNOUT_TIME_MIN) {
                currentPhase = COASTING;
                phaseStartTime = now;
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> MOTOR BURNOUT - COASTING <<<");
                #endif
            }
            #endif
        }
            break;
            
        case COASTING:
            // TODO: Detect approaching apogee using Kalman velocity estimate
            // For now, use simple altitude check
            if (AGL > 50.0f) {  // Placeholder logic
                // Will be replaced with: if (estimatedVelocity < APOGEE_VELOCITY_THRESHOLD)
                currentPhase = APOGEE_CONTROL;
                phaseStartTime = now;
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> APPROACHING APOGEE <<<");
                #endif
            }
            break;
            
        case APOGEE_CONTROL:
            #ifdef ENABLE_PHASE_BUFFER
            // Detect past apogee: altitude decreasing for last 20 samples
            if (isAltitudeDecreasingFor(APOGEE_SAMPLES)) {
                currentPhase = BALLISTIC_DESCENT;
                phaseStartTime = now;
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> PAST APOGEE - DESCENDING <<<");
                #endif
            }
            #else
            // Simple apogee detection without buffer
            if (currentState.baroAltitude < maxAltitude - 5.0f) {
                currentPhase = BALLISTIC_DESCENT;
                phaseStartTime = now;
                #ifdef DEBUG_PHASE_TRANSITIONS
                DEBUG_PRINTLN(">>> PAST APOGEE - DESCENDING <<<");
                #endif
            }
            #endif
            break;
            
        case BALLISTIC_DESCENT:
            // Detect landing: low altitude + low acceleration for sustained time
            // TODO: Use Kalman velocity estimate for better detection
            if (AGL < 50.0f && accelMagnitude < 2.0f) {  // Placeholder
                if (lastMovementTime == 0) {
                    lastMovementTime = now;
                } else if (now - lastMovementTime > LANDING_TIME_THRESHOLD) {
                    currentPhase = LANDED;
                    phaseStartTime = now;
                    #ifdef DEBUG_PHASE_TRANSITIONS
                    DEBUG_PRINTLN(">>> LANDED <<<");
                    #endif
                }
            } else {
                lastMovementTime = 0;
            }
            break;
            
        case LANDED:
            // Stay in landed state
            break;
    }
}

bool shouldControl() {
    // Only control during coasting and near apogee
    return (currentPhase == COASTING || currentPhase == APOGEE_CONTROL);
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
// DATA LOGGING
// ============================================================================

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
    
    entry.timestamp = micros() - loggingStarTime;
    
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
// INITIALIZATION
// ============================================================================

void performIMUTare() {
    DEBUG_PRINTLN("Performing IMU tare (bias removal)...");
    DEBUG_PRINTLN("Keep sensor stationary for 3 seconds...");
    
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
    } else {
        DEBUG_PRINTLN("WARNING: No valid IMU samples collected!");
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait max 3 seconds for serial
    DEBUG_PRINTLN("\n=== ROCKET FLIGHT COMPUTER ===");
    DEBUG_PRINTLN("Initializing...");
    
    // ========================================================================
    // Initialize I2C
    // ========================================================================
    Wire.begin();
    delay(100);
    
    // ========================================================================
    // Initialize IMU (MTi)
    // ========================================================================
    DEBUG_PRINTLN("Initializing IMU...");
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
    
    // ========================================================================
    // Initialize Barometer (BMP390)
    // ========================================================================
    DEBUG_PRINTLN("Initializing Barometer...");
    
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
    
    // ========================================================================
    // TARE SENSORS (while stationary on pad)
    // This must happen BEFORE arming and starting timers
    // ========================================================================
    DEBUG_PRINTLN("\n=== SENSOR TARING ===");
    DEBUG_PRINTLN("Keep rocket stationary on launch pad...");
    delay(1000);
    
    // Perform IMU tare (bias removal)
    #ifdef ENABLE_IMU_TARE
    performIMUTare();
    #endif
    
    // Read and store ground altitude (barometer "tare")
    #ifdef ENABLE_BARO_TARE
    DEBUG_PRINTLN("Reading ground altitude...");
    float altSum = 0;
    int altSamples = 0;
    
    for (int i = 0; i < 100; i++) {
        if (baro.performReading()) {
            altSum += baro.readAltitude(1013.25);
            altSamples++;
        }
        delay(10);
    }
    
    if (altSamples > 0) {
        groundAltitude = altSum / altSamples;
        maxAltitude = groundAltitude;
        DEBUG_PRINT("Ground altitude: ");
        DEBUG_PRINT2(groundAltitude, 2);
        DEBUG_PRINTLN(" m");
    } else {
        DEBUG_PRINTLN("ERROR: Could not read ground altitude!");
        while(1);
    }
    #endif
    
    DEBUG_PRINTLN("Sensor taring complete!\n");
    
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
    // Initialize SD Card
    // ========================================================================
    #ifndef DEBUG_NO_SD
    DEBUG_PRINTLN("Initializing SD card...");
    
    if (!SD.begin(BUILTIN_SDCARD)) {
        DEBUG_PRINTLN("ERROR: SD card initialization failed!");
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    
    // Open log file
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (!logFile) {
        DEBUG_PRINTLN("ERROR: Could not open log file!");
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    
    // Write CSV header
    logFile.print("timestamp,");
    logFile.print("accelX,accelY,accelZ,");
    logFile.print("gyroX,gyroY,gyroZ,");
    logFile.print("baroAlt,baroPressure,baroTemp,");
    // TODO: Add GPS columns when implemented
    // logFile.print("gpsLat,gpsLon,gpsAlt,gpsSpeed,gpsSats,gpsFixOK,");
    // TODO: Add Kalman columns
    // logFile.print("estAlt,estVel,");
    // TODO: Add control column
    // logFile.print("control,");
    logFile.println("phase");
    logFile.flush();
    
    DEBUG_PRINTLN("SD card initialized!");
    #endif
    
    // ========================================================================
    // System Ready - Auto-arm and start
    // ========================================================================
    DEBUG_PRINTLN("\n=== SYSTEM READY ===");
    
    // Auto-arm the system
    currentPhase = ARMED;
    phaseStartTime = micros();
    loggingStarTime = micros();
    DEBUG_PRINTLN(">>> SYSTEM ARMED <<<");
    DEBUG_PRINTLN("Waiting for liftoff detection...");
    
    // Beep confirmation
    tone(BUZZER_PIN, 2000, 500);
    delay(1000);
    
    // ========================================================================
    // Start Hardware Timers
    // ========================================================================
    DEBUG_PRINTLN("Starting timers...");
    
    imuTimer.begin(imuISR, IMU_INTERVAL);
    baroTimer.begin(baroISR, BARO_INTERVAL);
    controlTimer.begin(controlISR, CONTROL_INTERVAL);
    
    DEBUG_PRINTLN("Flight computer running!");
    DEBUG_PRINTLN("========================================\n");
    
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
        float controlOutput = 0.0f;  // Default: airbrakes retracted
        
        if (shouldControl()) {
            // TODO: Implement PID control
            // controlOutput = pidController.compute(
            //     currentState.estimatedAltitude,
            //     currentState.estimatedVelocity
            // );
            // controlOutput = constrain(controlOutput, 0.0f, 100.0f);
            
            // Placeholder for now
            controlOutput = 0.0f;
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
        if (millis() - lastPrint > 1000) {
            DEBUG_PRINT("Phase: ");
            DEBUG_PRINT(phaseNames[currentPhase]);
            DEBUG_PRINT(" | Alt: ");
            DEBUG_PRINT2(currentState.baroAltitude, 1);
            DEBUG_PRINT("m | AGL: ");
            DEBUG_PRINT2(currentState.baroAltitude - groundAltitude, 1);
            DEBUG_PRINT("m | AccelZ: ");
            DEBUG_PRINT2(currentState.accelZ, 2);
            DEBUG_PRINTLN(" m/s²");
            lastPrint = millis();
        }
        #endif
    }
    
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
