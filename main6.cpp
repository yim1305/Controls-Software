/*
 * ============================================================================
 * APOGEE CONTROL SYSTEM (ACS) NASA USLI CONTROLS 2026
 * ============================================================================
 * 
 * Hardware:
 * - Teensy 4.1
 * - MTi IMU (I2C address 0x6B, DRDY on pin 20)
 * - BMP390 Barometer (I2C)
 * - Ultimate GPS V3
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
#include "Logger.h"
#include "Debug.h"
#include "PhaseManager.h"
#include "Sensors.h"
#include "GPS.h"
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

unsigned long loggingStartTime = 0;

// ============================================================================
// CONTROL CONFIGURATION
// ============================================================================

// TODO: PID gains
// #define PID_KP 1.0
// #define PID_KI 0.1
// #define PID_KD 0.05
// #define TARGET_APOGEE 4600.0  // meters AG

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// State
State currentState = {0};

// Timing flags
volatile bool imuReady = false;
volatile bool baroReady = false;
volatile bool controlReady = false;

// Timers
IntervalTimer imuTimer;
IntervalTimer baroTimer;
IntervalTimer controlTimer;

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
// INITIALIZATION
// ============================================================================

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
    // Initialize SD Card and Logger
    if (!initializeLogger()) {
        while (1);  // Halt on failure
    }

    // Write TXT header
    debugFile.println("=====================================================");
    debugFile.println("                 USLI Controls 2026                  ");
    debugFile.println("=====================================================");

    loggingStartTime = micros();

    // Set the logging time reference for debug messages
    setLoggingStartTime(&loggingStartTime);

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
    logFile.print("gpsLat,gpsLon,gpsAlt,gpsSpeed,gpsSats,gpsFixOK,");

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
    if (!initializeIMU()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }
    
    // ========================================================================
    // Initialize Barometer (BMP390)
    // ========================================================================
    if (!initializeBarometer()) {
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    // ========================================================================
    // Initialize GPS (Ultimate GPS V3)
    // ========================================================================
    // Initialize GPS
    initializeGPS();

    // Perform IMU tare (bias removal)
    #ifdef ENABLE_IMU_TARE
    performIMUTare();
    #endif

    // Read and store ground altitude (barometer "tare")
    #ifdef ENABLE_BARO_TARE
    performBarometerTare();
    #endif


    KF.initialize(0.0f);
    currentState.estimatedAltitude = 0.0f;
    currentState.estimatedVelocity = 0.0f;
    lastKalmanTime = micros();
    
    //GPS 
    currentState.gpsLat = 0.0f;
    currentState.gpsLon = 0.0f;
    currentState.gpsAltitude = 0.0f;
    currentState.gpsSpeed = 0.0f;
    currentState.gpsSatellites = 0;
    currentState.gpsHasFix = false;

    
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
    // Initialize Phase Manager and auto-arm
    initializePhaseManager();
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
        readIMU(currentState);
    }
    
    // ========================================================================
    // PATH 2: BAROMETER (100 Hz)
    // Measurement updates for altitude Kalman filter
    // ========================================================================
    if (baroReady) {
        baroReady = false;
        readBarometer(currentState);
    }
    
    // ========================================================================
    // PATH 3: GPS (~10 Hz, asynchronous)
    // TODO: Implement when GPS is added
    // ========================================================================
    
    float lat, lon;
    bool fix;
    int sats;

    String sentence = getGPSString();
    if (sentence.length() > 0) {
        if (parseGGA(sentence, lat, lon, fix, sats)) {
            Serial.print("Lat: "); Serial.print(lat, 6);
            Serial.print(" Lon: "); Serial.print(lon, 6);
            Serial.print(" Fix: "); Serial.print(fix);
            Serial.print(" Sats: "); Serial.println(sats);
        }
    }
    
    // ========================================================================
    // PATH 4: CONTROL LOOP (100 Hz)
    // Main control & logging happens here
    // ========================================================================
    if (controlReady) {
        controlReady = false;
        
        // ────────────────────────────────────────────────────────────────────
        // STEP 1: Update flight phase based on current state
        // ────────────────────────────────────────────────────────────────────
        updateFlightPhase(currentState, groundAltitude, KF);
        
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
        
        addLogEntry(currentState, currentPhase, groundAltitude, loggingStartTime);
        
        
        // ────────────────────────────────────────────────────────────────────
        // STEP 5: Print debug info periodically
        // ────────────────────────────────────────────────────────────────────
        #ifdef DEBUG_SENSOR_READINGS
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {                   // set to print every 0.5 sec, but irl it can operate as fast as 31ms
            // TIME
            //DEBUG_PRINT("Time: ");
            //DEBUG_PRINT2((micros() - loggingStartTime)/1000000.0, 3);

            // IMU
            //DEBUG_PRINT("Accel: X="); DEBUG_PRINT2(currentState.accelX, 3);
            //DEBUG_PRINT(" Y="); DEBUG_PRINT2(currentState.accelY, 3);
            //DEBUG_PRINT(" accel Z = "); DEBUG_PRINT2(currentState.accelZ, 3);

            //DEBUG_PRINT("Gyro:  X="); DEBUG_PRINT2(currentState.gyroX, 3);
            //DEBUG_PRINT(" Y="); DEBUG_PRINT2(currentState.gyroY, 3);
            //DEBUG_PRINT(" Z="); DEBUG_PRINT2(currentState.gyroZ, 3);

            // Barometer
            //DEBUG_PRINT(" Alt = "); DEBUG_PRINT2(currentState.baroAltitude, 2);
            //DEBUG_PRINT(" AGL = "); DEBUG_PRINTLN2(currentState.baroAltitude - groundAltitude, 2);
            //DEBUG_PRINT(" m | Pressure="); DEBUG_PRINT2(currentState.baroPressure, 2);
            //DEBUG_PRINT(" hPa | Temp="); DEBUG_PRINT2(currentState.baroTemperature, 2);
            //DEBUG_PRINTLN(" C");

            // GPS
            //DEBUG_PRINT(" lat = "); DEBUG_PRINT2(currentState.gpsLat, 2);
            //DEBUG_PRINT(" lon = "); DEBUG_PRINT2(currentState.gpsLon, 2);
            //DEBUG_PRINT(" Sat = "); DEBUG_PRINT(currentState.gpsSatellites);
            //DEBUG_PRINT(" Fix = "); DEBUG_PRINT(currentState.gpsHasFix);

            // Flight phase
            //DEBUG_PRINT("Phase: "); DEBUG_PRINTLN(phaseNames[currentPhase]);

            lastPrint = millis();
            // TODO: Add GPS fields here if enabled
        }
        #endif

    
    // ========================================================================
    // PATH 5: SD CARD FLUSH (~1 Hz)
    // Periodic write to SD card (doesn't block control loop)
    // ========================================================================
    #ifndef DEBUG_NO_SD
    checkFlushNeeded(phaseNames);
    #endif
    
    // ========================================================================
    // PATH 6: SHUTDOWN DETECTION
    // ========================================================================
    if (currentPhase == LANDED) {
        // Force final flush
        DEBUG_PRINTLN("Flight complete. Flushing final data...");
        #ifndef DEBUG_NO_SD
        flushLogBuffer(phaseNames);
        
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
