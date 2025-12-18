// ==========================
// USLI Controls 2026
// ==========================

#include <Arduino.h>
#include <SCServo.h>
#include <eigen.h>
#include <Eigen/Dense>
#include "MTi.h"
#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <string>
#include <cmath>

using std::string;

// ---------- Pins and Constants ----------
#define BUZZER_PIN 33
#define DRDY 20
#define IMU_ADDRESS 0x6B
#define SERVOSerial Serial4
#define SERVO_ID 1
#define SERVO_OPEN_ANGLE 1650
#define SERVO_CLOSE_ANGLE 2500
const float deg2servo = 4096.0 / 360;
const float LAUNCH_THRESHOLD = 20.0f; // m/s^2
const size_t LAUNCH_CHECK_POINTS = 5;
const size_t LAUNCH_MIN_COUNT = 2;    // 2 out of the last 5 data points

// ---------- Buzzer Notes ----------
const int NOTE_A7 = 3520, NOTE_B7 = 3951, NOTE_C8 = 4186,
          NOTE_D8 = 4698, NOTE_E8 = 5274, NOTE_F8 = 5588,
          NOTE_G8 = 6272, NOTE_A8 = 7040, NOTE_B8 = 7902;

// ---------- Central Timing ----------
unsigned long lastLoopTime = 0; 
const unsigned long LOOP_INTERVAL = 1000; // ms

constexpr size_t BUFFER_SIZE = 100;
constexpr size_t FLUSH_SIZE = 10;

// ---------- Barometer ----------
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1018) // changed from 1013.25 to 1018
File barometer_output;
string barometer_output_filename = "barometer_output.txt";

struct baroData {
    float t_s; // time in seconds
    float altitude;
};

baroData baroBuffer[BUFFER_SIZE];
size_t imu_head = 0;
size_t imu_count = 0;
size_t imu_unflushed = 0;

File IMU_output;
string IMU_output_filename = "IMU_output.txt";


// ---------- IMU ----------
MTi *MyMTi = NULL;

struct IMUData {
    float t_s; // time in seconds
    float ax, ay, az;
    float wx, wy, wz;
};

IMUData imuBuffer[BUFFER_SIZE];
size_t baro_head = 0;
size_t baro_count = 0;
size_t baro_unflushed = 0;

File baro_output;
string baro_output_filename = "baro_output.txt";

// ---------- Servo and Actuation ----------
SMS_STS st;
bool Actuation = false;
float burnout_time = 3000; // burnout = 3 sec
bool BurnoutDone = false;
unsigned long elapsed_time = 0;
unsigned long Launch_Detected_time = 0;

// ---------- Tare ----------
static bool IMU_TARE_FLAG = false;
static bool Baro_TARE_FLAG = false;
static int imu_ii = 0;
static int baro_ii = 0;
static float imu_tare[6] = {0.0f};
static float baro_tare = 0.0f;
const float BARO_TARE_SKIP = 10.0;  // skip first  seconds because the sensor gives a high val that throws the avg off
const float BARO_TARE_DURATION = 5.0; // compute tare for next 5 seconds

// ---------- Stages ----------
enum RocketStage { ON_RAIL, LAUNCH, BURNOUT, APOGEE, LANDING, ABORT };
RocketStage stage = ON_RAIL;

volatile bool imuReady = false;

void imuISR() {
    imuReady = true;
}

// ==========================
// Setup
// ==========================

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(BUZZER_PIN, OUTPUT);

    // --- Buzzer start chime ---
    tone(BUZZER_PIN, NOTE_B7, 50); delay(50);
    tone(BUZZER_PIN, NOTE_D8, 50); delay(50);
    tone(BUZZER_PIN, NOTE_G8, 100); delay(2000);

    // --- SD Card ---
    if (!SD.begin(BUILTIN_SDCARD)) {
        tone(BUZZER_PIN, NOTE_E8, 200); delay(200);
        tone(BUZZER_PIN, NOTE_C8, 200); delay(200);
        tone(BUZZER_PIN, NOTE_A7, 500); delay(2000);
        while (true); // panic
    }

    IMU_output = SD.open(IMU_output_filename.c_str(), FILE_WRITE);
    if (!IMU_output) while(true); // panic
    IMU_output.println("time ax ay az wx wy wz");

    baro_output = SD.open(baro_output_filename.c_str(), FILE_WRITE);
    if (!baro_output) while(true); // panic
    baro_output.println("time altitude");

    // --- Servo ---
    SERVOSerial.begin(1000000, SERIAL_8N1);
    st.pSerial = &SERVOSerial;
    //float Pos = 0;
    //int increment = 0;
    /*while(Pos < 358*deg2servo)
    {
        st.WritePosEx(SERVO_ID, increment, 3000, 50); delay(1000);
        Pos = st.ReadPos(SERVO_ID);
        Serial.print("Servo POS reads: ");
        Serial.println(Pos);
        increment += 100;
    }*/
    st.WritePosEx(SERVO_ID, SERVO_OPEN_ANGLE, 3000, 50); delay(1000);
    Serial.println("Servo in OPEN position.");
    st.WritePosEx(SERVO_ID, SERVO_CLOSE_ANGLE, 3000, 50); delay(1000);
    Serial.println("Servo in CLOSED position.");
    // --- Barometer ---
    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("Barometer setup complete.");

    // --- IMU ---
    Wire.begin();
    pinMode(DRDY, INPUT);
    attachInterrupt(digitalPinToInterrupt(DRDY), imuISR, RISING); // added to test if it works

    MyMTi = new MTi(IMU_ADDRESS, DRDY);
    if (!MyMTi->detect(1000)) while(true);
    delay(100); MyMTi->goToConfig();
    delay(100); MyMTi->requestDeviceInfo();
    delay(100); MyMTi->configureOutputs();
    delay(100); MyMTi->goToMeasurement();
    delay(100);

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("IMU setup complete.");

    Serial.println(" --- NASA USLI CONTROLS 2026 --- ");
}

// ==========================
// Store IMU Data to Buffer and FLush
// ==========================
void StoreIMUData(float t_s, float ax, float ay, float az, float wx, float wy, float wz) {
    imuBuffer[imu_head] = {t_s, ax, ay, az, wx, wy, wz};
    imu_head = (imu_head + 1) % BUFFER_SIZE;
    if (imu_count < BUFFER_SIZE) imu_count++;
    imu_unflushed++;

    if (imu_unflushed >= FLUSH_SIZE) {
        size_t flush_start = (imu_head + BUFFER_SIZE - imu_count) % BUFFER_SIZE;
        for (size_t i = 0; i < FLUSH_SIZE; i++) {
            const IMUData& d = imuBuffer[(flush_start + i) % BUFFER_SIZE];
            IMU_output.print(d.t_s); IMU_output.print(' ');
            IMU_output.print(d.ax); IMU_output.print(' ');
            IMU_output.print(d.ay); IMU_output.print(' ');
            IMU_output.print(d.az); IMU_output.print(' ');
            IMU_output.print(d.wx); IMU_output.print(' ');
            IMU_output.print(d.wy); IMU_output.print(' ');
            IMU_output.println(d.wz);
        }
        IMU_output.flush();
        imu_unflushed = 0;
    }
}

void StoreBaroData(float t_s, float altitude) {
    baroBuffer[baro_head] = {t_s, altitude};
    baro_head = (baro_head + 1) % BUFFER_SIZE;
    if (baro_count < BUFFER_SIZE) baro_count++;
    baro_unflushed++;

    if (baro_unflushed >= FLUSH_SIZE) {
        size_t flush_start = (baro_head + BUFFER_SIZE - baro_count) % BUFFER_SIZE;
        for (size_t i = 0; i < FLUSH_SIZE; i++) {
            const baroData& d = baroBuffer[(flush_start + i) % BUFFER_SIZE];
            baro_output.print(d.t_s); baro_output.print(' ');
            baro_output.print(d.altitude); baro_output.print(' ');
        }
        IMU_output.flush();
        baro_unflushed = 0;
    }
}
// ==========================
// Read baro Data
// ==========================
void readbaro(float t_s) {
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    
    // ---------- Tare ----------
       if (!Baro_TARE_FLAG) {
        if (t_s < BARO_TARE_SKIP) {
            // skipping
            Serial.print(t_s); Serial.print(" skipping, altitude = "); Serial.println(altitude);
            return; 
        } else if (t_s < BARO_TARE_SKIP + BARO_TARE_DURATION) {
            // accumulate for tare
            baro_tare += altitude;
            baro_ii++;
            Serial.print(t_s); Serial.print(" accumulating, altitude = "); Serial.println(altitude);
            return; 
        } else if (baro_ii > 0) {
            // compute bias
            baro_tare /= baro_ii;
            Baro_TARE_FLAG = true;
            Serial.print("Barometer Tare Complete, bias = ");
            Serial.println(baro_tare);
        }
    }
 
    // ---------- Corrected Altitude ----------
    altitude = altitude - baro_tare;


    // Add to buffer
    StoreBaroData(t_s, altitude);

    // Print to Serial Monitor
    Serial.print(t_s); Serial.print(' ');
    Serial.println(altitude);
}


// ==========================
// Read IMU Data
// ==========================
void readIMU(float t_s) {
    if (digitalRead(MyMTi->drdy)) {
        MyMTi->readMessages();
        float* acc_b_N = MyMTi->getAcceleration();
        float* omega_b_rps = MyMTi->getRateOfTurn();

        // Tare
        if (IMU_TARE_FLAG) {
            acc_b_N[0] -= imu_tare[0]; 
            acc_b_N[1] -= imu_tare[1]; 
            acc_b_N[2] -= imu_tare[2];
            omega_b_rps[0] -= imu_tare[3]; 
            omega_b_rps[1] -= imu_tare[4]; 
            omega_b_rps[2] -= imu_tare[5];
        } else if (t_s < 5.0) { // First 5 seconds for tare
            imu_tare[0] += acc_b_N[0]; 
            imu_tare[1] += acc_b_N[1]; 
            imu_tare[2] += acc_b_N[2];
            imu_tare[3] += omega_b_rps[0]; 
            imu_tare[4] += omega_b_rps[1]; 
            imu_tare[5] += omega_b_rps[2];
            imu_ii++;
        } else if (!IMU_TARE_FLAG) {
            if (imu_ii > 0) for (int j = 0; j < 6; j++) imu_tare[j] /= imu_ii;
            IMU_TARE_FLAG = true;
            Serial.print("IMU Tare Complete, Bias =");
            Serial.print(imu_tare[0]); Serial.print(' ');
            Serial.print(acc_b_N[1]); Serial.print(' ');
            Serial.print(acc_b_N[2]); Serial.print(' ');
            Serial.print(acc_b_N[3]); Serial.print(' ');
            Serial.print(acc_b_N[4]); Serial.print(' ');
            Serial.println(acc_b_N[5]);
            delay(1000);
        }

        // Add to buffer
        StoreIMUData(t_s, acc_b_N[0], acc_b_N[1], acc_b_N[2],
                        omega_b_rps[0], omega_b_rps[1], omega_b_rps[2]);

    // print
    Serial.print(t_s); Serial.print(' ');
    Serial.print(acc_b_N[0]); Serial.print(' ');
    Serial.print(acc_b_N[1]); Serial.print(' ');
    Serial.print(acc_b_N[2]); Serial.print(' ');
    Serial.print(acc_b_N[3]); Serial.print(' ');
    Serial.print(acc_b_N[4]); Serial.print(' ');
    Serial.println(acc_b_N[5]);
    }
}

// ==========================
// Detect Launch
// ==========================
bool detectLaunch() {
    Eigen::VectorXf v(3);

    if (imu_count < LAUNCH_CHECK_POINTS) return false;
    size_t above = 0;
    for (size_t i = 0; i < LAUNCH_CHECK_POINTS; i++) {
        size_t index = (imu_head + BUFFER_SIZE - 1 - i) % BUFFER_SIZE;
        v << imuBuffer[index].ax, imuBuffer[index].ay, imuBuffer[index].az;
        //Serial.print("The norm: ");
        //Serial.println(v.norm());
        if (v.norm() > LAUNCH_THRESHOLD) above++;
    }
    return above >= LAUNCH_MIN_COUNT;
}

// ==========================
// Main Loop
// ==========================
unsigned long lastIMUTime = 0;
const unsigned long IMU_INTERVAL = 10; // 

unsigned long lastBaroTime = 0;
const unsigned long BARO_INTERVAL = 100; // 

void loop() {
    unsigned long now = millis();
    float t_s = now / 1000.0;

    // IMU read 
    if (now - lastIMUTime >= IMU_INTERVAL) {
        lastIMUTime = now;
        readIMU(t_s);
    }
    
    // Baro read
    if (now - lastBaroTime >= BARO_INTERVAL) {
        lastBaroTime = now;
        //readbaro(t_s);
    }

        // --- Launch Detection ---
        if (stage == ON_RAIL && detectLaunch()) {
        stage = LAUNCH;
        Launch_Detected_time = millis();
        Serial.println("Launch detected!");
        }   

        // --- Stages Handling ---
        switch(stage) {
            case ON_RAIL:
                // waiting for launch
                break;
            case LAUNCH: 
                // integrate acceleration, timers, etc.
                elapsed_time = millis() - Launch_Detected_time;
                if (!Actuation) {
                    delay(3000);
                    st.WritePosEx(SERVO_ID, SERVO_OPEN_ANGLE, 3000, 50);
                    Actuation = true;
                    Serial.println("Airbrake Actuated");
                } else if (BurnoutDone && elapsed_time >= burnout_time) {
                    
                    st.WritePosEx(SERVO_ID, SERVO_CLOSE_ANGLE, 3000, 50);
                    BurnoutDone = true;
                    Serial.println("Airbrake Retracted");
                }
//              CHECK if we have moved, if not we go back to On Rail


                break;
            case BURNOUT: 
                // handle burnout logic
                break;
            case APOGEE: 
                // handle apogee logic
                break;
            case LANDING: 
                // handle landing logic
                break;
            case ABORT: 
                // handle abort logic
                break;
        }
    }
