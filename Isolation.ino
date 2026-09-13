#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <LoRa.h>

// Include your generated Machine Learning model
#include "isolation_forest.h" 

using namespace BLA;

// ================================================================
// PIN DEFINITIONS & MODULE INSTANCES
// ================================================================
#define SDA_PIN 21
#define SCL_PIN 22

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 115200

#define VIBRATION_PIN 25

// LoRa SX1276 Pin Configuration
#define LORA_NSS  5
#define LORA_RST  27
#define LORA_DIO0 26
const long LORA_FREQUENCY = 433E6; // 433 MHz

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // ESP32 UART2

// ================================================================
// MPU6050 STATE VARIABLES
// ================================================================
uint32_t LoopTimer;
const float dt = 0.004; // 250 Hz Loop (4ms)

float RateRoll, RatePitch, RateYaw;
float RateCalRoll = 0, RateCalPitch = 0, RateCalYaw = 0;
float AccX, AccY, AccZ;
float AccCalX = 0, AccCalY = 0, AccCalZ = 0;

float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0;
float RateRollDegS, RatePitchDegS, RateYawDegS;

float AccXEarth = 0, AccYEarth = 0, AccZEarth = 0;

float PosX = 0, PosY = 0, PosZ = 0;
float PrevPosX = 0, PrevPosY = 0, PrevPosZ = 0;

float KalmanAngleRoll = 0, UncertaintyRoll = 4.0;
float KalmanAnglePitch = 0, UncertaintyPitch = 4.0;

// Variables to hold previous states for ML Delta calculation
float prevAccelMag = 0;
float prevGyroMag = 0;
float prevAngleRoll = 0;
float prevAnglePitch = 0;

struct Kalman3D {
  BLA::Matrix<2,1> State;
  BLA::Matrix<2,2> P;
  BLA::Matrix<2,2> F;
  BLA::Matrix<2,1> G;
  BLA::Matrix<2,2> Q;
  BLA::Matrix<1,1> R;
  BLA::Matrix<1,2> H;
  BLA::Matrix<2,2> I;
};

Kalman3D kfX, kfY, kfZ;

// Transmission timing variables
uint32_t lastTxTime = 0;
int lastGpsSecond = -1;

// Function Prototypes
void initIMU();
void readIMU();
void calibrateIMU();
void update1DKalman(float &state, float &uncertainty, float gyroRate, float accelAngle);
void init3DKalman(Kalman3D &kf);
void update3DKalman(Kalman3D &kf, float accelInput, float &outPos, float &outVel);
void processGPS();
void sendLoRaPacket(String packetData);

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  pinMode(VIBRATION_PIN, INPUT_PULLDOWN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  initIMU();
  calibrateIMU();

  init3DKalman(kfX);
  init3DKalman(kfY);
  init3DKalman(kfZ);

  // FIX: Increase RX buffer to prevent GPS character drops during blocking LoRa TX
  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize LoRa SX1276
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println(F("[ERROR] LoRa initialization failed! Check wiring."));
  } else {
    Serial.println(F("[INFO] LoRa SX1276 Initialized Successfully."));
  }

  LoopTimer = micros();
}

// ================================================================
// MAIN LOOP
// ================================================================
void loop() {
  // 1. Ingest GPS Characters
  processGPS();

  // 2. Vibration Sensor Check
  String eventStatus = "";
  if (digitalRead(VIBRATION_PIN) == HIGH) {
    eventStatus = "[VIB ALERT] ";
  }

  // 3. IMU Reading & Processing
  readIMU();

  RateRollDegS  = RateRoll  - RateCalRoll;
  RatePitchDegS = RatePitch - RateCalPitch;
  RateYawDegS   = RateYaw   - RateCalYaw;

  if (abs(RateYawDegS) < 1.5) RateYawDegS = 0;

  float AccAngleRoll  =  atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / 3.14159265);
  float AccAnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / 3.14159265);

  update1DKalman(KalmanAngleRoll, UncertaintyRoll, RateRollDegS, AccAngleRoll);
  update1DKalman(KalmanAnglePitch, UncertaintyPitch, RatePitchDegS, AccAnglePitch);
  AngleYaw += RateYawDegS * dt;

  AngleRoll  = KalmanAngleRoll;
  AnglePitch = KalmanAnglePitch;

  float rRoll  = AngleRoll  * (3.14159265 / 180.0);
  float rPitch = AnglePitch * (3.14159265 / 180.0);

  float ax = (AccX - AccCalX) * 9.81;
  float ay = (AccY - AccCalY) * 9.81;
  float az = (AccZ - AccCalZ) * 9.81;

  AccXEarth = cos(rPitch) * ax + sin(rRoll) * sin(rPitch) * ay + cos(rRoll) * sin(rPitch) * az;
  AccYEarth = cos(rRoll) * ay - sin(rRoll) * az;
  AccZEarth = -sin(rPitch) * ax + sin(rRoll) * cos(rPitch) * ay + cos(rRoll) * cos(rPitch) * az - 9.81;

  float accelMag = sqrt(AccXEarth * AccXEarth + AccYEarth * AccYEarth + AccZEarth * AccZEarth);
  float gyroMag  = sqrt(RateRollDegS * RateRollDegS + RatePitchDegS * RatePitchDegS + RateYawDegS * RateYawDegS);

  bool isStationary = (accelMag < 0.60) && (gyroMag < 2.0);

  if (isStationary) {
    AccXEarth = 0; AccYEarth = 0; AccZEarth = 0;
    kfX.State(1,0) = 0; kfY.State(1,0) = 0; kfZ.State(1,0) = 0;
  }

  float VelX, VelY, VelZ;
  update3DKalman(kfX, AccXEarth, PosX, VelX);
  update3DKalman(kfY, AccYEarth, PosY, VelY);
  update3DKalman(kfZ, AccZEarth, PosZ, VelZ);

  if (isStationary) {
    PosX = PrevPosX; PosY = PrevPosY; PosZ = PrevPosZ;
    kfX.State(0,0) = PosX; kfY.State(0,0) = PosY; kfZ.State(0,0) = PosZ;
  }

  PrevPosX = PosX; PrevPosY = PosY; PrevPosZ = PosZ;

  // --- 4. ANOMALY DETECTION (ISOLATION FOREST) ---
  
  // Calculate Deltas (Current Value - Previous Value)
  float deltaAccelMag = accelMag - prevAccelMag;
  float deltaGyroMag = gyroMag - prevGyroMag;
  float deltaAngleRoll = AngleRoll - prevAngleRoll;
  float deltaAnglePitch = AnglePitch - prevAnglePitch;
  
  // Update previous values for the next loop iteration
  prevAccelMag = accelMag;
  prevGyroMag = gyroMag;
  prevAngleRoll = AngleRoll;
  prevAnglePitch = AnglePitch;
  
  // Feed the delta values to the model
  float features[4] = { deltaAccelMag, deltaGyroMag, deltaAngleRoll, deltaAnglePitch }; 
  
  // Call the function from your header file
  float anomalyScore = mine_anomaly_model_decision_function(features);
  
  if (anomalyScore < 0.0) {
    eventStatus += "[ANOMALY DETECTED] ";
  }

  // --- 5. CONSTRUCT DATA PACKET STRING ---
  String payload = eventStatus;

  // FIX: Separate Time and Date checks to prevent N/A when time is available but date is not
  if (gps.time.isValid()) {
    char timeStr[40];
    if (gps.date.isValid()) {
      snprintf(timeStr, sizeof(timeStr), "UTC[%04d-%02d-%02dT%02d:%02d:%02dZ] ",
               gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
      snprintf(timeStr, sizeof(timeStr), "UTC[NO_DATE %02d:%02d:%02dZ] ",
               gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    payload += String(timeStr);
  } else {
    payload += "UTC[SEARCHING...] ";
  }

  // Append Machine Learning Score
  payload += "AI[Score:" + String(anomalyScore, 3) + "] | ";

  // Append IMU Data (Distance/Pos data removed)
  payload += "IMU[R:" + String(AngleRoll, 1) + 
             " P:" + String(AnglePitch, 1) + 
             " Y:" + String(AngleYaw, 1) + "] | ";

  // Append GPS Data
  if (gps.location.isValid()) {
    payload += "GPS[Lat:" + String(gps.location.lat(), 6) + 
               " Lng:" + String(gps.location.lng(), 6) + 
               " Alt:" + String(gps.altitude.meters(), 1) + 
               "m Spd:" + String(gps.speed.kmph(), 1) + 
               "km/h Sat:" + String(gps.satellites.value()) + "]";
  } else {
    payload += "GPS[Sats:" + String(gps.satellites.value()) + " - Searching...]";
  }


  // --- 6. LORA TRANSMISSION TRIGGER ---
  bool shouldTransmit = false;

  if (gps.time.isValid()) {
    // Sync trigger using GPS UTC Seconds rollover
    int currentGpsSec = gps.time.second();
    if (currentGpsSec != lastGpsSecond) {
      lastGpsSecond = currentGpsSec;
      shouldTransmit = true;
    }
  } else {
    // Fallback trigger using millis() every 1 second if GPS fix is lost
    if (millis() - lastTxTime >= 1000) {
      shouldTransmit = true;
    }
  }

  if (shouldTransmit) {
    Serial.println(payload); // Moved here so it only prints 1x per second!
    sendLoRaPacket(payload);
    lastTxTime = millis();
  }

  // 250Hz Loop Pace (4ms loop execution time)
  // Ensure we don't get stuck if LoRa packet took longer than 4ms
  long timeToWait = 4000 - (micros() - LoopTimer);
  if (timeToWait > 0) {
    delayMicroseconds(timeToWait);
  }
  LoopTimer = micros();
}

// ================================================================
// LORA TRANSMISSION HELPER
// ================================================================
void sendLoRaPacket(String packetData) {
  // LoRa.beginPacket() starts packet creation.
  LoRa.beginPacket();
  LoRa.print(packetData);
  LoRa.endPacket(); // Non-blocking/blocking send depending on library state
}

// ================================================================
// GPS INGESTION
// ================================================================
void processGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

// ================================================================
// KALMAN FILTER & IMU HELPERS
// ================================================================
void update1DKalman(float &state, float &uncertainty, float gyroRate, float accelAngle) {
  state += dt * gyroRate;
  uncertainty += dt * dt * 16.0;
  float gain = uncertainty / (uncertainty + 9.0);
  state += gain * (accelAngle - state);
  uncertainty *= (1.0 - gain);
}

void init3DKalman(Kalman3D &kf) {
  kf.State = {0, 0};
  kf.F = {1, dt, 0, 1};
  kf.G = {0.5f * dt * dt, dt};
  kf.H = {1, 0};
  kf.I = {1, 0, 0, 1};
  kf.Q = kf.G * ~kf.G * 0.25f;
  kf.R = {0.05f};
  kf.P = {0, 0, 0, 0};
}

void update3DKalman(Kalman3D &kf, float accelInput, float &outPos, float &outVel) {
  BLA::Matrix<1,1> AccelMat = {accelInput};
  kf.State = kf.F * kf.State + kf.G * AccelMat;
  kf.P = kf.F * kf.P * ~kf.F + kf.Q;
  kf.State(1,0) += accelInput * dt;
  kf.State(0,0) += kf.State(1,0) * dt + 0.5f * accelInput * dt * dt;
  outPos = kf.State(0,0);
  outVel = kf.State(1,0);
}

void initIMU() {
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
}

void readIMU() {
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  int16_t axLSB = Wire.read() << 8 | Wire.read();
  int16_t ayLSB = Wire.read() << 8 | Wire.read();
  int16_t azLSB = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  int16_t gxLSB = Wire.read() << 8 | Wire.read();
  int16_t gyLSB = Wire.read() << 8 | Wire.read();
  int16_t gzLSB = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)gxLSB / 65.5;
  RatePitch = (float)gyLSB / 65.5;
  RateYaw   = (float)gzLSB / 65.5;

  AccX = (float)axLSB / 4096.0;
  AccY = (float)ayLSB / 4096.0;
  AccZ = (float)azLSB / 4096.0;
}

void calibrateIMU() {
  for (int i = 0; i < 2000; i++) {
    readIMU();
    RateCalRoll  += RateRoll; RateCalPitch += RatePitch; RateCalYaw += RateYaw;
    AccCalX += AccX; AccCalY += AccY;
    float totalAcc = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
    AccCalZ += (AccZ - totalAcc); 
    delay(1);
  }
  RateCalRoll /= 2000.0; RateCalPitch /= 2000.0; RateCalYaw /= 2000.0;
  AccCalX /= 2000.0; AccCalY /= 2000.0; AccCalZ /= 2000.0;
}