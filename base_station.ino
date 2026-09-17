#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>

// ================================================================
// PIN DEFINITIONS FOR SX1276
// ================================================================
#define LORA_NSS   5
#define LORA_RST   27
#define LORA_DIO0  26

const long LORA_FREQUENCY = 433E6; // 433 MHz (Must match TX)

// ================================================================
// GPS DEFINITIONS (ESP32 Hardware Serial 2)
// ================================================================
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
const uint32_t GPS_BAUD = 9600; // 9600 for base station

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // Use ESP32 UART2

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("--- LoRa SX1276 & Local GPS Receiver ---"));

  // Initialize GPS Hardware Serial
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println(F("[INFO] GPS Initialized on Pins 16 (RX) & 17 (TX)."));

  // Configure LoRa module pins
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  // Initialize LoRa at 433 MHz
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println(F("[ERROR] LoRa initialization failed! Check wiring."));
    while (1); // Halt execution on failure
  }

  Serial.println(F("[INFO] LoRa SX1276 Initialized Successfully."));
  Serial.println(F("---------------------------------------------------------------------"));
}


void loop() {
  // 1. Constantly feed GPS data to the TinyGPS++ parser
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Check if a LoRa packet has arrived
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String receivedData = "";

    // Read packet payload character by character
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    // ================================================================
    // DISPLAY ALL RECEIVED DATA
    // ================================================================
    Serial.println(F("====================================================================="));
    
    // --- LoRa Data ---
    Serial.print(F("[RX] LoRa Packet Received (Size: "));
    Serial.print(packetSize);
    Serial.println(F(" bytes)"));
    Serial.print(F("Payload: "));
    Serial.println(receivedData);
    Serial.print(F("Signal : RSSI "));
    Serial.print(LoRa.packetRssi());
    Serial.print(F(" dBm | SNR "));
    Serial.print(LoRa.packetSnr());
    Serial.println(F(" dB"));
    Serial.println(F("---------------------------------------------------------------------"));

    // --- Local GPS Data ---
    Serial.println(F("[GPS] Local GPS Status:"));
    
    // Latitude & Longitude
    if (gps.location.isValid()) {
      Serial.print(F("Location  : ")); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(", ")); 
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println(F("Location  : INVALID / SEARCHING..."));
    }

    // Altitude
    if (gps.altitude.isValid()) {
      Serial.print(F("Altitude  : ")); 
      Serial.print(gps.altitude.meters()); 
      Serial.println(F(" m"));
    } else {
      Serial.println(F("Altitude  : INVALID / SEARCHING..."));
    }

    // UTC Time
    if (gps.time.isValid()) {
      Serial.print(F("UTC Time  : "));
      if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.println(gps.time.second());
    } else {
      Serial.println(F("UTC Time  : INVALID / SEARCHING..."));
    } 

    // Satellites
    if (gps.satellites.isValid()) {
      Serial.print(F("Satellites: ")); 
      Serial.println(gps.satellites.value());
    } else {
      Serial.println(F("Satellites: INVALID / SEARCHING..."));
    }
    
    Serial.println(F("=====================================================================\n"));
  }
}
