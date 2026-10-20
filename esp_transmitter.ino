
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// -------- YOUR WIFI --------
const char* WIFI_SSID = "your ssid";
const char* WIFI_PASSWORD = "your password";

// -------- UDP SETTINGS --------
IPAddress receiverIP(255, 255, 255, 255);
const uint16_t receiverPort = 4210;
const uint16_t localPort = 4211;

WiFiUDP udp;

// -------- HARDWARE PINS --------
const uint8_t SDA_PIN = 4;       // GPIO4 / D2
const uint8_t SCL_PIN = 5;       // GPIO5 / D1
const uint8_t BUTTON_A = 14;     // GPIO14 / D5
const uint8_t BUTTON_B = 12;     // GPIO12 / D6

// -------- MPU SETTINGS --------
const uint8_t MPU_ADDR = 0x68;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 20; // 50 packets/sec

bool mpuReady = false;

// Read a signed 16-bit MPU register pair
bool readMPU16(uint8_t reg, int16_t &value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((uint8_t)MPU_ADDR,
                       (uint8_t)2) != 2) {
    return false;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  value = (int16_t)((highByte << 8) | lowByte);
  return true;
}

// Write an MPU register
bool writeMPU(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// -------- WIFI CONNECTION --------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected!");

  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Broadcast destination: ");
  Serial.println(receiverIP);

  udp.begin(localPort);
  Serial.println("UDP transmitter ready.");
}

// -------- MPU INITIALIZATION --------
bool initializeMPU() {
  Serial.println("Checking MPU sensor...");

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: MPU not found at 0x68.");
    return false;
  }

  // Wake sensor from sleep mode
  if (!writeMPU(0x6B, 0x00)) {
    Serial.println("ERROR: Could not wake MPU.");
    return false;
  }

  delay(100);

  // Read WHO_AM_I (MPU-6500 commonly returns 0x70)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom((uint8_t)MPU_ADDR,
                       (uint8_t)1) != 1) {
    Serial.println("ERROR: Could not read sensor ID.");
    return false;
  }

  uint8_t sensorID = Wire.read();

  Serial.print("Sensor WHO_AM_I: 0x");
  Serial.println(sensorID, HEX);

  if (sensorID != 0x70 && sensorID != 0x68) {
    Serial.println("WARNING: Unexpected sensor ID.");
  }

  Serial.println("MPU initialized.");
  return true;
}

// -------- READ ALL MOTION DATA --------
bool readMotion() {
  bool ok = true;

  ok &= readMPU16(0x3B, ax);
  ok &= readMPU16(0x3D, ay);
  ok &= readMPU16(0x3F, az);

  ok &= readMPU16(0x43, gx);
  ok &= readMPU16(0x45, gy);
  ok &= readMPU16(0x47, gz);

  return ok;
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("ESP-12E Wii Remote starting...");

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  mpuReady = initializeMPU();

  connectWiFi();
}

// -------- MAIN LOOP --------
void loop() {
  // Reconnect automatically if Wi-Fi drops
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println();
    Serial.println("Wi-Fi reconnected.");
    udp.begin(localPort);
  }

  if (millis() - lastSend < SEND_INTERVAL_MS) {
    return;
  }
  lastSend = millis();

  // Retry sensor initialization if it wasn't detected at startup
  if (!mpuReady) {
    mpuReady = initializeMPU();
  }

  bool motionOK = mpuReady && readMotion();

  // INPUT_PULLUP means LOW = pressed
  int a = (digitalRead(BUTTON_A) == LOW) ? 1 : 0;
  int b = (digitalRead(BUTTON_B) == LOW) ? 1 : 0;

  if (!motionOK) {
    ax = ay = az = 0;
    gx = gy = gz = 0;
  }

  char packet[128];

  snprintf(packet, sizeof(packet),
           "A=%d,B=%d,AX=%d,AY=%d,AZ=%d,GX=%d,GY=%d,GZ=%d",
           a, b, ax, ay, az, gx, gy, gz);

  udp.beginPacket(receiverIP, receiverPort);
  udp.write((const uint8_t*)packet, strlen(packet));
  udp.endPacket();

  // Debug output in Serial Monitor
  //Serial.println(packet);
}
