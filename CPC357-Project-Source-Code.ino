/*
 * Project: Smart Urban Drainage & Blockage Detection
 * Target: SDG 11 - Sustainable Cities
 * Hardware: ESP32, HC-SR04, Laser, LDR, Rain Sensor
 * Update: Telemetry now sends "YES"/"NO" strings instead of 0/1
 */

#include "VOneMqttClient.h"

// --- DEVICE ID ---
const char* MyDeviceID = "ac4840e2-6047-4198-a0a4-2243519125e7"; 

// --- PIN DEFINITIONS ---
#define TRIG_PIN    5   
#define ECHO_PIN    18  
#define LASER_PIN   4   
#define LDR_PIN     34  
#define RAIN_PIN    19  

// --- CONSTANTS ---
const int LDR_THRESHOLD = 2000;    
const int BLOCKAGE_TIME = 3000;
const int PUBLISH_INTERVAL = 2000; 

// Total Depth of the drain/container in cm
const int DRAIN_TOTAL_DEPTH = 16; 

// --- GLOBAL VARIABLES ---
VOneMqttClient voneClient;
unsigned long lastMsgTime = 0;
unsigned long blockageStartTime = 0;
bool isBlockageDetected = false;
bool blockageConfirmed = false;

// --- WIFI SETUP FUNCTION ---
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  // Use WIFI_SSID and WIFI_PASSWORD from vonesetting.h
  Serial.println(WIFI_SSID); 

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- FlowGuard Starting ---");

  // 1. CONNECT WIFI
  setup_wifi();

  // 2. Setup V-ONE
  voneClient.setup(); 

  // 3. Setup Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(RAIN_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
}

void loop() {
  // 1. Check Cloud Connection
  if (!voneClient.connected()) {
    voneClient.reconnect();
    voneClient.publishDeviceStatusEvent(MyDeviceID, true);
  }
  voneClient.loop();

  // 2. Main Logic Timer
  unsigned long cur = millis();
  
  if (cur - lastMsgTime > PUBLISH_INTERVAL) {
    lastMsgTime = cur;

    // ==========================================================
    // A. READ WATER LEVEL
    // ==========================================================
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    
    // 1. Get raw distance from sensor to water surface
    int distance = duration * 0.034 / 2;
    
    // 2. Calculate Actual Water Depth
    int waterLevel = DRAIN_TOTAL_DEPTH - distance;
    if (waterLevel < 0) waterLevel = 0;
    
    // ==========================================================
    // B. READ OTHER SENSORS
    // ==========================================================
    bool isRaining = (digitalRead(RAIN_PIN) == LOW);

    digitalWrite(LASER_PIN, HIGH); delay(10);
    int ldrValue = analogRead(LDR_PIN);
    digitalWrite(LASER_PIN, LOW);
    
    bool beamBroken = (ldrValue > LDR_THRESHOLD);

    // ==========================================================
    // C. BLOCKAGE LOGIC
    // ==========================================================
    if (beamBroken) {
      if (!isBlockageDetected) {
        blockageStartTime = millis();
        isBlockageDetected = true;
      } else if ((millis() - blockageStartTime) > BLOCKAGE_TIME) {
        blockageConfirmed = true;
      }
    } else {
      isBlockageDetected = false;
      blockageConfirmed = false;
    }

    // ==========================================================
    // D. STATUS MESSAGE & ALERT LOGIC
    // ==========================================================
    String statusMsg = "Normal";
    
    if (waterLevel > (DRAIN_TOTAL_DEPTH - 4)) statusMsg = "Critical Flood";
    else if (blockageConfirmed && !isRaining) statusMsg = "Illegal Dumping";
    else if (blockageConfirmed && isRaining) statusMsg = "Debris Flow";
    else if (isRaining) statusMsg = "Raining";

    // ==========================================================
    // E. PUBLISH TO V-ONE
    // ==========================================================
    voneClient.publishTelemetryData(MyDeviceID, "WaterLevel", waterLevel);
    
    // Send "YES" or "NO" for Rain
    voneClient.publishTelemetryData(MyDeviceID, "RainStatus", isRaining ? "YES" : "NO");
    
    // Send "YES" or "NO" for Blockage
    voneClient.publishTelemetryData(MyDeviceID, "BlockageAlert", blockageConfirmed ? "YES" : "NO");
    
    voneClient.publishTelemetryData(MyDeviceID, "StatusMsg", statusMsg.c_str());

    // Debug
    Serial.print("Water Depth: "); Serial.print(waterLevel);
    Serial.print("cm | Rain: "); Serial.print(isRaining ? "YES" : "NO");
    Serial.print(" | Blockage: "); Serial.print(blockageConfirmed ? "YES" : "NO");
    Serial.print(" | Status: "); Serial.println(statusMsg);
  }
}