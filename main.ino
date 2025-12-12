/*
 * NodeMCU Multi-Sensor with Reliable WiFi
 * Fixes WiFi error -301
 */

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ThingSpeak.h>

// ========== CONFIGURATION ==========
const char* ssid = "AYITO LADU";
const char* password = "ayitoladu007";

const unsigned long channelID = 3197779;
const char* writeAPIKey = "BELYYIAT4DWVJUNN";

const int soilPin = A0;
const int mpuAddress = 0x68;

// Soil calibration - UPDATE THESE VALUES
const int DRY_VALUE = 776;    // Your sensor in DRY air
const int WET_VALUE = 275;    // Your sensor in WATER

// MPU data
float temperature = 0;
float pitch = 0, roll = 0;
float vibration = 0;
float totalAccel = 0;
int wifiSignal = 0;
int motionState = 0;

// Timing
const long uploadInterval = 20000;
unsigned long lastUpload = 0;
unsigned long lastMPURead = 0;
const int MPU_READ_INTERVAL = 50;

// WiFi client
WiFiClient client;

// Sensor values
int moisture = 0;
int rawSoil = 0;

// WiFi status
bool wifiConnected = false;
int wifiRetryCount = 0;
const int MAX_WIFI_RETRIES = 5;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n====================================");
  Serial.println("   NodeMCU Sensor System v2.0");
  Serial.println("   WiFi Error -301 Fix");
  Serial.println("====================================\n");
  
  // Initialize I2C for MPU6050
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
  
  // Connect to WiFi with retry logic
  connectWiFiWithRetry();
  
  // Only initialize ThingSpeak if WiFi is connected
  if (wifiConnected) {
    ThingSpeak.begin(client);
    Serial.println("✅ ThingSpeak initialized");
  } else {
    Serial.println("⚠️ ThingSpeak disabled - No WiFi");
  }
  
  Serial.println("\n✅ System Initialized");
  Serial.print("Soil Calibration: Dry=");
  Serial.print(DRY_VALUE);
  Serial.print(" (0%), Wet=");
  Serial.print(WET_VALUE);
  Serial.println(" (100%)");
  Serial.println("====================================\n");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read soil sensor
  readSoil();
  
  // Read MPU6050 at fixed interval
  if (currentTime - lastMPURead >= MPU_READ_INTERVAL) {
    readMPU();
    calculateTiltVibration();
    lastMPURead = currentTime;
  }
  
  // Update WiFi signal if connected
  if (wifiConnected) {
    wifiSignal = WiFi.RSSI();
  } else {
    wifiSignal = 0;
    // Try to reconnect WiFi every 30 seconds
    static unsigned long lastWifiTry = 0;
    if (currentTime - lastWifiTry >= 30000) {
      connectWiFiWithRetry();
      lastWifiTry = currentTime;
    }
  }
  
  // Update motion state
  motionState = (vibration > 0.1) ? 1 : 0;
  
  // Display data every second
  static unsigned long lastDisplay = 0;
  if (currentTime - lastDisplay >= 1000) {
    showData();
    lastDisplay = currentTime;
  }
  
  // Upload to ThingSpeak only if WiFi is connected
  if (wifiConnected && currentTime - lastUpload >= uploadInterval) {
    uploadToThingSpeak();
    lastUpload = currentTime;
  }
}

// ========== WIFI FUNCTIONS ==========

void connectWiFiWithRetry() {
  Serial.print("\n🔗 Connecting to WiFi: ");
  Serial.println(ssid);
  
  // Disconnect first to clear any previous state
  WiFi.disconnect(true);
  delay(1000);
  
  // Set WiFi mode
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  // Start connection
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  unsigned long startTime = millis();
  bool connected = false;
  
  while (millis() - startTime < 15000) {  // 15 second timeout
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    Serial.print(".");
    delay(500);
  }
  
  if (connected) {
    wifiConnected = true;
    wifiRetryCount = 0;
    
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("   IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("   Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    // Reinitialize ThingSpeak if needed
    ThingSpeak.begin(client);
  } else {
    wifiConnected = false;
    wifiRetryCount++;
    
    Serial.println("\n❌ WiFi Connection Failed!");
    Serial.print("   Error: ");
    
    switch(WiFi.status()) {
      case WL_IDLE_STATUS:
        Serial.println("WiFi idle");
        break;
      case WL_NO_SSID_AVAIL:
        Serial.println("SSID not found");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("Wrong password");
        break;
      case WL_DISCONNECTED:
        Serial.println("Disconnected");
        break;
      default:
        Serial.print("Unknown error: ");
        Serial.println(WiFi.status());
    }
    
    if (wifiRetryCount < MAX_WIFI_RETRIES) {
      Serial.print("   Retry ");
      Serial.print(wifiRetryCount);
      Serial.print(" of ");
      Serial.println(MAX_WIFI_RETRIES);
    } else {
      Serial.println("   Max retries reached. Working offline.");
    }
  }
}

// ========== SENSOR FUNCTIONS ==========

void readSoil() {
  rawSoil = analogRead(soilPin);
  
  // Calculate moisture
  if (rawSoil >= DRY_VALUE) {
    moisture = 0;
  } else if (rawSoil <= WET_VALUE) {
    moisture = 100;
  } else {
    moisture = map(rawSoil, DRY_VALUE, WET_VALUE, 0, 100);
  }
  
  moisture = constrain(moisture, 0, 100);
}

void readMPU() {
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)mpuAddress, (size_t)14, (bool)true);
  
  if (Wire.available() >= 14) {
    // Read accelerometer
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    
    // Read temperature
    int16_t t = Wire.read() << 8 | Wire.read();
    
    // Skip gyroscope
    for (int i = 0; i < 6; i++) Wire.read();
    
    // Convert values
    float accX = ax / 16384.0;
    float accY = ay / 16384.0;
    float accZ = az / 16384.0;
    
    temperature = t / 340.0 + 36.53;
    
    // Store for vibration
    static float lastAccX = 0, lastAccY = 0, lastAccZ = 0;
    
    // Total acceleration
    totalAccel = sqrt(accX*accX + accY*accY + accZ*accZ);
    
    // Calculate tilt
    pitch = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180.0 / PI;
    roll = atan2(accY, accZ) * 180.0 / PI;
    
    // Calculate vibration
    float deltaX = accX - lastAccX;
    float deltaY = accY - lastAccY;
    float deltaZ = (accZ - 1.0) - (lastAccZ - 1.0);
    
    vibration = sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
    
    lastAccX = accX;
    lastAccY = accY;
    lastAccZ = accZ;
  }
}

void calculateTiltVibration() {
  // Already calculated in readMPU()
}

// ========== DISPLAY FUNCTION ==========

void showData() {
  Serial.println("================================");
  
  // Soil data
  Serial.print("🌱 SOIL: ");
  Serial.print(moisture);
  Serial.print("% (RAW VALUE: ");
  Serial.print(rawSoil);
  Serial.println(")");
  
  // Tilt data
  Serial.print("📐 TILT: Pitch=");
  Serial.print(pitch, 1);
  Serial.print("°, Roll=");
  Serial.print(roll, 1);
  Serial.println("°");
  
  // Vibration & temperature
  Serial.print("📳 VIB: ");
  Serial.print(vibration * 1000, 0);
  Serial.print("mg | 🌡️ TEMP: ");
  Serial.print(temperature, 1);
  Serial.println("°C");
  
  // Acceleration
  Serial.print("⚡ ACCEL: ");
  Serial.print(totalAccel, 2);
  Serial.print("g | MOTION: ");
  Serial.println(motionState ? "YES" : "NO");
  
  // WiFi status
  Serial.print("📶 WIFI: ");
  if (wifiConnected) {
    Serial.print(wifiSignal);
    Serial.print(" dBm | IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("DISCONNECTED");
  }
  
  // Upload status
  long timeLeft = uploadInterval - (millis() - lastUpload);
  Serial.print("📤 UPLOAD: ");
  if (wifiConnected) {
    Serial.print(timeLeft / 1000);
    Serial.println("s");
  } else {
    Serial.println("OFFLINE");
  }
  
  Serial.println("================================");
}

// ========== THINGSPEAK UPLOAD ==========

void uploadToThingSpeak() {
  Serial.println("\n📡 Uploading to ThingSpeak...");
  
  // Double-check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi disconnected. Skipping upload.");
    wifiConnected = false;
    return;
  }
  
  // Prepare data
  ThingSpeak.setField(1, moisture);
  ThingSpeak.setField(2, pitch);
  ThingSpeak.setField(3, roll);
  ThingSpeak.setField(4, vibration * 1000);
  ThingSpeak.setField(5, temperature);
  ThingSpeak.setField(6, totalAccel);
  ThingSpeak.setField(7, wifiSignal);
  ThingSpeak.setField(8, motionState);
  
  // Send with timeout
  unsigned long startTime = millis();
  int result = ThingSpeak.writeFields(channelID, writeAPIKey);
  
  if (result == 200) {
    Serial.println("✅ Upload successful!");
    Serial.print("   Time: ");
    Serial.print(millis() - startTime);
    Serial.println("ms");
  } else {
    Serial.print("❌ Upload failed. Error: ");
    Serial.println(result);
    
    // Common error codes:
    switch(result) {
      case -301:
        Serial.println("   WiFi connection lost");
        wifiConnected = false;
        break;
      case 404:
        Serial.println("   Channel ID or API key incorrect");
        break;
      case 400:
        Serial.println("   Invalid data or rate limit exceeded");
        break;
      default:
        Serial.print("   Unknown error code: ");
        Serial.println(result);
    }
  }
}