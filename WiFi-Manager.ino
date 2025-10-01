/************************************************************
 * ESP32 BME680 Air Quality Logger with Firebase
 * - Realtime data upload every 2s
 * - Historical data upload every 10s
 * - NTP timestamp formatting
 * - Button on IO13 to reset Wi-Fi credentials
 ************************************************************/

#include <WiFiManager.h>           // WiFiManager for easy Wi-Fi configuration
#include <Wire.h>                  // I2C communication library
#include <Adafruit_BME680.h>       // BME680 sensor library
#include <FirebaseESP32.h>         // Firebase library for ESP32
#include <addons/TokenHelper.h>    // Firebase helper addon
#include <addons/RTDBHelper.h>     // Firebase Realtime Database helper
#include <time.h>                  // For NTP time functions

// ===================== Firebase Settings =====================
#define FIREBASE_HOST "https://bme680-logging-c6e44-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define API_KEY "AIzaSyC3UJyuQZWS1p3X1zMtZ5agK5T_CeD_cFE"

FirebaseData fbdo;     // Object to handle Firebase operations
FirebaseAuth auth;     // Firebase authentication object
FirebaseConfig config; // Firebase configuration object
bool firebaseAvailable = false; // Flag to indicate Firebase connection status

// ===================== Sensor =====================
Adafruit_BME680 bme;   // BME680 I2C sensor object
float temperature, humidity, pressure, gasResistance; // Sensor readings

// ===================== Timing Variables =====================
unsigned long lastSensorReadTime = 0;       // Timer for sensor reading
const unsigned long sensorReadInterval = 2000;  // Read sensor every 2s

unsigned long lastRealtimeUploadTime = 0;   // Timer for realtime Firebase upload
const unsigned long realtimeUploadInterval = 2000; // Upload realtime every 2s

unsigned long lastHistoricalUploadTime = 0; // Timer for historical Firebase upload
const unsigned long historicalUploadInterval = 10000; // Upload historical every 10s

// ===================== Button Settings =====================
#define BUTTON_PIN 13           // IO13 for Wi-Fi reset button
#define BUTTON_HOLD_TIME 1000  // Hold for 1 second to trigger Wi-Fi reset
unsigned long buttonPressTime = 0;
bool buttonPressed = false;

// ===================== WiFiManager =====================
WiFiManager wm; // Handles Wi-Fi connection and config portal

// ===================== NTP Settings =====================
const char* ntpServer = "pool.ntp.org"; // NTP server for time sync
const long gmtOffset_sec = 7 * 3600;    // GMT+7 timezone
const int daylightOffset_sec = 0;       // No daylight saving

// ===================== Firebase Initialization =====================
void initializeFirebaseNonBlocking() {
  // Only initialize if not already connected and Wi-Fi is available
  if (!firebaseAvailable && WiFi.status() == WL_CONNECTED) {
    Serial.println("[Firebase] Initializing...");
    
    // Firebase authentication
    config.api_key = API_KEY;
    config.database_url = FIREBASE_HOST;
    auth.user.email = "bme680@gmail.com";
    auth.user.password = "Bmelogging@2025";

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);      // Auto reconnect if Wi-Fi lost
    fbdo.setResponseSize(4096);        // Max response size

    if (Firebase.ready()) {
      Serial.print("UID: ");
      Serial.println(auth.token.uid.c_str());
      firebaseAvailable = true;        // Connected successfully
      Serial.println("[Firebase] ✅ Ready");
    } else {
      Serial.print("❌ Firebase init failed: ");
      Serial.println(fbdo.errorReason());
      firebaseAvailable = false;
    }
  }
}

// ===================== Upload Realtime Data =====================
void uploadRealtime() {
  if (!firebaseAvailable) return;

  FirebaseJson json;
  json.set("Timestamp/.sv", "timestamp"); // Firebase server timestamp
  json.set("Temperature", temperature);
  json.set("Humidity", humidity);
  json.set("Pressure", pressure);
  json.set("GasResistance", gasResistance);

  // Upload to Realtime node
  if (Firebase.setJSON(fbdo, "1_Real-Time_Data", json)) {
    Serial.println("✅ BME680 uploaded to Firebase (Realtime)!");
  } else {
    Serial.print("❌ Firebase upload failed (Realtime): ");
    Serial.println(fbdo.errorReason());
    firebaseAvailable = false;
  }
}

// ===================== Upload Historical Data =====================
void uploadHistorical() {
  if (!firebaseAvailable) return;

  FirebaseJson json;
  json.set("Timestamp/.sv", "timestamp");
  json.set("Temperature", temperature);
  json.set("Humidity", humidity);
  json.set("Pressure", pressure);
  json.set("GasResistance", gasResistance);

  // Create a path using current date and time for historical storage
  time_t nowTime = time(nullptr);
  struct tm* t = localtime(&nowTime);
  char path[64];
  snprintf(path, sizeof(path),
           "2_Historical_Data/%04d-%02d-%02d/%02d-%02d-%02d",
           t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
           t->tm_hour, t->tm_min, t->tm_sec);

  // Upload historical data
  if (Firebase.setJSON(fbdo, path, json)) {
    Serial.println("✅ BME680 uploaded to Firebase (Historical)!");
  } else {
    Serial.print("❌ Firebase upload failed (Historical): ");
    Serial.println(fbdo.errorReason());
    firebaseAvailable = false;
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Wire.begin(); // Start I2C bus

  // --- Initialize BME680 ---
  if (!bme.begin()) {
    Serial.println("Could not find BME680 sensor!");
    while (1) delay(10); // Stop execution if sensor not found
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150); // Heater for gas sensor
  Serial.println("BME680 found!");

  // --- Button pin ---
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Pull-up for button

  // --- Wi-Fi Setup with WiFiManager ---
  Serial.println("Starting WiFiManager...");
  wm.autoConnect("Cofig_ESP32"); // AP mode if Wi-Fi not saved
  Serial.println("WiFi connected!");

  // --- NTP Time Sync ---
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Waiting for NTP time...");
  time_t nowTime = time(nullptr);
  while (nowTime < 24 * 3600) {
    delay(500);
    Serial.print(".");
    nowTime = time(nullptr);
  }
  Serial.println("\nNTP time obtained!");
}

// ===================== Main Loop =====================
void loop() {
  unsigned long now = millis();

  // --- Button check for Wi-Fi reset ---
  if (digitalRead(BUTTON_PIN) == LOW) { // Button pressed (active LOW)
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressTime = now;
    } else if (now - buttonPressTime >= BUTTON_HOLD_TIME) {
      Serial.println("\n🔄 Button held 1s: Restarting Wi-FiManager for new credentials...");
      wm.resetSettings();              // Clear saved Wi-Fi credentials
      wm.autoConnect("Cofig_ESP32");  // Enter Wi-Fi config portal
      Serial.println("Wi-Fi reset done!");
      buttonPressed = false;           // Reset button state
    }
  } else {
    buttonPressed = false; // Reset if released before 1s
  }

  // --- Read BME680 sensor every 2s ---
  if (now - lastSensorReadTime >= sensorReadInterval) {
    lastSensorReadTime = now;

    if (!bme.performReading()) {
      Serial.println("Failed to perform reading from BME680!");
    } else {
      temperature   = bme.temperature;
      humidity      = bme.humidity;
      pressure      = bme.pressure / 100.0;        // hPa
      gasResistance = bme.gas_resistance / 1000.0; // kΩ

      // Print to Serial for monitoring
      Serial.print("Temperature: "); Serial.println(temperature);
      Serial.print("Humidity: "); Serial.println(humidity);
      Serial.print("Pressure: "); Serial.println(pressure);
      Serial.print("Gas Resistance: "); Serial.println(gasResistance);
    }
  }

  // --- Ensure Firebase is initialized ---
  initializeFirebaseNonBlocking();

  // --- Upload Realtime Data every 2s ---
  if (now - lastRealtimeUploadTime >= realtimeUploadInterval && firebaseAvailable) {
    lastRealtimeUploadTime = now;
    uploadRealtime();
  }

  // --- Upload Historical Data every 10s ---
  if (now - lastHistoricalUploadTime >= historicalUploadInterval && firebaseAvailable) {
    lastHistoricalUploadTime = now;
    uploadHistorical();
  }
}
