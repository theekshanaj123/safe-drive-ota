#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <math.h>

BLEServer* pServer = NULL;
BLECharacteristic* alertCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define BUZ 2

// #define 1 1
bool state = false;

#define BUTTON_PIN 4
bool stopAlert = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/


#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define ALERT_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


// ========== MPU6050 Configuration ========== //
const int MPU_addr = 0x68;  // I2C address for MPU6050 (default: SDA=8, SCL=9 on ESP32-S3)

// Raw sensor data variables
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, Amp, gyroMagnitude;

// Processed sensor data (accelerometer in g's and gyroscope in °/s)
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

// Global variable for crash detection count (to filter out noise)
int crashCount = 0;

// Global variable to hold the time until which "CRASH DETECTED" is shown
unsigned long crashDisplayUntil = 0;

Preferences preferences;

const char* ssid = "Dialog 4G 929";
const char* password = "60c302f3";

// GitHub raw links (MUST use raw.githubusercontent.com)
const char* firmwareUrl = "https://raw.githubusercontent.com/theekshanaj123/safe-drive-ota/main/sketch_oct8a.ino.bin";
const char* versionUrl = "https://raw.githubusercontent.com/theekshanaj123/safe-drive-ota/main/version.txt";

unsigned long currentMillis = 0;
unsigned long previousMillis_LED = 0;
const long interval_LED = 300;



class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("connected");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};


void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);

  Serial.println("v2");

  preferences.begin("my-app", false);
  int currentVersion = preferences.getInt("version", 0);
  Serial.print("Stored firmware version: ");
  Serial.println(currentVersion);

  WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }
  // Serial.println("\nConnected!");

  // ======== Fetch latest version ======== //
  HTTPClient http;
  http.begin(versionUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String latestVersionStr = http.getString();
    latestVersionStr.trim();
    int latestVersion = latestVersionStr.toInt();

    Serial.printf("Latest version on GitHub: %d\n", latestVersion);

    HTTPClient httpFire;
    httpFire.begin(firmwareUrl);

    if (latestVersion > currentVersion) {
      Serial.println("New firmware available! Starting OTA update...");

      preferences.putInt("version", latestVersion);
      preferences.end();

      t_httpUpdate_return ret = httpUpdate.update(httpFire, firmwareUrl);


      switch (ret) {
        case HTTP_UPDATE_FAILED:
          // Serial.printf("Update failed: %s\n", httpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("No update available.");
          break;
        case HTTP_UPDATE_OK:

          // Serial.println("Update successful! Rebooting...");
          break;
      }
    } else {
      Serial.println("Firmware is up to date.");
    }
  } else {
    // Serial.printf("Failed to fetch version file. HTTP code: %d\n", httpCode);
  }
  http.end();

  // Create the BLE Device
  BLEDevice::init("Crash Detection");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for temperature
  alertCharacteristic = pService->createCharacteristic(
    ALERT_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // Create a BLE Descriptor
  alertCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a connection...");


  while (!deviceConnected) {
    digitalWrite(1, HIGH);
    delay(300);
    digitalWrite(1, LOW);
    delay(300);
  }

  Wire.begin();  // I2C initialization (default: SDA=8, SCL=9 on ESP32-S3)
  initMPU();
}

void loop() {
  // notify changed value
  if (deviceConnected) {
    digitalWrite(1, LOW);

    if (!mpu_read()) {
      unsigned long currentMillis = millis();
      Serial.println("Sensor read error");
      digitalWrite(1, HIGH);
      delay(1000);
      return;
    }

    processIMUData();
    detectFall();

    if (millis() < crashDisplayUntil) {

      // 10 time loop
      for (int i = 1; i < 10; i++) {
        currentMillis = millis();

        if (digitalRead(BUTTON_PIN) == LOW) {
          // Serial.println("PULL");
          stopAlert = true;
          crashDisplayUntil = millis();
          break;
        }
        Serial.println("CRASH WAS DETECTED");

        if (currentMillis - previousMillis_LED >= interval_LED) {
          if (state) {
            tone(BUZ, 5000);
          } else {
            noTone(BUZ);
          }
          digitalWrite(1, state);
          state = !state;
        }
      }

      // Not stop
      while (!stopAlert) {
        currentMillis = millis();

        alertCharacteristic->setValue("CRASH WAS DETECTED");
        alertCharacteristic->notify();

        if (currentMillis - previousMillis_LED >= interval_LED) {
          if (state) {
            tone(BUZ, 5000);
          } else {
            noTone(BUZ);
          }
          digitalWrite(1, state);
          state = !state;
        }
      }
    }
    crashDisplayUntil = millis();
    Serial.println(stopAlert);
    delay(20);
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");

    oldDeviceConnected = deviceConnected;

    while (!deviceConnected) {
      currentMillis = millis();
      if (currentMillis - previousMillis_LED >= interval_LED) {
        digitalWrite(1, state);
        state = !state;
      }
    }
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  delay(100);
}



void initMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

// ========== Process Sensor Data ========== //
void processIMUData() {
  ax = (AcX - 2050) / 16384.0;
  ay = (AcY - 77) / 16384.0;
  az = (AcZ - 1947) / 16384.0;

  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
}

// ========== Crash Detection Algorithm ========== //
void detectFall() {
  float Raw_Amp = sqrt(ax * ax + ay * ay + az * az);
  Amp = Raw_Amp * 10;
  char buffer[6];

  gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

  Serial.print("Amp: ");
  Serial.print(Amp);
  Serial.print("\t");
  Serial.print("  Gyro: ");
  Serial.println(gyroMagnitude);

  if (Amp >= 30) {
    crashCount++;
    if (crashCount >= 3) {
      stopAlert = false;
      Serial.println("CRASH WAS DETECTED");
      crashDisplayUntil = millis() + 10000;
      crashCount = 0;
    }
  } else {
    crashCount = 0;
  }

  sprintf(buffer, "%02d", 0);
  alertCharacteristic->setValue(buffer);
  alertCharacteristic->notify();
}

// ========== Read Sensor Data from MPU6050 ========== //
bool mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom(MPU_addr, 14, true);
  if (Wire.available() < 14) {
    return false;
  }
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  return true;
}
