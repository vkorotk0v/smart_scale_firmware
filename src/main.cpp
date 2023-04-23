#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "HX711.h"
#include <SimpleKalmanFilter.h>

// Time interval for sending data
const unsigned long interval = 100;
unsigned long previousMillis = 0;
bool isTaring = false;

// Counter and time variables for incrementing every second
double weight = 0;
unsigned long previousSecondMillis = 0;
const int ledPin = 5;

#define DOUT  16
#define CLK  17

HX711 scale;

SimpleKalmanFilter kalmanFilter(0.3, 4, 0.1);

// Custom service and characteristic UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define COUNTER_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TARE_CHARACTERISTIC_UUID "156858ce-e2bf-4e1f-8d72-0b4df0a9f7e6"

// BLE characteristics
BLECharacteristic *pCounterCharacteristic;
BLECharacteristic *pTareCharacteristic;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Connected to device");
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Disconnected from device");
    BLEDevice::startAdvertising();
  }
};

class TareCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pTareCharacteristic) {
    isTaring = true;
    scale.tare();
    Serial.println("Tared");
    isTaring = false;
  }
};

void calibrate_scale() {
  Serial.println("Remove all weight from the scale and press any key to continue...");
  while (Serial.available() == 0) {}
  while (Serial.read() != -1) {}

  scale.set_scale(); 
  scale.tare(); 

  Serial.println("Place the 100g weight on the scale and press any key to continue...");
  while (Serial.available() == 0) {}
  while (Serial.read() != -1) {}

  long weight_reading = scale.get_units(10);
  float calibration_factor = weight_reading/100;
  Serial.print("Calibration factor: ");
  Serial.println(calibration_factor, 5);

  scale.set_scale(calibration_factor);
  scale.tare();
}


void setup() {
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);

  // Initialize BLE
  BLEDevice::init("ESP32_Counter");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Counter characteristic
  pCounterCharacteristic = pService->createCharacteristic(
                           COUNTER_CHARACTERISTIC_UUID,
                           BLECharacteristic::PROPERTY_READ |
                           BLECharacteristic::PROPERTY_NOTIFY
                         );
  pCounterCharacteristic->addDescriptor(new BLE2902());

  // Tare characteristic
  pTareCharacteristic = pService->createCharacteristic(
                         TARE_CHARACTERISTIC_UUID,
                         BLECharacteristic::PROPERTY_WRITE
                       );

  pTareCharacteristic->setCallbacks(new TareCharacteristicCallbacks());

  pServer->setCallbacks(new ServerCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE Counter Ready");
  scale.begin(DOUT, CLK);
  // calibrate_scale();
  scale.set_scale(-427);
  scale.tare();
}

void loop() {
  unsigned long currentMillis = millis();


  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (isTaring) return;
    
    double raw_weight= scale.get_units(1);
    int weightInt = kalmanFilter.updateEstimate(raw_weight)*10;
    weight = weightInt/10.0;

    if (int(weight) % 2 == 0) {
      digitalWrite(ledPin, HIGH);
    } else {
      digitalWrite(ledPin, LOW);
    }

    pCounterCharacteristic->setValue(weight);
    pCounterCharacteristic->notify();

    Serial.println(weight);
  }
}
