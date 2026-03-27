#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// --- CONFIGURACIÓN BLE ---
#define HEART_RATE_SERVICE_UUID      BLEUUID((uint16_t)0x180D)
#define HEART_RATE_CHAR_UUID         BLEUUID((uint16_t)0x2A37)

BLECharacteristic *pHeartRateChar;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { 
      deviceConnected = false;
      BLEDevice::startAdvertising(); // Reiniciar publicidad al desconectar
    }
};

// --- CONFIGURACIÓN SENSOR ---
MAX30105 sensor;
#define SDA_PIN 8
#define SCL_PIN 9
#define IR_THRESHOLD 50000

uint32_t irBuffer[100]; 
uint32_t redBuffer[100];
int32_t bufferLength = 100; 
int32_t spo2;               
int8_t validSPO2;           
int32_t heartRateMaxim;    
int8_t validHeartRate;     

long lastBeat = 0;
float bpmFiltered = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // 1. Inicializar Sensor
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Sensor no encontrado.");
    while (1);
  }
  sensor.setup(60, 4, 2, 100, 411, 4096);

  // 2. Inicializar BLE
  BLEDevice::init("VitaPulse-ESP32"); // Nombre que aparecerá en tu app
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pHeart = pServer->createService(HEART_RATE_SERVICE_UUID);
  pHeartRateChar = pHeart->createCharacteristic(
                      HEART_RATE_CHAR_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                   );
  pHeartRateChar->addDescriptor(new BLE2902());
  pHeart->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HEART_RATE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE Activo. Esperando conexión...");
}

void loop() {
  // Llenar buffer inicial
  for (byte i = 0 ; i < bufferLength ; i++) {
    while (sensor.available() == false) sensor.check(); 
    redBuffer[i] = sensor.getFIFORed();
    irBuffer[i] = sensor.getFIFOIR();
    sensor.nextSample();
  }

  while (1) {
    // Ventana deslizante (75 viejas, 25 nuevas)
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    for (byte i = 75; i < 100; i++) {
      while (sensor.available() == false) sensor.check(); 
      redBuffer[i] = sensor.getFIFORed();
      irBuffer[i] = sensor.getFIFOIR();

      if (checkForBeat(irBuffer[i]) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        float currentBPM = 60.0 / (delta / 1000.0);
        
        if (currentBPM > 40 && currentBPM < 160) {
          bpmFiltered = (bpmFiltered == 0) ? currentBPM : (bpmFiltered * 0.8) + (currentBPM * 0.2);
          
          // ENVIAR POR BLE CADA VEZ QUE HAY UN LATIDO VÁLIDO
          if (deviceConnected) {
    uint8_t sensorData[3];
    sensorData[0] = 0b00000110;          // Flags
    sensorData[1] = (uint8_t)bpmFiltered; // Byte 1: Pulso
    sensorData[2] = (uint8_t)spo2;        // Byte 2: Oxígeno (SpO2)
    
    pHeartRateChar->setValue(sensorData, 3); // Enviamos 3 bytes
    pHeartRateChar->notify();
}
        }
      }
      sensor.nextSample();
    }

    if (irBuffer[99] < IR_THRESHOLD) {
      bpmFiltered = 0;
      break; 
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRateMaxim, &validHeartRate);
    
    // Debug por Serial
    Serial.print("BPM: "); Serial.print((int)bpmFiltered);
    Serial.print(" | SpO2: "); Serial.print(validSPO2 ? spo2 : 0); Serial.println("%");
  }
}