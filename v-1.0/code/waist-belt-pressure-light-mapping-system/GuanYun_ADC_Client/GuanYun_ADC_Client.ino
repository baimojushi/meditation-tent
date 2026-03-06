// GuanYun_ADC_Client - 稳定版（125Hz ADC采样）
#include <Arduino.h>
#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLE2902.h>
#include "esp_log.h"

// -------------------------- 参数 --------------------------
const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 255;
const int BUFFER_SIZE = 300;
const CRGB BASE_WARM_WHITE = CRGB(255, 160, 40);
const int DIM_THRESHOLD = 40;
const int BRIGHT_THRESHOLD = 215;
const float SMOOTH_UP_FACTOR = 0.12f;
const float SMOOTH_DOWN_FACTOR = 0.12f;
const int BRIGHTNESS_THRESHOLD = 2;
const float TRIM_RATIO = 0.25f;

// -------------------------- 硬件引脚 --------------------------
const int POT_PIN = 7;  // ADC1_CH0
const int BUTTON_PIN = 16;
const int BUTTON_LED_PIN = 17;

// -------------------------- BLE配置 --------------------------
#define SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEClient* pBLEClient = nullptr;
BLERemoteCharacteristic* pRemoteChar = nullptr;
BLEServer* pServer = nullptr;
BLECharacteristic* pServerChar = nullptr;
BLEScan* pBLEScan = nullptr;

bool bleConnected = false;
uint8_t bleRole = 0; // 0=未确定，1=Server，2=Client

// -------------------------- 数据结构 --------------------------
typedef struct {
  bool programRunning;
  int brightness;
} DataPacket;
DataPacket txPacket;

// -------------------------- 全局变量 --------------------------
int ledcChannel;
int voltageBuffer[BUFFER_SIZE];
int voltageBufferIndex = 0;
int rawMaxVoltage = 0, rawMinVoltage = 4095;
int maxVoltage = 0, minVoltage = 4095;
int lastUpdateBufferIndex = 0;
unsigned long lastUpdateTime = 0;

int buttonState = HIGH, lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

volatile int lastButtonLedBrightness = MIN_BRIGHTNESS;
portMUX_TYPE adcBleMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long lastBleScanTime = 0;
const unsigned long BLE_SCAN_INTERVAL = 3000;

// -------------------------- 日志宏 --------------------------
#define LINFO(...)    Serial.printf(__VA_ARGS__); Serial.println();
#define LPRINT(...)   Serial.print(__VA_ARGS__);

// -------------------------- 辅助函数 --------------------------
CRGB getDynamicColor(int brightness) {
  int outputBrightness = constrain(brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
  if (outputBrightness < DIM_THRESHOLD) {
    float f = (float)outputBrightness / DIM_THRESHOLD;
    return CRGB(constrain((int)(BASE_WARM_WHITE.r * f),0,255),
                constrain((int)(BASE_WARM_WHITE.g * f),0,255),
                constrain((int)(BASE_WARM_WHITE.b * f),0,255));
  } else if (outputBrightness > BRIGHT_THRESHOLD) {
    float f = 1.0f + (float)(outputBrightness - BRIGHT_THRESHOLD) / (MAX_BRIGHTNESS - BRIGHT_THRESHOLD) * 0.3f;
    return CRGB(constrain((int)(BASE_WARM_WHITE.r * f),0,255),
                constrain((int)(BASE_WARM_WHITE.g * f),0,255),
                constrain((int)(BASE_WARM_WHITE.b * f),0,255));
  }
  return BASE_WARM_WHITE;
}

int applySmoothing(int target, int last, float factor) {
  return (int)(target * factor + last * (1 - factor));
}

// -------------------------- 按钮处理 --------------------------
void handleButtonInput() {
  int currentReading = digitalRead(BUTTON_PIN);
  if (currentReading != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (currentReading != buttonState) {
      buttonState = currentReading;
      if (buttonState == HIGH) {
        txPacket.programRunning = !txPacket.programRunning;
        LINFO("[BTN] 切换程序状态：%s", txPacket.programRunning ? "运行" : "暂停");
        if (txPacket.programRunning) {
          int val = analogRead(POT_PIN);
          portENTER_CRITICAL(&adcBleMux);
          for (int i = 0; i < BUFFER_SIZE; i++) voltageBuffer[i] = val;
          portEXIT_CRITICAL(&adcBleMux);
          lastUpdateTime = millis();
          lastUpdateBufferIndex = voltageBufferIndex;
        }
      }
    }
  }
  lastButtonState = currentReading;
}

// -------------------------- BLE回调 --------------------------
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override { bleConnected = true; LINFO("[BLE] Client connected"); }
  void onDisconnect(BLEClient* pclient) override { bleConnected = false; pRemoteChar = nullptr; bleRole = 0; LINFO("[BLE] Client disconnected"); }
};
class MyServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer* pserver) override { bleConnected = true; LINFO("[BLE] Server connected"); }
  void onDisconnect(BLEServer* pserver) override { bleConnected = false; pserver->startAdvertising(); LINFO("[BLE] Server disconnected, re-advertising"); }
};

// -------------------------- BLE Server 初始化 --------------------------
void setupBleServerOnce() {
  if (pServer) return;
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallback());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pServerChar = pService->createCharacteristic(
    DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_NOTIFY
  );
  pServerChar->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
  bleRole = 1;
  LINFO("[BLE] Server started & advertising");
}

// -------------------------- BLE扫描 & 连接 --------------------------
void bleScanAndConnect_once() {
  unsigned long now = millis();
  if (bleConnected || now - lastBleScanTime < BLE_SCAN_INTERVAL) return;
  lastBleScanTime = now;

  if (!pBLEScan) pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);

  BLEScanResults* results = pBLEScan->start(2, false);
  if (!results) return;

  int cnt = results->getCount();
  for (int i = 0; i < cnt; i++) {
    BLEAdvertisedDevice dev = results->getDevice(i);
    String name = dev.getName().c_str();
    if (name.indexOf("ESP32S3_Master") != -1 || dev.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      if (pBLEClient) { delete pBLEClient; pBLEClient = nullptr; }
      pBLEClient = BLEDevice::createClient();
      pBLEClient->setClientCallbacks(new MyClientCallback());
      if (pBLEClient->connect(&dev)) {
        BLERemoteService* pService = pBLEClient->getService(SERVICE_UUID);
        if (pService) {
          pRemoteChar = pService->getCharacteristic(DATA_CHAR_UUID);
          if (pRemoteChar) {
            bleConnected = true;
            bleRole = 2;
            pBLEScan->clearResults();
            return;
          }
        }
        pBLEClient->disconnect();
      }
    }
  }
  pBLEScan->clearResults();
  if (!bleConnected && bleRole != 1) setupBleServerOnce();
}

// -------------------------- ADC Task（Core0，125Hz） --------------------------
void adcTask(void* pvParameters) {
  unsigned long lastSampleTime = 0, lastSerial = 0;

  for (;;) {
    handleButtonInput();

    unsigned long now = millis();
    // 125Hz -> 8ms采样一次
    if (now - lastSampleTime >= 8) {
      int v;
      portENTER_CRITICAL(&adcBleMux);
      v = analogRead(POT_PIN);
      voltageBuffer[voltageBufferIndex] = v;
      voltageBufferIndex = (voltageBufferIndex + 1) % BUFFER_SIZE;
      portEXIT_CRITICAL(&adcBleMux);
      lastSampleTime = now;

      int targetBrightness = map(v, minVoltage, maxVoltage, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
      targetBrightness = constrain(targetBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
      int smoothed = (targetBrightness > lastButtonLedBrightness) ?
                       applySmoothing(targetBrightness, lastButtonLedBrightness, SMOOTH_UP_FACTOR) :
                       applySmoothing(targetBrightness, lastButtonLedBrightness, SMOOTH_DOWN_FACTOR);
      smoothed = constrain(smoothed, MIN_BRIGHTNESS, MAX_BRIGHTNESS);

      if (abs(smoothed - lastButtonLedBrightness) >= BRIGHTNESS_THRESHOLD) {
        portENTER_CRITICAL(&adcBleMux);
        lastButtonLedBrightness = smoothed;
        ledcWrite(BUTTON_LED_PIN, lastButtonLedBrightness);
        portEXIT_CRITICAL(&adcBleMux);
      }
    }

    // 每5秒更新极值
    if (now - lastUpdateTime >= 5000) {
      int rMax=-1, rMin=4096;
      int start=lastUpdateBufferIndex, end=voltageBufferIndex;
      if (start<=end) { for(int i=start;i<end;i++){ rMax=max(rMax,voltageBuffer[i]); rMin=min(rMin,voltageBuffer[i]); } }
      else { for(int i=start;i<BUFFER_SIZE;i++){ rMax=max(rMax,voltageBuffer[i]); rMin=min(rMin,voltageBuffer[i]); }
             for(int i=0;i<end;i++){ rMax=max(rMax,voltageBuffer[i]); rMin=min(rMin,voltageBuffer[i]); } }
      rawMaxVoltage=(rMax==-1)?maxVoltage+1:rMax;
      rawMinVoltage=(rMin==4096)?minVoltage-1:rMin;
      int range = rawMaxVoltage-rawMinVoltage; if(range<=0) range=200;
      int trim=max(1,(int)(range*TRIM_RATIO));
      maxVoltage=rawMaxVoltage-trim;
      minVoltage=rawMinVoltage+trim;
      if(maxVoltage<=minVoltage){ minVoltage=rawMinVoltage; maxVoltage=minVoltage+200; }
      lastUpdateBufferIndex=voltageBufferIndex;
      lastUpdateTime=now;
    }

    // 每秒打印一次
    if(now-lastSerial>=1000){
      CRGB color=getDynamicColor(lastButtonLedBrightness);
      LINFO("[ADC] brightness=%d RGB=(%d,%d,%d)", lastButtonLedBrightness,color.r,color.g,color.b);
      lastSerial=now;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// -------------------------- BLE Task（Core1） --------------------------
void bleTask(void* pvParameters) {
  for(;;){
    bleScanAndConnect_once();

    if(bleConnected && bleRole==2 && pRemoteChar){
      portENTER_CRITICAL(&adcBleMux);
      txPacket.brightness=lastButtonLedBrightness;
      pRemoteChar->writeValue((uint8_t*)&txPacket,sizeof(txPacket),false);
      portEXIT_CRITICAL(&adcBleMux);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// -------------------------- setup --------------------------
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("\n=== GuanYun ADC Client - startup ===");
  esp_log_level_set("*", ESP_LOG_ERROR);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_LED_PIN, OUTPUT);
  ledcChannel = ledcAttach(BUTTON_LED_PIN, 500, 8);
  ledcWrite(BUTTON_LED_PIN, lastButtonLedBrightness);
  analogSetPinAttenuation(POT_PIN, ADC_11db);

  int v = analogRead(POT_PIN);
  for(int i=0;i<BUFFER_SIZE;i++) voltageBuffer[i]=v;
  lastUpdateBufferIndex=voltageBufferIndex;
  txPacket.programRunning=false;

  BLEDevice::init("ESP32S3_Collector");
  LINFO("[SYS] BLEDevice initialized");

  BaseType_t r1=xTaskCreatePinnedToCore(adcTask,"ADC_Task",8192,NULL,1,NULL,0);
  BaseType_t r2=xTaskCreatePinnedToCore(bleTask,"BLE_Task",16384,NULL,3,NULL,1);

  if (r1==pdPASS){
    LINFO("[SYS] ADC task started (core0)");
  } else { 
    LINFO("[SYS] ADC task failed");
  }
  if (r2==pdPASS){
    LINFO("[SYS] BLE task started (core1)"); 
  } else {
    LINFO("[SYS] BLE task failed");
  }

  Serial.println("=== setup done ===");
}

void loop(){ vTaskDelay(pdMS_TO_TICKS(2000)); }


