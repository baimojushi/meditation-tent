#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// -------------------------- 通用参数 --------------------------
const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 255;
const int BUFFER_LOW = -10;
const int BUFFER_HIGH = 265;
const CRGB BASE_WARM_WHITE = CRGB(255, 160, 40);
const int DIM_THRESHOLD = 40;
const int BRIGHT_THRESHOLD = 215;
const float SMOOTH_UP_FACTOR = 0.12f;
const float SMOOTH_DOWN_FACTOR = 0.12f;
const int BRIGHTNESS_THRESHOLD = 2;

// -------------------------- 主控机硬件引脚 --------------------------
const int LED_STRIP_PIN = 48; //RGB_CTRL引脚

// -------------------------- 灯带参数 --------------------------
#define LED_COUNT 97
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB ledStrip[LED_COUNT];
int lastPwmValue = MIN_BRIGHTNESS;

// -------------------------- BLE配置 --------------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer* pBLEServer = nullptr;
BLECharacteristic* pDataChar = nullptr;
bool bleConnected = false;
uint8_t bleRole = 0; // 0=未确定，1=Server，2=Client

// -------------------------- 数据传输结构体 --------------------------
typedef struct {
  bool programRunning;
  int  brightness;
} DataPacket;
DataPacket rxPacket;

// -------------------------- 串口指令配置 --------------------------
const int SERIAL_BUFFER_SIZE = 10;
char serialBuffer[SERIAL_BUFFER_SIZE] = {0};
int bufferIndex = 0;
const unsigned long SERIAL_PRINT_INTERVAL = 500;

// -------------------------- 原代码函数 --------------------------
CRGB getDynamicColor(int brightness) {
  int outputBrightness = constrain(brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
  if (outputBrightness < DIM_THRESHOLD) {
    float dimFactor = (float)outputBrightness / DIM_THRESHOLD;
    return CRGB(constrain(BASE_WARM_WHITE.r * dimFactor, 0, 255),
                constrain(BASE_WARM_WHITE.g * dimFactor, 0, 255),
                constrain(BASE_WARM_WHITE.b * dimFactor, 0, 255));
  } else if (outputBrightness > BRIGHT_THRESHOLD) {
    float brightFactor = 1.0f + (float)(outputBrightness - BRIGHT_THRESHOLD) / (MAX_BRIGHTNESS - BRIGHT_THRESHOLD) * 0.3f;
    return CRGB(constrain(BASE_WARM_WHITE.r * brightFactor, 0, 255),
                constrain(BASE_WARM_WHITE.g * brightFactor, 0, 255),
                constrain(BASE_WARM_WHITE.b * brightFactor, 0, 255));
  } else {
    return BASE_WARM_WHITE;
  }
}

void listenSerialCommand() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar != '\n' && receivedChar != '\r') {
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[bufferIndex] = receivedChar;
        bufferIndex++;
        serialBuffer[bufferIndex] = '\0';
      }
    } else {
      if (strcmp(serialBuffer, "sa") == 0) {
        rxPacket.programRunning = true;
        Serial.println("\n【指令生效】程序已恢复运行（本地强制）");
      } else if (strcmp(serialBuffer, "sp") == 0) {
        rxPacket.programRunning = false;
        Serial.println("\n【指令生效】程序已暂停（本地强制）");
      } else {
        Serial.print("\n【指令错误】未知指令：");
        Serial.print(serialBuffer);
        Serial.println("，请输入 'sa' 或 'sp'");
      }
      memset(serialBuffer, 0, SERIAL_BUFFER_SIZE);
      bufferIndex = 0;
    }
  }
}

int applySmoothing(int target, int last, float factor) {
  return (int)(target * factor + last * (1 - factor));
}

// -------------------------- BLE Server回调 --------------------------
class MyServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer* pserver) {
    bleConnected = true;
    Serial.println("[BLE] 已连接采集机");
  }
  void onDisconnect(BLEServer* pserver) {
    bleConnected = false;
    Serial.println("[BLE] 与采集机断开连接，重新广播...");
    pserver->startAdvertising();
  }
};

class MyCharCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pchar) {
    // 修复：使用正确的类型获取值
    String rxStr = pchar->getValue();    
    if (rxValue.length() == sizeof(DataPacket)) {
      memcpy(&rxPacket, rxValue.data(), sizeof(DataPacket));
    }
  }
};

// -------------------------- BLE Client回调 --------------------------
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    bleConnected = true;
    Serial.println("[BLE] 已连接采集机");
  }
  void onDisconnect(BLEClient* pclient) {
    bleConnected = false;
    pDataChar = nullptr;
    Serial.println("[BLE] 与采集机断开连接，重新尝试连接...");
    bleRole = 0;
  }
};

// -------------------------- BLE角色自动识别 --------------------------
void autoBleRoleInit() {
  BLEDevice::init("ESP32S3_Master");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(50);

  // 修复：正确获取扫描结果（非指针类型）
  Serial.println("[BLE] 扫描采集机...");
  BLEScanResults scanResults = pBLEScan->start(3);  // 修复类型转换错误
  pBLEScan->stop();

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice dev = foundDevices.getDevice(i);
    // 修复：使用Arduino String的indexOf方法替代find
    if (dev.getName().indexOf("ESP32S3_Collector") != -1) {
      BLEClient* pBLEClient = BLEDevice::createClient();
      pBLEClient->setClientCallbacks(new MyClientCallback());
      if (pBLEClient->connect(&dev)) {
        // 修复：使用正确的远程服务类型
        BLERemoteService* pService = pBLEClient->getService(SERVICE_UUID);
        if (pService) {
          BLERemoteCharacteristic* pRemoteChar = pService->getCharacteristic(DATA_CHAR_UUID);
          if (pRemoteChar) {
            pRemoteChar->registerForNotify([](BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
              if (length == sizeof(DataPacket)) {
                memcpy(&rxPacket, pData, sizeof(DataPacket));
              }
            });
            bleRole = 2;
            Serial.println("[BLE] 找到采集机，作为Client连接成功");
            return;
          }
        }
        pBLEClient->disconnect();
      }
    }
  }

  // 未找到采集机，作为Server广播
  Serial.println("[BLE] 未找到采集机，作为Server广播");
  pBLEServer = BLEDevice::createServer();
  pBLEServer->setCallbacks(new MyServerCallback());
  BLEService* pService = pBLEServer->createService(SERVICE_UUID);
  pDataChar = pService->createCharacteristic(
    DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_NOTIFY
  );
  pDataChar->addDescriptor(new BLE2902());
  pDataChar->setCallbacks(new MyCharCallback());
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  bleRole = 1;
}

// -------------------------- 初始化 --------------------------
void setup() {
  FastLED.addLeds<LED_TYPE, LED_STRIP_PIN, COLOR_ORDER>(ledStrip, LED_COUNT)
         .setCorrection(TypicalLEDStrip);
  FastLED.setMaxRefreshRate(60);
  FastLED.setBrightness(lastPwmValue);
  fill_solid(ledStrip, LED_COUNT, getDynamicColor(lastPwmValue));
  FastLED.show();

  Serial.begin(115200);
  Serial.println("===================================");
  Serial.println("ESP32S3 主控机（灯带控制版）");
  Serial.println("功能：接收亮度值、控制灯带、响应启停指令");
  Serial.println("初始状态：暂停（等待采集机指令）");
  Serial.println("===================================\n");

  rxPacket.programRunning = false;
  rxPacket.brightness = MIN_BRIGHTNESS;
  Serial.println("系统启动，当前状态：暂停");

  autoBleRoleInit();
}

// -------------------------- 主循环 --------------------------
void loop() {
  listenSerialCommand();
  unsigned long currentTime = millis();

  if (rxPacket.programRunning) {
    int targetBrightness = constrain(rxPacket.brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    int smoothedBrightness = (targetBrightness > lastPwmValue) ?
      applySmoothing(targetBrightness, lastPwmValue, SMOOTH_UP_FACTOR) :
      applySmoothing(targetBrightness, lastPwmValue, SMOOTH_DOWN_FACTOR);
    smoothedBrightness = constrain(smoothedBrightness, BUFFER_LOW, BUFFER_HIGH);

    if (abs(smoothedBrightness - lastPwmValue) >= BRIGHTNESS_THRESHOLD) {
      lastPwmValue = smoothedBrightness;
      int outputBrightness = constrain(lastPwmValue, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
      CRGB dynamicColor = getDynamicColor(outputBrightness);
      FastLED.setBrightness(outputBrightness);
      fill_solid(ledStrip, LED_COUNT, dynamicColor);
      FastLED.show();
    }
  } else {
    if (lastPwmValue != MIN_BRIGHTNESS) {
      lastPwmValue = MIN_BRIGHTNESS;
      FastLED.setBrightness(lastPwmValue);
      fill_solid(ledStrip, LED_COUNT, getDynamicColor(lastPwmValue));
      FastLED.show();
    }
  }

  static unsigned long lastSerialPrintTime = 0;
  if (currentTime - lastSerialPrintTime >= SERIAL_PRINT_INTERVAL) {
    int outputBrightness = constrain(lastPwmValue, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    CRGB currentColor = getDynamicColor(outputBrightness);

    Serial.print("AD值："); Serial.print("N/A（采集机）");
    Serial.print(" | 电压："); Serial.print("N/A（采集机）");
    Serial.print(" | Rs："); Serial.print("N/A（采集机）");
    Serial.print(" | 电导："); Serial.print("N/A（采集机）");
    Serial.print(" | 力："); Serial.print("N/A（采集机）");
    Serial.print(" | 目标亮度："); Serial.print(rxPacket.brightness);
    Serial.print(" | 实际亮度："); Serial.print(outputBrightness);
    Serial.print(" | RGB：("); Serial.print(currentColor.r);
    Serial.print(","); Serial.print(currentColor.g);
    Serial.print(","); Serial.print(currentColor.b); Serial.print(")");
    Serial.print(" | 运行状态："); Serial.print(rxPacket.programRunning ? "运行" : "暂停");
    Serial.print(" | BLE连接："); Serial.println(bleConnected ? "是" : "否");

    lastSerialPrintTime = currentTime;
  }

  if (!bleConnected && bleRole == 2) {
    delay(1000);
    autoBleRoleInit();
  }

  delay(50);
}
