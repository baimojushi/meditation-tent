// 引入FastLED库（需先在Arduino IDE中安装）
#include <FastLED.h>
#include "esp_wifi.h"
#include "esp_bt.h"

// 亮度与颜色参数（提前定义，确保在使用前声明）
const int MIN_BRIGHTNESS = 0;     // 最终输出最小亮度
const int MAX_BRIGHTNESS = 255;   // 最终输出最大亮度
const int BUFFER_LOW = -10;       // 缓冲范围下限（仅灯带用）
const int BUFFER_HIGH = 265;      // 缓冲范围上限（仅灯带用）
const CRGB BASE_WARM_WHITE = CRGB(255, 160, 40); // 基础暖白光RGB值（打印用）
const int DIM_THRESHOLD = 40;     // 低亮度阈值（打印用）
const int BRIGHT_THRESHOLD = 215; // 高亮度阈值（打印用）
const float SMOOTH_UP_FACTOR = 0.12f;   // 上升平滑因子（按钮LED与灯带共用）
const float SMOOTH_DOWN_FACTOR = 0.12f; // 下降平滑因子（按钮LED与灯带共用）
const int BRIGHTNESS_THRESHOLD = 2;    // 亮度变化阈值（按钮LED与灯带共用）
const float TRIM_RATIO = 0.25f;   // 电压极值掐头去尾比例

// 引脚定义（适配ESP32，使用支持RMT功能的引脚）
const int POT_PIN = 4;         // 压力传感器连接到GPIO4（ADC2，原POT_PIN复用）
const int LED_STRIP_PIN = 2;   // 灯带信号控制引脚（GPIO2，支持RMT功能）
const int BUTTON_PIN = 16;     // 按钮连接到GPIO16（与3.3V之间接常开自复位按钮）

// 按钮LED配置（移除呼吸灯相关，保留PWM基础参数）
const int BUTTON_LED_PIN = 17;  // 控制按钮LED的GPIO引脚
const int LEDC_FREQ = 500;      // PWM频率（500Hz）
const int LEDC_RESOLUTION = 8;  // PWM分辨率（8位，0~255亮度）
int ledcChannel;                // 存储ledcAttach()自动分配的通道号
// 按钮LED平滑化变量（与灯带平滑逻辑一致）
int lastButtonLedBrightness = MIN_BRIGHTNESS;  // 现在MIN_BRIGHTNESS已提前声明

// 灯带参数配置
#define LED_COUNT 97            // 灯珠数量
#define LED_TYPE WS2812B        // 灯带类型
#define COLOR_ORDER GRB         // 颜色顺序
CRGB ledStrip[LED_COUNT];       // 灯带像素数组

// 电压测量参数
const float ADC_REF_VOLTAGE = 3.3f; // 校准后的参考电压（3.3V）
const int ADC_RESOLUTION = 4095;   // ESP32 ADC分辨率（12位，0-4095）
const int ADC_SAMPLE_TIMES = 10;   // ADC多次采样次数（抗噪声）

// 压力传感器关键参数
const float R0 = 150000.0f;    // 分压固定电阻（150KΩ）
const int MIN_FORCE = 2;       // 传感器最小有效力（2g）
const int MAX_FORCE = 1500;    // 传感器最大有效力（1500g）
const float MIN_CONDUCTANCE = 0.1f;  // 对应MIN_FORCE的电导（μS）
const float MAX_CONDUCTANCE = 0.6f;  // 对应MAX_FORCE的电导（μS）

// 时间与数据缓冲区参数（按钮LED与灯带共用缓冲区）
const unsigned long UPDATE_INTERVAL = 5000;  // 极值更新间隔（5秒）
const int SAMPLE_INTERVAL = 200;             // 电压采样间隔（200ms）
const int BUFFER_SIZE = 300;                 // 数据缓冲区大小
const unsigned long SERIAL_PRINT_INTERVAL = 500; // 串口实时数据打印间隔

// 串口指令配置
const int SERIAL_BUFFER_SIZE = 10;  // 串口接收缓冲区大小
char serialBuffer[SERIAL_BUFFER_SIZE] = {0}; 
int bufferIndex = 0;                // 缓冲区索引

int voltageBuffer[BUFFER_SIZE] = {0};
int voltageBufferIndex = 0;        // 电压缓冲区索引

// 极值与时间变量（按钮LED与灯带共用）
int rawMaxVoltage = 0;
int rawMinVoltage = 4095;
int maxVoltage = 0;
int minVoltage = 4095;
unsigned long lastUpdateTime = 0;
int lastUpdateBufferIndex = 0;     // 记录上一次更新后的缓冲区索引

// 平滑化与状态变量（仅灯带用）
int lastPwmValue = MIN_BRIGHTNESS;  // 灯带平滑化变量
bool programRunning = false;        // 程序运行状态，初始为暂停

// 按钮状态变量
int buttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;  // 按钮防抖动延迟

// 根据亮度动态调整RGB值的函数（保留原功能，用于串口打印）
CRGB getDynamicColor(int brightness) {
  int outputBrightness = constrain(brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
  
  if (outputBrightness < DIM_THRESHOLD) {
    float dimFactor = (float)outputBrightness / DIM_THRESHOLD;
    return CRGB(
      constrain(BASE_WARM_WHITE.r * dimFactor, 0, 255),
      constrain(BASE_WARM_WHITE.g * dimFactor, 0, 255),
      constrain(BASE_WARM_WHITE.b * dimFactor, 0, 255)
    );
  } else if (outputBrightness > BRIGHT_THRESHOLD) {
    float brightFactor = 1.0f + (float)(outputBrightness - BRIGHT_THRESHOLD) / (MAX_BRIGHTNESS - BRIGHT_THRESHOLD) * 0.3f;
    return CRGB(
      constrain(BASE_WARM_WHITE.r * brightFactor, 0, 255),
      constrain(BASE_WARM_WHITE.g * brightFactor, 0, 255),
      constrain(BASE_WARM_WHITE.b * brightFactor, 0, 255)
    );
  } else {
    return BASE_WARM_WHITE;
  }
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);  // 初始化按钮引脚（内部下拉电阻）
  pinMode(BUTTON_LED_PIN, OUTPUT);      // 按钮LED引脚为输出模式
  ledcChannel = ledcAttach(BUTTON_LED_PIN, LEDC_FREQ, LEDC_RESOLUTION);
  // 初始化按钮LED亮度（平滑化初始值）
  ledcWrite(BUTTON_LED_PIN, lastButtonLedBrightness);

  // 初始化FastLED灯带（保留原配置）
  FastLED.addLeds<LED_TYPE, LED_STRIP_PIN, COLOR_ORDER>(ledStrip, LED_COUNT)
         .setCorrection(TypicalLEDStrip);
  FastLED.setMaxRefreshRate(60);
  FastLED.setBrightness(lastPwmValue);
  fill_solid(ledStrip, LED_COUNT, getDynamicColor(lastPwmValue));
  FastLED.show();

  // 配置ADC衰减（保留原配置）
  analogSetPinAttenuation(POT_PIN, ADC_11db); 
  analogSetAttenuation(ADC_11db);
  
  // 禁用WiFi和蓝牙（保留原功能）
  esp_wifi_stop();
  esp_bt_controller_disable();
  
  // 初始化串口（保留原提示，移除暂停提示相关）
  Serial.begin(115200);
  Serial.println("===================================");
  Serial.println("ESP32 灯带控制（压力传感器版）");
  Serial.println("传感器：RP-C7.6ST-LF2（1/R与力线性）");
  Serial.println("按钮功能：按下切换程序运行/暂停状态");
  Serial.println("初始状态：暂停（静默读取）");
  Serial.println("===================================\n");
  
  // 初始化电压缓冲区（保留原功能）
  int initialValue = getSmoothedAdcValue();
  Serial.print("初始化AD值: "); Serial.print(initialValue);
  Serial.print(" | 初始电压: "); Serial.print((float)initialValue / ADC_RESOLUTION * ADC_REF_VOLTAGE, 3);
  Serial.println(" V");
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    voltageBuffer[i] = initialValue;
  }
  
  lastUpdateBufferIndex = voltageBufferIndex;
  
  Serial.println("系统启动，当前状态：暂停");
}

void loop() {
  handleButtonInput();    // 检测按钮状态
  listenSerialCommand();  // 保留串口指令控制
  unsigned long currentTime = millis();
  
  if (programRunning) {
    // 运行时：按钮LED常亮，灯带随传感器变化（原逻辑不变）
    ledcWrite(BUTTON_LED_PIN, 255); 

    // 定时采样电压（保留原功能）
    static unsigned long lastSampleTime = 0;
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
      int currentVoltage = getSmoothedAdcValue();
      voltageBuffer[voltageBufferIndex] = currentVoltage;
      voltageBufferIndex = (voltageBufferIndex + 1) % BUFFER_SIZE;
      lastSampleTime = currentTime;
    }
    
    // 每5秒更新一次电压极值（保留原功能）
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      rawMaxVoltage = -1;
      rawMinVoltage = ADC_RESOLUTION + 1;
      int trimAmount = 1;
      
      // 遍历上一次更新后新增的数据
      if (lastUpdateBufferIndex <= voltageBufferIndex) {
        for (int i = lastUpdateBufferIndex; i < voltageBufferIndex; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
      } else {
        for (int i = lastUpdateBufferIndex; i < BUFFER_SIZE; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
        for (int i = 0; i < voltageBufferIndex; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
      }
      
      // 异常处理：若未采集到新数据，沿用上次极值
      if (rawMaxVoltage == -1 || rawMinVoltage == ADC_RESOLUTION + 1) {
        rawMaxVoltage = maxVoltage + trimAmount;
        rawMinVoltage = minVoltage - trimAmount;
      }
      
      // 计算修剪量
      int voltageRange = rawMaxVoltage - rawMinVoltage;
      if (voltageRange <= 0) voltageRange = 200;
      trimAmount = (int)(voltageRange * TRIM_RATIO);
      if (trimAmount < 1) trimAmount = 1;
      
      maxVoltage = rawMaxVoltage - trimAmount;
      minVoltage = rawMinVoltage + trimAmount;
      if (maxVoltage <= minVoltage) {
        minVoltage = rawMinVoltage;
        maxVoltage = minVoltage + 200;
      }
      
      // 打印极值信息（保留原功能）
      float rawMaxVolt = (float)rawMaxVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
      float rawMinVolt = (float)rawMinVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
      float maxVolt = (float)maxVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
      float minVolt = (float)minVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
      
      Serial.println("\n-----------------------------------");
      Serial.print("原始极值：最大 = "); Serial.print(rawMaxVoltage);
      Serial.print(" ("); Serial.print(rawMaxVolt, 3); Serial.print("V)，");
      Serial.print("最小 = "); Serial.print(rawMinVoltage);
      Serial.print(" ("); Serial.print(rawMinVolt, 3); Serial.println("V)");
      Serial.print("有效极值：最大 = "); Serial.print(maxVoltage);
      Serial.print(" ("); Serial.print(maxVolt, 3); Serial.print("V)，");
      Serial.print("最小 = "); Serial.print(minVoltage);
      Serial.print(" ("); Serial.print(minVolt, 3); Serial.println("V)");
      Serial.println("-----------------------------------");
      
      // 更新缓冲区索引记录
      lastUpdateBufferIndex = voltageBufferIndex;
      lastUpdateTime = currentTime;
    }
    
    if (maxVoltage <= minVoltage) {
      maxVoltage = minVoltage + 1;
    }
    
    // 动态极值映射亮度（灯带控制，原逻辑不变）
    int currentVoltage = getSmoothedAdcValue();
    float actualVoltage = (float)currentVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
    
    // 传感器电阻Rs和电导计算（用于串口打印）
    float Rs = 1e8f;  
    if (actualVoltage > 0.01f) {  
      Rs = R0 * (ADC_REF_VOLTAGE - actualVoltage) / actualVoltage;
    }
    float conductance = (1.0f / Rs) * 1e6f;  
    conductance = constrain(conductance, MIN_CONDUCTANCE, MAX_CONDUCTANCE);
    
    // 力计算（用于串口打印）
    int force = map((int)(conductance * 100), 
                   (int)(MIN_CONDUCTANCE * 100), 
                   (int)(MAX_CONDUCTANCE * 100), 
                   MIN_FORCE, 
                   MAX_FORCE);
    
    // AD值→缓冲区动态极值→亮度映射（灯带）
    int targetBrightness = map(currentVoltage, 
                               minVoltage, 
                               maxVoltage, 
                               MIN_BRIGHTNESS, 
                               MAX_BRIGHTNESS);
    targetBrightness = constrain(targetBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    
    // 平滑化处理（灯带）
    int smoothedBrightness;
    if (targetBrightness > lastPwmValue) {
      smoothedBrightness = applySmoothing(targetBrightness, lastPwmValue, SMOOTH_UP_FACTOR);
    } else {
      smoothedBrightness = applySmoothing(targetBrightness, lastPwmValue, SMOOTH_DOWN_FACTOR);
    }
    smoothedBrightness = constrain(smoothedBrightness, BUFFER_LOW, BUFFER_HIGH);
    
    // 亮度更新（灯带）
    if (abs(smoothedBrightness - lastPwmValue) >= BRIGHTNESS_THRESHOLD) {
      lastPwmValue = smoothedBrightness;
      int outputBrightness = constrain(lastPwmValue, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
      CRGB dynamicColor = getDynamicColor(outputBrightness);
      FastLED.setBrightness(outputBrightness);
      fill_solid(ledStrip, LED_COUNT, dynamicColor);
      FastLED.show();
    }
    
    // 串口打印（保留原逻辑，无改动）
    static unsigned long lastSerialPrintTime = 0;
    if (currentTime - lastSerialPrintTime >= SERIAL_PRINT_INTERVAL) {
      int outputBrightness = constrain(lastPwmValue, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
      CRGB currentColor = getDynamicColor(outputBrightness);
      
      Serial.print("AD值："); Serial.print(currentVoltage);
      Serial.print(" | 电压："); Serial.print(actualVoltage, 3); Serial.print("V");
      Serial.print(" | Rs："); Serial.print(Rs / 1000, 1); Serial.print("KΩ");
      Serial.print(" | 电导："); Serial.print(conductance, 2); Serial.print("μS");
      Serial.print(" | 力："); Serial.print(force); Serial.print("g");
      Serial.print(" | 目标亮度："); Serial.print(targetBrightness);
      Serial.print(" | 实际亮度："); Serial.print(outputBrightness);
      Serial.print(" | RGB：("); Serial.print(currentColor.r);
      Serial.print(","); Serial.print(currentColor.g);
      Serial.print(","); Serial.print(currentColor.b); Serial.println(")");
      
      lastSerialPrintTime = currentTime;
    }

  } else {
    // 暂停时：静默读取传感器+按钮LED随传感器变化（与灯带映射逻辑完全一致）
    // 1. 定时采样电压（与运行时共用缓冲区）
    static unsigned long lastSampleTime = 0;
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
      int currentVoltage = getSmoothedAdcValue();
      voltageBuffer[voltageBufferIndex] = currentVoltage;
      voltageBufferIndex = (voltageBufferIndex + 1) % BUFFER_SIZE;
      lastSampleTime = currentTime;
    }
    
    // 2. 定时更新电压极值（与运行时共用极值变量）
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      rawMaxVoltage = -1;
      rawMinVoltage = ADC_RESOLUTION + 1;
      int trimAmount = 1;
      
      // 遍历上一次更新后新增的数据
      if (lastUpdateBufferIndex <= voltageBufferIndex) {
        for (int i = lastUpdateBufferIndex; i < voltageBufferIndex; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
      } else {
        for (int i = lastUpdateBufferIndex; i < BUFFER_SIZE; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
        for (int i = 0; i < voltageBufferIndex; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
      }
      
      // 异常处理：未采集到新数据时沿用上次极值
      if (rawMaxVoltage == -1 || rawMinVoltage == ADC_RESOLUTION + 1) {
        rawMaxVoltage = maxVoltage + trimAmount;
        rawMinVoltage = minVoltage - trimAmount;
      }
      
      // 计算修剪量与有效极值
      int voltageRange = rawMaxVoltage - rawMinVoltage;
      if (voltageRange <= 0) voltageRange = 200;
      trimAmount = (int)(voltageRange * TRIM_RATIO);
      if (trimAmount < 1) trimAmount = 1;
      
      maxVoltage = rawMaxVoltage - trimAmount;
      minVoltage = rawMinVoltage + trimAmount;
      if (maxVoltage <= minVoltage) {
        minVoltage = rawMinVoltage;
        maxVoltage = minVoltage + 200;
      }
      
      // 更新缓冲区索引（与运行时共用）
      lastUpdateBufferIndex = voltageBufferIndex;
      lastUpdateTime = currentTime;
    }
    
    if (maxVoltage <= minVoltage) {
      maxVoltage = minVoltage + 1;
    }
    
    // 3. 传感器运算与按钮LED亮度映射（与灯带逻辑完全一致）
    int currentVoltage = getSmoothedAdcValue();
    float actualVoltage = (float)currentVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
    
    // 计算电导、力（用于串口打印，保持打印逻辑不变）
    float Rs = 1e8f;
    if (actualVoltage > 0.01f) {
      Rs = R0 * (ADC_REF_VOLTAGE - actualVoltage) / actualVoltage;
    }
    float conductance = (1.0f / Rs) * 1e6f;
    conductance = constrain(conductance, MIN_CONDUCTANCE, MAX_CONDUCTANCE);
    
    int force = map((int)(conductance * 100),
                   (int)(MIN_CONDUCTANCE * 100),
                   (int)(MAX_CONDUCTANCE * 100),
                   MIN_FORCE,
                   MAX_FORCE);
    
    // 目标亮度计算（与灯带相同：AD值→动态极值映射）
    int targetBrightness = map(currentVoltage,
                               minVoltage,
                               maxVoltage,
                               MIN_BRIGHTNESS,
                               MAX_BRIGHTNESS);
    targetBrightness = constrain(targetBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    
    // 4. 按钮LED亮度平滑化（与灯带平滑逻辑一致）
    int smoothedBrightness;
    if (targetBrightness > lastButtonLedBrightness) {
      smoothedBrightness = applySmoothing(targetBrightness, lastButtonLedBrightness, SMOOTH_UP_FACTOR);
    } else {
      smoothedBrightness = applySmoothing(targetBrightness, lastButtonLedBrightness, SMOOTH_DOWN_FACTOR);
    }
    smoothedBrightness = constrain(smoothedBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    
    // 5. 更新按钮LED亮度（超过阈值时生效，避免突变）
    if (abs(smoothedBrightness - lastButtonLedBrightness) >= BRIGHTNESS_THRESHOLD) {
      lastButtonLedBrightness = smoothedBrightness;
      ledcWrite(BUTTON_LED_PIN, lastButtonLedBrightness);
    }
    
    // 6. 保留串口打印（与运行时格式完全一致，无改动）
    static unsigned long lastSerialPrintTime = 0;
    if (currentTime - lastSerialPrintTime >= SERIAL_PRINT_INTERVAL) {
      int outputBrightness = constrain(lastButtonLedBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
      CRGB currentColor = getDynamicColor(outputBrightness);
      
      Serial.print("AD值："); Serial.print(currentVoltage);
      Serial.print(" | 电压："); Serial.print(actualVoltage, 3); Serial.print("V");
      Serial.print(" | Rs："); Serial.print(Rs / 1000, 1); Serial.print("KΩ");
      Serial.print(" | 电导："); Serial.print(conductance, 2); Serial.print("μS");
      Serial.print(" | 力："); Serial.print(force); Serial.print("g");
      Serial.print(" | 目标亮度："); Serial.print(targetBrightness);
      Serial.print(" | 实际亮度："); Serial.print(outputBrightness);
      Serial.print(" | RGB：("); Serial.print(currentColor.r);
      Serial.print(","); Serial.print(currentColor.g);
      Serial.print(","); Serial.print(currentColor.b); Serial.println(")");
      
      lastSerialPrintTime = currentTime;
    }
  }
  
  delay(50);
}

// 多次采样并取平均值（保留原功能）
int getSmoothedAdcValue() {
  int sum = 0;
  for (int i = 0; i < ADC_SAMPLE_TIMES; i++) {
    sum += analogRead(POT_PIN);
    delayMicroseconds(100);
  }
  return sum / ADC_SAMPLE_TIMES;
}

// 串口指令监听（保留原功能）
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
        if (!programRunning) {
          programRunning = true;
          Serial.println("\n【指令生效】程序已恢复运行");
        } else {
          Serial.println("\n【提示】程序当前已在运行中");
        }
      } else if (strcmp(serialBuffer, "sp") == 0) {
        if (programRunning) {
          programRunning = false;
          Serial.println("\n【指令生效】程序已暂停");
        } else {
          Serial.println("\n【提示】程序当前已暂停");
        }
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

// 平滑化函数（保留原功能，按钮LED与灯带共用）
int applySmoothing(int target, int last, float factor) {
  return (int)(target * factor + last * (1 - factor));
}

// 按钮输入处理（保留原功能，仅切换运行状态）
void handleButtonInput() {
  int currentReading = digitalRead(BUTTON_PIN);
  
  if (currentReading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (currentReading != buttonState) {
      buttonState = currentReading;
      if (buttonState == HIGH) {
        programRunning = !programRunning;
        if (programRunning) {
          Serial.println("\n【按钮控制】程序已启动");
          int initialValue = getSmoothedAdcValue();
          for (int i = 0; i < BUFFER_SIZE; i++) {
            voltageBuffer[i] = initialValue;
          }
          lastUpdateTime = millis();
          lastUpdateBufferIndex = voltageBufferIndex;
        } else {
          Serial.println("\n【按钮控制】程序已暂停");
        }
      }
    }
  }
  
  lastButtonState = currentReading;
}
