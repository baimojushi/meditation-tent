/* ===========================================================
   ESP32 灯带控制（压力传感器版）
   目的：保持原代码结构与逻辑不变，将功能拆分为：
     - A机（ESP32-S3）职责：
         * ADC 采集（多次采样与去抖/平均）
         * 滤波 / 极值更新 / 亮度映射（AD -> 动态极值 -> 映射到 0-255）
         * 按钮读取与启停控制（处理防抖、切换 programRunning 状态）
         * 串口打印（调试信息，包含 AD、电压、电导、力、目标/实际亮度、RGB）
         * (新增）通过低功耗蓝牙发送：实时亮度值与按钮启停信号（作为主机）
     - B机（ESP32-C3）职责：
         * 接收 A 机发送的亮度值与启停信号
         * 控制灯带显示（FastLED）并做相应平滑处理
         * 本机也保持串口打印（便于两端对比调试）
         * （新增）通过低功耗蓝牙接收/反馈心跳或确认
     设计说明：
       1. 本代码保留了灯带与采集在同一份文件的实现，便于理解映射逻辑。
       2. 拆分步骤建议：把与 ADC、极值、按钮、串口打印相关的函数/变量移到 A 机固件；
          把 FastLED、LED 输出、lastPwmValue、lastButtonLedBrightness、PWM 控制相关的变量移到 B 机固件。
       3. 蓝牙连接策略（实现建议）：
          - 开机后自动以固定 ID 搜索对端（若找到则进入点对点通信），否则进入广播模式等待配对。
          - 发送最小数据包：{亮度: uint8, 启停: bool}；可选加心跳/ACK。
       4. 注意保持数据帧最小化以降低蓝牙功耗；采样频率与 UPDATE_INTERVAL 保持一致或更低。

   注：下面代码为原始实现。
   =========================================================== */

 // 引入第三方库
#include <FastLED.h>
#include "esp_wifi.h"
#include "esp_bt.h"

// ---------------------------
// 常量：亮度 / 颜色 基础参数
// 说明：这些常量用于整个映射/输出流程，A机与B机都需了解（或复制到两端）
// ---------------------------
const int MIN_BRIGHTNESS = 0;     // 最终输出最小亮度（范围底端）
const int MAX_BRIGHTNESS = 255;   // 最终输出最大亮度（范围顶端）
const int BUFFER_LOW = -10;       // 灯带平滑/容错下限（仅用于内部限制）
const int BUFFER_HIGH = 265;      // 灯带平滑/容错上限（仅用于内部限制）
const CRGB BASE_WARM_WHITE = CRGB(255, 160, 40); // 基础暖白色（用于打印与视觉参考）
const int DIM_THRESHOLD = 40;     // 低亮度阈值（用于颜色渐变分支）
const int BRIGHT_THRESHOLD = 215; // 高亮度阈值（用于颜色亮化分支）
const float SMOOTH_UP_FACTOR = 0.12f;   // 亮度上升平滑系数（用于平滑算法）
const float SMOOTH_DOWN_FACTOR = 0.12f; // 亮度下降平滑系数（用于平滑算法）
const int BRIGHTNESS_THRESHOLD = 2;    // 最小亮度变化阈值（避免频繁更新）
const float TRIM_RATIO = 0.25f;   // 极值修剪比例（用于去除极端采样值）

// ---------------------------
// 引脚定义（针对 ESP32 硬件）
// 说明：移植或拆分时注意 S3 / C3 GPIO 映射与 ADC 支持差异
// ---------------------------
const int POT_PIN = 4;         // 压力传感器连接引脚（ADC 输入）
const int LED_STRIP_PIN = 2;   // 灯带控制引脚（RMT/输出）
const int BUTTON_PIN = 16;     // 物理按钮输入（使用下拉）

// ---------------------------
// 按钮LED（本实现用 PWM 控制）
// 说明：保持 PWM 基础参数；ledcChannel 在运行时由 ledcAttach 分配
// ---------------------------
const int BUTTON_LED_PIN = 17;  // 控制按钮上小灯的 GPIO
const int LEDC_FREQ = 500;      // PWM 频率（Hz）
const int LEDC_RESOLUTION = 8;  // PWM 分辨率（位）
int ledcChannel;                // 存放 ledcAttach 返回的通道号
int lastButtonLedBrightness = MIN_BRIGHTNESS;  // 按钮LED上一次记录亮度（用于平滑）

// ---------------------------
// 灯带参数（FastLED）
// 说明：灯珠数量与类型固定；拆分后 B 机将承担 FastLED 控制
// ---------------------------
#define LED_COUNT 97            // 灯珠数量
#define LED_TYPE WS2812B        // 灯带型号
#define COLOR_ORDER GRB         // 颜色顺序
CRGB ledStrip[LED_COUNT];       // 灯带像素缓冲

// ---------------------------
// ADC 与采样参数
// 说明：ADC 分辨率与采样次数影响平滑与极值统计（属于 A 机核心）
// ---------------------------
const float ADC_REF_VOLTAGE = 3.3f; // 参考电压
const int ADC_RESOLUTION = 4095;   // ADC 最大值（12位）
const int ADC_SAMPLE_TIMES = 10;   // 每次读取平均采样次数（抗噪）

// ---------------------------
// 传感器参数（压力传感器特性）
// 说明：用于将电压/电导映射到力值，仅用于串口打印/调试
// ---------------------------
const float R0 = 150000.0f;    // 分压固定电阻（单位：欧姆）
const int MIN_FORCE = 2;       // 传感器最小有效力（g）
const int MAX_FORCE = 1500;    // 传感器最大有效力（g）
const float MIN_CONDUCTANCE = 0.1f;  // 对应最小力的电导（μS）
const float MAX_CONDUCTANCE = 0.6f;  // 对应最大力的电导（μS）

// ---------------------------
// 时间与缓冲区配置（通用）
// 说明：UPDATE_INTERVAL 控制极值更新频率；SAMPLE_INTERVAL 控制采样节奏
// ---------------------------
const unsigned long UPDATE_INTERVAL = 5000;  // 极值重算周期（毫秒）
const int SAMPLE_INTERVAL = 200;             // 单次采样间隔（毫秒）
const int BUFFER_SIZE = 300;                 // 环形缓冲区长度
const unsigned long SERIAL_PRINT_INTERVAL = 500; // 串口打印间隔（毫秒）

// ---------------------------
// 串口接收缓冲（命令行交互）
// 说明：保留原串口命令接口 ('sa'/'sp') 以便远程手动控制
// ---------------------------
const int SERIAL_BUFFER_SIZE = 10;  // 串口输入缓冲区长度
char serialBuffer[SERIAL_BUFFER_SIZE] = {0}; 
int bufferIndex = 0;                // 当前缓冲索引

// ---------------------------
// 数据缓冲区：电压环形数组
// 说明：保存 recent ADC 读数，用于极值计算（A机维护）
// ---------------------------
int voltageBuffer[BUFFER_SIZE] = {0};
int voltageBufferIndex = 0;        // 写指针（环形）

// ---------------------------
// 极值与更新时间记录（A机维护）
// 说明：rawMax/rawMin 用于临时统计，maxVoltage/minVoltage 为有效极值
// ---------------------------
int rawMaxVoltage = 0;
int rawMinVoltage = 4095;
int maxVoltage = 0;
int minVoltage = 4095;
unsigned long lastUpdateTime = 0;
int lastUpdateBufferIndex = 0;     // 上一次极值更新后环形缓冲指针位置

// ---------------------------
// 平滑与程序运行状态（注意：lastPwmValue 与 programRunning 关联到灯带）
// 说明：拆分时 lastPwmValue 与灯带实际输出应保留在 B 机；但当前文件仍共存
// ---------------------------
int lastPwmValue = MIN_BRIGHTNESS;  // 灯带平滑记录
bool programRunning = false;        // 主逻辑运行/暂停标志（由按钮或串口控制）

// ---------------------------
// 按钮（去抖）变量（A机维护）
// ---------------------------
int buttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;  // 去抖延时（毫秒）

// ---------------------------
// 根据亮度计算颜色（仅用于串口打印显示视觉参考）
// 说明：保持原实现，用于将亮度映射为近似暖白色的色彩输出
// 返回：CRGB（可直接用于 FastLED）
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
  // 按钮与按键 LED 初始化（GPIO 模式 & PWM）：
  // - BUTTON_PIN 使用内部下拉（便于低电平空闲）
  // - BUTTON_LED_PIN 由 ledcAttach 控制 PWM，ledcChannel 存储分配值
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);  // 初始化按钮引脚（内部下拉电阻）
  pinMode(BUTTON_LED_PIN, OUTPUT);      // 按钮LED引脚为输出模式
  ledcChannel = ledcAttach(BUTTON_LED_PIN, LEDC_FREQ, LEDC_RESOLUTION);
  // 初始化按钮LED亮度（使用 lastButtonLedBrightness 作为初始值）
  ledcWrite(BUTTON_LED_PIN, lastButtonLedBrightness);

  // FastLED 初始化（保留原配置，拆分到 B 机时在 B 机固件中重复此配置）
  FastLED.addLeds<LED_TYPE, LED_STRIP_PIN, COLOR_ORDER>(ledStrip, LED_COUNT)
         .setCorrection(TypicalLEDStrip);
  FastLED.setMaxRefreshRate(60);
  FastLED.setBrightness(lastPwmValue);
  fill_solid(ledStrip, LED_COUNT, getDynamicColor(lastPwmValue));
  FastLED.show();

  // ADC 衰减配置（保留原设置，针对 ESP32 ADC）
  analogSetPinAttenuation(POT_PIN, ADC_11db); 
  analogSetAttenuation(ADC_11db);
  
  // 禁用 WiFi 与 Bluetooth
  // 当前实现关闭以节约资源；拆分实现蓝牙通信时请按平台文档重新初始化蓝牙
  esp_wifi_stop();
  esp_bt_controller_disable();
  
  // 串口打印初始化（保留，用于调试）
  Serial.begin(115200);
  Serial.println("===================================");
  Serial.println("ESP32 灯带控制（压力传感器版）");
  Serial.println("传感器：RP-C7.6ST-LF2（1/R与力线性）");
  Serial.println("按钮功能：按下切换程序运行/暂停状态");
  Serial.println("初始状态：暂停（静默读取）");
  Serial.println("===================================\n");
  
  // 电压缓冲区预填充：使用一次平滑读数填充整个缓冲区以避免极值计算异常
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

// ---------------------------
// 主循环：入口（loop）
// 说明：主循环按 programRunning 分两套行为：运行/暂停；A 机侧处理数据采样与映射，B 机侧负责灯带输出
//       目前实现两端在同一文件中，拆分后请把对应逻辑移动到各自设备
// ---------------------------
void loop() {
  handleButtonInput();    // 处理按钮（去抖并切换 programRunning）
  listenSerialCommand();  // 监听串口命令（'sa' / 'sp'）
  unsigned long currentTime = millis();
  
  if (programRunning) {
    // ========== 运行状态 ==========
    // 说明：运行时按钮 LED 常亮，灯带随传感器实时变化
    // 拆分提示：A机继续采样并发送亮度，B机接收后更新灯带
    ledcWrite(BUTTON_LED_PIN, 255); 

    // 定时采样 ADC（把新读数写入环形缓冲）
    static unsigned long lastSampleTime = 0;
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
      int currentVoltage = getSmoothedAdcValue();
      voltageBuffer[voltageBufferIndex] = currentVoltage;
      voltageBufferIndex = (voltageBufferIndex + 1) % BUFFER_SIZE;
      lastSampleTime = currentTime;
    }
    
    // 每 UPDATE_INTERVAL 更新一次动态极值（min/max）
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      rawMaxVoltage = -1;
      rawMinVoltage = ADC_RESOLUTION + 1;
      int trimAmount = 1;
      
      // 遍历自上次更新后写入缓冲的所有数据（支持环形情况）
      if (lastUpdateBufferIndex <= voltageBufferIndex) {
        for (int i = lastUpdateBufferIndex; i < voltageBufferIndex; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
      } else {
        // 缓冲已回绕，分两段读取
        for (int i = lastUpdateBufferIndex; i < BUFFER_SIZE; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
        for (int i = 0; i < voltageBufferIndex; i++) {
          rawMaxVoltage = max(rawMaxVoltage, voltageBuffer[i]);
          rawMinVoltage = min(rawMinVoltage, voltageBuffer[i]);
        }
      }
      
      // 若未采集到新数据，则沿用上一轮的极值（防止出现无效空区间）
      if (rawMaxVoltage == -1 || rawMinVoltage == ADC_RESOLUTION + 1) {
        rawMaxVoltage = maxVoltage + trimAmount;
        rawMinVoltage = minVoltage - trimAmount;
      }
      
      // 计算修剪量（去掉 TRIM_RATIO 的两端极值干扰）
      int voltageRange = rawMaxVoltage - rawMinVoltage;
      if (voltageRange <= 0) voltageRange = 200;
      trimAmount = (int)(voltageRange * TRIM_RATIO);
      if (trimAmount < 1) trimAmount = 1;
      
      maxVoltage = rawMaxVoltage - trimAmount;
      minVoltage = rawMinVoltage + trimAmount;
      if (maxVoltage <= minVoltage) {
        // 若经修剪后反转，恢复为基础窗口
        minVoltage = rawMinVoltage;
        maxVoltage = minVoltage + 200;
      }
      
      // 串口打印：显示计算得到的极值（便于调试与参数调整）
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
      
      // 记录此次更新后的缓冲索引，供下次只扫描新增数据
      lastUpdateBufferIndex = voltageBufferIndex;
      lastUpdateTime = currentTime;
    }
    
    // 防止分母/区间为零情况
    if (maxVoltage <= minVoltage) {
      maxVoltage = minVoltage + 1;
    }
    
    // ========== 传感器读数到亮度映射（核心映射逻辑） ==========
    // 说明：该部分为 A 机核心职责（计算当前亮度目标）；拆分为 A->B 通信时应把 targetBrightness 发送到 B 机
    int currentVoltage = getSmoothedAdcValue();
    float actualVoltage = (float)currentVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
    
    // 计算传感器电阻 Rs 与电导（仅供调试打印）
    float Rs = 1e8f;  
    if (actualVoltage > 0.01f) {  
      Rs = R0 * (ADC_REF_VOLTAGE - actualVoltage) / actualVoltage;
    }
    float conductance = (1.0f / Rs) * 1e6f;  
    conductance = constrain(conductance, MIN_CONDUCTANCE, MAX_CONDUCTANCE);
    
    // 将电导映射为近似力值（用于打印）
    int force = map((int)(conductance * 100), 
                   (int)(MIN_CONDUCTANCE * 100), 
                   (int)(MAX_CONDUCTANCE * 100), 
                   MIN_FORCE, 
                   MAX_FORCE);
    
    // 基于动态极值将 AD 值映射为 0-255 亮度
    int targetBrightness = map(currentVoltage, 
                               minVoltage, 
                               maxVoltage, 
                               MIN_BRIGHTNESS, 
                               MAX_BRIGHTNESS);
    targetBrightness = constrain(targetBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    
    // --------------------------
    // 平滑化
    // 说明：applySmoothing 用于避免突变
    // --------------------------
    int smoothedBrightness;
    if (targetBrightness > lastPwmValue) {
      smoothedBrightness = applySmoothing(targetBrightness, lastPwmValue, SMOOTH_UP_FACTOR);
    } else {
      smoothedBrightness = applySmoothing(targetBrightness, lastPwmValue, SMOOTH_DOWN_FACTOR);
    }
    smoothedBrightness = constrain(smoothedBrightness, BUFFER_LOW, BUFFER_HIGH);
    
    // ========== 更新灯带（若有显著变化才更新） ==========
    if (abs(smoothedBrightness - lastPwmValue) >= BRIGHTNESS_THRESHOLD) {
      lastPwmValue = smoothedBrightness;
      int outputBrightness = constrain(lastPwmValue, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
      CRGB dynamicColor = getDynamicColor(outputBrightness);
      FastLED.setBrightness(outputBrightness);
      fill_solid(ledStrip, LED_COUNT, dynamicColor);
      FastLED.show();
    }
    
    // ========== 串口打印当前状态（周期性） ==========
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
    // ========== 暂停（静默）状态 ==========
    // 说明：暂停时仍持续采样并维护极值窗口，但不驱动灯带（仅更新按钮LED）
    // 拆分提示：A 机保持采样/极值更新，B 机根据收到的启停状态选择是否暂停输出
    // 1. 定时采样（与运行时相同）
    static unsigned long lastSampleTime = 0;
    if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
      int currentVoltage = getSmoothedAdcValue();
      voltageBuffer[voltageBufferIndex] = currentVoltage;
      voltageBufferIndex = (voltageBufferIndex + 1) % BUFFER_SIZE;
      lastSampleTime = currentTime;
    }

    // 2. 定时更新极值（与运行时相同逻辑）
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
      rawMaxVoltage = -1;
      rawMinVoltage = ADC_RESOLUTION + 1;
      int trimAmount = 1;
      
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
      
      // 若未采集到数据则沿用上次极值
      if (rawMaxVoltage == -1 || rawMinVoltage == ADC_RESOLUTION + 1) {
        rawMaxVoltage = maxVoltage + trimAmount;
        rawMinVoltage = minVoltage - trimAmount;
      }
      
      // 计算 trim 后的有效极值
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
      
      // 更新缓冲索引记录
      lastUpdateBufferIndex = voltageBufferIndex;
      lastUpdateTime = currentTime;
    }
    
    if (maxVoltage <= minVoltage) {
      maxVoltage = minVoltage + 1;
    }
    
    // 3. 传感器运算与按钮LED映射（与运行时一致，按钮LED用于暂停时的视觉反馈）
    int currentVoltage = getSmoothedAdcValue();
    float actualVoltage = (float)currentVoltage / ADC_RESOLUTION * ADC_REF_VOLTAGE;
    
    // 计算 Rs / 电导（供打印）
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
    
    // 目标亮度（同运行时映射）
    int targetBrightness = map(currentVoltage,
                               minVoltage,
                               maxVoltage,
                               MIN_BRIGHTNESS,
                               MAX_BRIGHTNESS);
    targetBrightness = constrain(targetBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    
    // 4. 按钮LED 的平滑化（与灯带平滑一致）
    int smoothedBrightness;
    if (targetBrightness > lastButtonLedBrightness) {
      smoothedBrightness = applySmoothing(targetBrightness, lastButtonLedBrightness, SMOOTH_UP_FACTOR);
    } else {
      smoothedBrightness = applySmoothing(targetBrightness, lastButtonLedBrightness, SMOOTH_DOWN_FACTOR);
    }
    smoothedBrightness = constrain(smoothedBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
    
    // 5. 更新按钮 LED（仅在超过阈值时更新，避免抖动）
    if (abs(smoothedBrightness - lastButtonLedBrightness) >= BRIGHTNESS_THRESHOLD) {
      lastButtonLedBrightness = smoothedBrightness;
      ledcWrite(BUTTON_LED_PIN, lastButtonLedBrightness);
    }
    
    // 6. 串口打印（与运行时一致，用于调试）
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
  
  // 主循环结尾的短延时，保留以降低 CPU 占用
  delay(50);
}

// ---------------------------
// 工具函数：多次采样并取平均（去噪）
// 说明：该函数为 A 机采样核心，保持原行为
// ---------------------------
int getSmoothedAdcValue() {
  int sum = 0;
  for (int i = 0; i < ADC_SAMPLE_TIMES; i++) {
    sum += analogRead(POT_PIN);
    delayMicroseconds(100);
  }
  return sum / ADC_SAMPLE_TIMES;
}

// ---------------------------
// 串口指令监听（'sa' 恢复运行，'sp' 暂停）
// 说明：保留原命令集，便于在开发/调试阶段通过串口远程控制
// ---------------------------
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

// ---------------------------
// 平滑化函数（用于所有亮度平滑）
// 说明：统一的平滑函数，仅A机使用
// ---------------------------
int applySmoothing(int target, int last, float factor) {
  return (int)(target * factor + last * (1 - factor));
}

// ---------------------------
// 按钮输入处理（去抖并切换运行状态）
// 说明：该逻辑负责把按键事件转换为 programRunning 切换。
//       当从暂停切换到运行时，会重填电压缓冲区以避免极值异常。
//       拆分提示：A 机需要负责此逻辑，并在按钮切换时通过蓝牙把状态广播给 B 机
// ---------------------------
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

/* ===========================================================
   结语与拆分建议（简短版）：
         1) 在 A 机固件中保留：getSmoothedAdcValue、极值更新、按钮处理、serial 命令解析、采样缓冲；
         2) 在 B 机固件中保留：FastLED 初始化、lastPwmValue、输出更新、按钮LED PWM 控制；
         3) 设计一个最小蓝牙数据帧（例如两个字节：亮度、启停），并实现开机自动配对/广播（按顶部说明）。
     - 注意：ESP32-S3 与 ESP32-C3 在 ADC、GPIO、蓝牙栈上的差异，拆分实现时请参考各自 SDK 文档。
   =========================================================== */

