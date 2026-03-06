#include "zeezPPG.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// 引入ESP32蓝牙串口库
#include "BluetoothSerial.h"

// 蓝牙串口对象实例化
BluetoothSerial BTSerial;

#define INPUT_PIN 32    // PPG引脚
#define SAMPLE_RATE 125 // 采样率
#define DATA_BUFFER_SIZE 128

// 全局互斥锁用于保护共享数据
SemaphoreHandle_t rrmutex;
// 全局变量存储HRV计算结果
float latestHrv = 0.0f;
float hrvHistory[6] = {0};

CheezPPG ppg(INPUT_PIN, SAMPLE_RATE);  

// 第二核心任务：处理HRV计算
void hrvCalculationTask(void *param) {
  while (true) {
    // 每20秒计算一次HRV
    vTaskDelay(pdMS_TO_TICKS(20000));
    
    // 检查佩戴状态
    if (!ppg.getPpgisWear()) {
      latestHrv = 0.0f;
      continue;
    }

    // 加锁保护共享数据访问
    if (xSemaphoreTake(rrmutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      // 获取RR间期数据
      const auto& rrBuffer = ppg.getRRBuffer();
      int totalRRCount = rrBuffer.numItems();
      int useRRCount = min(totalRRCount, 60);

      float rmssd = 0.0f;
      if (useRRCount >= 40) {
        unsigned long long diffSquareSum = 0;
        int validDiffCount = 0;
        int startIdx = totalRRCount - useRRCount;

        for (int i = startIdx; i < totalRRCount - 1; i++) {
          unsigned long rr1 = rrBuffer.peek(i);
          unsigned long rr2 = rrBuffer.peek(i + 1);
          long diff = rr2 - rr1;
          diffSquareSum += diff * diff;
          validDiffCount++;
        }

        if (validDiffCount >= 4) {
          float meanDiffSquare = (float)diffSquareSum / validDiffCount;
          rmssd = sqrt(meanDiffSquare);
          rmssd = (rmssd >= 5 && rmssd <= 150) ? rmssd : 0.0f;
        }
      }

      // 更新HRV历史数据
      for (int i = 5; i > 0; i--) {
        hrvHistory[i] = hrvHistory[i - 1];
      }
      hrvHistory[0] = rmssd;
      latestHrv = rmssd;

      xSemaphoreGive(rrmutex);
    }
  }
}

void setup() {
  // 初始化硬件串口（USB）
  Serial.begin(115200);
  
  // 初始化蓝牙串口，设备名称为"ESP32-PPG-Monitor"
  if (!BTSerial.begin("ESP32-PPG-Monitor")) {
    Serial.println("蓝牙初始化失败！");
    while (1); // 初始化失败则死循环
  }
  Serial.println("蓝牙初始化成功，可搜索连接ESP32-PPG-Monitor");

  // PPG传感器配置
  ppg.setWearThreshold(1);
  ppg.setPeakThresholdFactor(11.0f);
  
  // 同时向硬件串口和蓝牙串口输出表头
  String header = "Raw,Avg,Filtered,Peak,HR,LatestHRV,HRV1,HRV2,HRV3,HRV4,HRV5,HRV6";
  Serial.println(header);
  BTSerial.println(header);
  
  // 初始化互斥锁
  rrmutex = xSemaphoreCreateMutex();
  
  // 创建HRV计算任务并绑定到核心1
  xTaskCreatePinnedToCore(
    hrvCalculationTask,    // 任务函数
    "HRV Task",            // 任务名称
    4096,                  // 栈大小
    NULL,                  // 参数
    1,                     // 优先级（低于主任务）
    NULL,                  // 任务句柄
    1                      // 核心1
  );
}

void loop() {   
  if (ppg.checkSampleInterval()) {  
    // 加锁保护RR缓冲区写入
    if (xSemaphoreTake(rrmutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      ppg.ppgProcess();  // 主核心处理采样和心率计算
      xSemaphoreGive(rrmutex);
    }

    // 构建输出数据
    String baseData = String(ppg.getRawPPG())    + "," + 
                      String(ppg.getAvgPPG())    + "," + 
                      String(ppg.getFilterPPG()) + "," + 
                      String(ppg.getPpgPeak())   + "," + 
                      String(ppg.getPpgHr(), 1);

    // 读取HRV数据（加锁保护）
    if (xSemaphoreTake(rrmutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      baseData += "," + String(latestHrv, 1);
      for (int i = 0; i < 6; i++) {
        baseData += "," + String(hrvHistory[i], 1);
      }
      xSemaphoreGive(rrmutex);
    }

    // 同时向硬件串口（USB）和蓝牙串口输出数据
    Serial.println(baseData);
    BTSerial.println(baseData);
  }   
}
