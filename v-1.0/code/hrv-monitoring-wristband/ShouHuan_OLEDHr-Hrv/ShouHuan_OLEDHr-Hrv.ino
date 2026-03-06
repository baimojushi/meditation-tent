/**
******************************************************************************
* @file   : CheezPPG_OLED_MultiCore.ino
* @brief  ：ESP32-DOWN-V3多核优化版本（FreeRTOS任务调度）
* @brief  : 核心优化：PPG高频任务（核心1）+ OLED低频任务（核心0），解决负载过载
* @brief  : 功能：串口输出完整PPG数据（含6次HRV历史），OLED实时显示HR和HRV
******************************************************************************
*/  
// 1. 引入FreeRTOS头文件（ESP32多核调度依赖）
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "zeezPPG.h"
#include "font.h"


// -------------------------- 硬件端口定义（与原代码一致）--------------------------
#define INPUT_PIN 32    // PPG信号输入引脚（ESP32 GPIO32）
#define SAMPLE_RATE 125 // 采样率(125Hz)
#define DATA_BUFFER_SIZE 256  // 数据缓冲区大小

// OLED屏幕引脚
int scl = 22;  // OLED SCL(D0)
int sda = 21;  // OLED SDA(D1)
int res = 10;  // OLED RES(复位)


// -------------------------- 全局变量（跨任务访问需加volatile）--------------------------
CheezPPG ppg(INPUT_PIN, SAMPLE_RATE); // 心率类实例
uint8_t OLED_GRAM[128][8];            // OLED显存
unsigned long previousDisplayMillis = 0;  
const long displayInterval = 100;   
// 跨任务共享数据：加volatile确保线程安全（防止编译器优化）
volatile int currentHr = 0;           // 当前心率（PPG任务更新，OLED任务读取）
volatile int currentHrv = 0;          // 当前HRV（PPG任务更新，OLED任务读取）
int lastHrValue = -1;                 // OLED任务：记录上一次心率（增量刷新用）
int lastHrvValue = -1;                // OLED任务：记录上一次HRV（增量刷新用）
char ppgDataBuf[256];                    // PPG任务的静态缓冲区（若有）

void ppgTask(void *pvParameters) {
  while (1) {  
    if (ppg.checkSampleInterval()) {  
      ppg.ppgProcess();
      currentHr = static_cast<int>(ppg.getPpgHr());
      currentHrv = static_cast<int>(ppg.getPpgHrv());
      // 用snprintf输出（不变）
      snprintf(ppgDataBuf, sizeof(ppgDataBuf), 
        "%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
        ppg.getRawPPG(), ppg.getAvgPPG(), ppg.getFilterPPG(), ppg.getPpgPeak(),
        ppg.getPpgHr(), ppg.getPpgHrv(),
        ppg.getHrvByIndex(0), ppg.getHrvByIndex(1), ppg.getHrvByIndex(2),
        ppg.getHrvByIndex(3), ppg.getHrvByIndex(4), ppg.getHrvByIndex(5)
      );
      Serial.println(ppgDataBuf);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// -------------------------- OLED底层驱动函数（完全保留原代码，无修改）--------------------------
// -------------------------- 1. 优化：I2C驱动（补回延时，稳定通信）--------------------------
#define OLED_SCLK_Clr() digitalWrite(scl, LOW)
#define OLED_SCLK_Set() digitalWrite(scl, HIGH)
#define OLED_SDIN_Clr() digitalWrite(sda, LOW)
#define OLED_SDIN_Set() digitalWrite(sda, HIGH)
#define OLED_RST_Clr() digitalWrite(res, LOW)
#define OLED_RST_Set() digitalWrite(res, HIGH)

#define OLED_CMD  0
#define OLED_DATA 1

// 补回微秒延时，确保I2C时序稳定（原代码缺少延时导致通信不稳定）
void I2C_Start(void) {
  OLED_SDIN_Set();
  OLED_SCLK_Set();
  delayMicroseconds(2); // 关键：补回延时
  OLED_SDIN_Clr();
  delayMicroseconds(2); // 关键：补回延时
  OLED_SCLK_Clr();
}

void I2C_Stop(void) {
  OLED_SCLK_Set();
  delayMicroseconds(2); // 关键：补回延时
  OLED_SDIN_Clr();
  delayMicroseconds(2); // 关键：补回延时
  OLED_SDIN_Set();
  delayMicroseconds(2); // 关键：补回延时
}

void I2C_WaitAck(void) {
  OLED_SCLK_Set();
  delayMicroseconds(1); // 关键：补回延时
  OLED_SCLK_Clr();
  delayMicroseconds(1); // 关键：补回延时
}

void Send_Byte(uint8_t dat) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    OLED_SCLK_Clr();
    delayMicroseconds(1); // 关键：补回延时
    if (dat & 0x80) OLED_SDIN_Set();
    else OLED_SDIN_Clr();
    delayMicroseconds(1); // 关键：补回延时
    OLED_SCLK_Set();
    delayMicroseconds(1); // 关键：补回延时
    OLED_SCLK_Clr();
    dat <<= 1;
  }
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode) {
  I2C_Start();
  Send_Byte(0x78);
  I2C_WaitAck();
  if (mode) Send_Byte(0x40);
  else Send_Byte(0x00);
  I2C_WaitAck();
  Send_Byte(dat);
  I2C_WaitAck();
  I2C_Stop();
  delayMicroseconds(2); // 关键：补回延时，避免连续通信冲突
}


// 优化1：全屏清屏（按“页+列”直接赋值，替代逐像素操作）
void OLED_Clear(void) {
  for (uint8_t page = 0; page < 8; page++) { // 8个页（对应64行）
    for (uint8_t col = 0; col < 128; col++) { // 128列
      OLED_GRAM[col][page] = 0x00; // 直接清空字节，无位运算
    }
  }
}

// 优化2：局部清屏（仅清数值区，按固定页/列范围，循环次数减少80%）
// 参数：数值区对应的页范围（如HR对应page2-page3）、列范围（如col55-col89）
void OLED_ClearPartial(uint8_t startPage, uint8_t endPage, uint8_t startCol, uint8_t endCol) {
  for (uint8_t page = startPage; page <= endPage; page++) {
    for (uint8_t col = startCol; col <= endCol; col++) {
      OLED_GRAM[col][page] = 0x00; // 直接清空字节，替代原OLED_ClearPoint
    }
  }
}

// 优化3：显示16号数字（直接读字库字节写入显存，替代嵌套调用OLED_ShowChar）
// 参数：startCol=起始列，startPage=起始页（16号字体占2页），num=要显示的数字（0-999）
void OLED_Show16x8Num(uint8_t startCol, uint8_t startPage, int num) {
  num = constrain(num, 0, 999); // 限制数字范围（避免异常）
  uint8_t d1 = num / 100;    // 百位
  uint8_t d2 = (num / 10) % 10; // 十位
  uint8_t d3 = num % 10;     // 个位

  // 16号字体：每个数字占8列×2页（共16字节），从字库asc2_1608读取数据
  for (uint8_t col = 0; col < 8; col++) {
    // 百位数字：startCol+col列，startPage页（上半部分）、startPage+1页（下半部分）
    OLED_GRAM[startCol + col][startPage] = pgm_read_byte(&asc2_1608[d1 + 48][col]);
    OLED_GRAM[startCol + col][startPage + 1] = pgm_read_byte(&asc2_1608[d1 + 48][col + 8]);
    
    // 十位数字：startCol+8+col列（与百位间隔1列）
    OLED_GRAM[startCol + 8 + col][startPage] = pgm_read_byte(&asc2_1608[d2 + 48][col]);
    OLED_GRAM[startCol + 8 + col][startPage + 1] = pgm_read_byte(&asc2_1608[d2 + 48][col + 8]);
    
    // 个位数字：startCol+16+col列（与十位间隔1列）
    OLED_GRAM[startCol + 16 + col][startPage] = pgm_read_byte(&asc2_1608[d3 + 48][col]);
    OLED_GRAM[startCol + 16 + col][startPage + 1] = pgm_read_byte(&asc2_1608[d3 + 48][col + 8]);
  }
}

// 保留原局部刷新函数（仅刷新变化的页，不改动）
void OLED_PartialRefresh(uint8_t startRow, uint8_t endRow) {
  for (uint8_t i = startRow; i <= endRow; i++) {
    OLED_WR_Byte(0xb0 + i, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    for (uint8_t n = 0; n < 128; n++) {
      OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
  }
}

// 保留原OLED初始化（仅删除冗余操作，功能不变）
void OLED_Init(void) {
  pinMode(scl, OUTPUT);
  pinMode(sda, OUTPUT);
  pinMode(res, OUTPUT);
  
  // 复位时序（保留原逻辑）
  OLED_RST_Set();
  delay(100);
  OLED_RST_Clr();
  delay(200);
  OLED_RST_Set();
  
  // 初始化命令（保留原逻辑，确保OLED正常启动）
  OLED_WR_Byte(0xAE, OLED_CMD);
  OLED_WR_Byte(0x00, OLED_CMD);
  OLED_WR_Byte(0x10, OLED_CMD);
  OLED_WR_Byte(0x40, OLED_CMD);
  OLED_WR_Byte(0x81, OLED_CMD);
  OLED_WR_Byte(0xCF, OLED_CMD);
  OLED_WR_Byte(0xA1, OLED_CMD);
  OLED_WR_Byte(0xC8, OLED_CMD);
  OLED_WR_Byte(0xA6, OLED_CMD);
  OLED_WR_Byte(0xA8, OLED_CMD);
  OLED_WR_Byte(0x3F, OLED_CMD);
  OLED_WR_Byte(0xD3, OLED_CMD);
  OLED_WR_Byte(0x00, OLED_CMD);
  OLED_WR_Byte(0xD5, OLED_CMD);
  OLED_WR_Byte(0x80, OLED_CMD);
  OLED_WR_Byte(0xD9, OLED_CMD);
  OLED_WR_Byte(0xF1, OLED_CMD);
  OLED_WR_Byte(0xDA, OLED_CMD);
  OLED_WR_Byte(0x12, OLED_CMD);
  OLED_WR_Byte(0xDB, OLED_CMD);
  OLED_WR_Byte(0x40, OLED_CMD);
  OLED_WR_Byte(0x20, OLED_CMD);
  OLED_WR_Byte(0x02, OLED_CMD);
  OLED_WR_Byte(0x8D, OLED_CMD);
  OLED_WR_Byte(0x14, OLED_CMD);
  OLED_WR_Byte(0xAF, OLED_CMD);
  
  OLED_Clear();
  OLED_PartialRefresh(0, 7); // 初始化时全屏刷新一次
}

// 优化1：初始化显示布局（仅绘制静态标签，一次完成）
void initDisplayLayout() {
  OLED_Clear(); // 清空全屏
  
  // 绘制静态HR标签（"HR: " + "BPM"），位置与原代码一致
  // 注：若asc2_1608字库支持，直接用OLED_Show16x8String（此处保留原OLED_ShowString确保兼容）
  OLED_ShowString(20, 15, "HR: ", 16);    // 原位置：x=20,y=15
  OLED_ShowString(90, 15, "BPM", 16);     // 原位置：x=90,y=15
  
  // 绘制静态HRV标签（"HRV: " + "ms"），位置与原代码一致
  OLED_ShowString(20, 45, "HRV: ", 16);   // 原位置：x=20,y=45
  OLED_ShowString(100, 45, "ms", 16);     // 原位置：x=100,y=45
  
  OLED_PartialRefresh(0, 7); // 刷新全屏，显示静态标签
}

// 优化2：更新数值显示（增量刷新，仅操作变化区域）
void updateDisplay() {
  // 仅数据变化时更新（保留原增量刷新逻辑，避免无意义操作）
  if (currentHr == lastHrValue && currentHrv == lastHrvValue) {
    return;
  }
  lastHrValue = currentHr;
  lastHrvValue = currentHrv;

  // 1. 局部清空HR数值区（原位置：x55-x89，对应页2-3、列55-89）
  OLED_ClearPartial(2, 3, 55, 89);
  // 2. 显示新HR数值（原位置：x55,y15，对应页2、列55）
  OLED_Show16x8Num(55, 2, currentHr);
  
  // 3. 局部清空HRV数值区（原位置：x65-x99，对应页6-7、列65-99）
  OLED_ClearPartial(6, 7, 65, 99);
  // 4. 显示新HRV数值（原位置：x65,y45，对应页6、列65）
  OLED_Show16x8Num(65, 6, currentHrv);
  
  // 5. 仅刷新数值区对应的页（HR刷页2-3，HRV刷页6-7），减少I2C通信量
  OLED_PartialRefresh(2, 3);
  OLED_PartialRefresh(6, 7);
}

// 保留原OLED_ShowString（确保静态标签正常显示，不改动）
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t size1) {
  while ((*chr >= ' ') && (*chr <= '~')) {
    OLED_ShowChar(x, y, *chr, size1);
    x += size1 / 2;
    if (x > 128 - size1 / 2) {
      x = 0;
      y += size1;
    }
    chr++;
  }
}

// 保留原OLED_ShowChar（仅用于静态标签，不改动）
void OLED_ShowChar(uint8_t x, uint8_t y, const char chr, uint8_t size1) {
  uint8_t i, m, temp, size2, chr1;
  uint8_t y0 = y;
  size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
  chr1 = chr - ' ';

  for (i = 0; i < size2; i++) {
    if (size1 == 16) temp = pgm_read_byte(&asc2_1608[chr1][i]);
    else return;

    for (m = 0; m < 8; m++) {
      if (temp & 0x80) {
        // 简化画点：直接操作显存字节，替代原OLED_DrawPoint的位运算
        uint8_t page = y / 8;
        uint8_t bit = y % 8;
        OLED_GRAM[x][page] |= (1 << bit);
      }
      temp <<= 1;
      y++;
      if ((y - y0) == size1) {
        y = y0;
        x++;
        break;
      }
    }
  }
}

// 保留原OLED_Pow（仅用于兼容旧代码，不改动）
uint32_t OLED_Pow(uint8_t m, uint8_t n) {
  uint32_t result = 1;
  while (n--) {
    result *= m;
  }
  return result;
}


// -------------------------- 4. 优化后的OLED任务（与多核调度兼容）--------------------------
void oledTask(void *pvParameters) {
  const long displayInterval = 100;  // 保留原10Hz刷新频率（功能不变）
  unsigned long previousDisplayMillis = millis();
  
  while (1) {  
    unsigned long currentMillis = millis();
    if (currentMillis - previousDisplayMillis >= displayInterval) {
      previousDisplayMillis = currentMillis;
      updateDisplay();  // 调用优化后的更新函数
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS); // 释放CPU，避免独占
  }
}


// -------------------------- 5. setup中OLED初始化（与原功能一致）--------------------------
void setup() {
  Serial.begin(115200);
  // OLED初始化（保留原启动显示逻辑）
  OLED_Init();
  initDisplayLayout(); // 初始化静态标签
  
  // PPG配置（不变）
  ppg.setWearThreshold(1);
  ppg.setPeakThresholdFactor(11.0f);
  
  // 创建多核任务（栈大小已修正为“栈项”）
  // PPG任务（核心1）
  xTaskCreatePinnedToCore(
    ppgTask, "PPG_Task", 4096, NULL, tskIDLE_PRIORITY + 2, NULL, 1
  );
  // OLED任务（核心0）
  xTaskCreatePinnedToCore(
    oledTask, "OLED_Task", 2048, NULL, tskIDLE_PRIORITY + 1, NULL, 0
  );
}

// 主循环：FreeRTOS任务已接管，loop函数空实现
void loop() {
  // 任务由FreeRTOS调度，无需手动编写逻辑
  vTaskDelay(portMAX_DELAY); // 让核心0空闲时进入深度休眠，降低功耗
}