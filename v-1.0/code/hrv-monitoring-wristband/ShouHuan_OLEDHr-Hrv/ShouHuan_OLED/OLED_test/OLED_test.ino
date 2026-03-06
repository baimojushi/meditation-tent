/**
******************************************************************************
* @file   : OLED_Test.ino
* @brief  ：OLED独立测试程序（显示随机HR和HRV数值）
* @brief  ：功能：生成随机心率(HR)和心率变异性(HRV)，通过OLED实时显示
******************************************************************************
*/  
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "font.h"  // 需保留原字库文件

// -------------------------- OLED硬件端口定义 --------------------------
#define OLED_SCL 22  // OLED SCL引脚
#define OLED_SDA 21  // OLED SDA引脚
#define OLED_RES 10  // OLED复位引脚

// -------------------------- 全局变量 --------------------------
uint8_t OLED_GRAM[128][8];  // OLED显存
unsigned long previousDisplayMillis = 0;
const long displayInterval = 300;  // 10Hz刷新频率
int lastHrValue = -1;        // 上一次心率值（用于增量刷新）
int lastHrvValue = -1;       // 上一次HRV值
volatile int currentHr = 0;  // 当前心率（随机生成）
volatile int currentHrv = 0; // 当前HRV（随机生成）

// -------------------------- I2C底层驱动函数 --------------------------
#define OLED_SCLK_Clr() digitalWrite(OLED_SCL, LOW)
#define OLED_SCLK_Set() digitalWrite(OLED_SCL, HIGH)
#define OLED_SDIN_Clr() digitalWrite(OLED_SDA, LOW)
#define OLED_SDIN_Set() digitalWrite(OLED_SDA, HIGH)
#define OLED_RST_Clr() digitalWrite(OLED_RES, LOW)
#define OLED_RST_Set() digitalWrite(OLED_RES, HIGH)

#define OLED_CMD  0
#define OLED_DATA 1

void I2C_Start(void) {
  OLED_SDIN_Set();
  OLED_SCLK_Set();
  delayMicroseconds(2);
  OLED_SDIN_Clr();
  delayMicroseconds(2);
  OLED_SCLK_Clr();
}

void I2C_Stop(void) {
  OLED_SCLK_Set();
  delayMicroseconds(2);
  OLED_SDIN_Clr();
  delayMicroseconds(2);
  OLED_SDIN_Set();
  delayMicroseconds(2);
}

void I2C_WaitAck(void) {
  OLED_SCLK_Set();
  delayMicroseconds(1);
  OLED_SCLK_Clr();
  delayMicroseconds(1);
}

void Send_Byte(uint8_t dat) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    OLED_SCLK_Clr();
    delayMicroseconds(1);
    if (dat & 0x80) OLED_SDIN_Set();
    else OLED_SDIN_Clr();
    delayMicroseconds(1);
    OLED_SCLK_Set();
    delayMicroseconds(1);
    OLED_SCLK_Clr();
    dat <<= 1;
  }
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode) {
  I2C_Start();
  Send_Byte(0x78);  // OLED I2C地址（0x3C左移1位）
  I2C_WaitAck();
  if (mode) Send_Byte(0x40);
  else Send_Byte(0x00);
  I2C_WaitAck();
  Send_Byte(dat);
  I2C_WaitAck();
  I2C_Stop();
  delayMicroseconds(2);
}

// -------------------------- OLED显示控制函数 --------------------------
// 全屏清屏
void OLED_Clear(void) {
  for (uint8_t page = 0; page < 8; page++) {
    for (uint8_t col = 0; col < 128; col++) {
      OLED_GRAM[col][page] = 0x00;
    }
  }
}

// 局部清屏（指定页和列范围）
void OLED_ClearPartial(uint8_t startPage, uint8_t endPage, uint8_t startCol, uint8_t endCol) {
  for (uint8_t page = startPage; page <= endPage; page++) {
    for (uint8_t col = startCol; col <= endCol; col++) {
      OLED_GRAM[col][page] = 0x00;
    }
  }
}

// 显示16号3位数字（0-999）
void OLED_Show16x8Num(uint8_t startCol, uint8_t startPage, int num) {
  num = constrain(num, 0, 999);
  uint8_t d1 = num / 100;    // 百位
  uint8_t d2 = (num / 10) % 10; // 十位
  uint8_t d3 = num % 10;     // 个位

  for (uint8_t col = 0; col < 8; col++) {
    // 百位数字
    OLED_GRAM[startCol + col][startPage] = pgm_read_byte(&asc2_1608[d1 + 48][col]);
    OLED_GRAM[startCol + col][startPage + 1] = pgm_read_byte(&asc2_1608[d1 + 48][col + 8]);
    // 十位数字
    OLED_GRAM[startCol + 8 + col][startPage] = pgm_read_byte(&asc2_1608[d2 + 48][col]);
    OLED_GRAM[startCol + 8 + col][startPage + 1] = pgm_read_byte(&asc2_1608[d2 + 48][col + 8]);
    // 个位数字
    OLED_GRAM[startCol + 16 + col][startPage] = pgm_read_byte(&asc2_1608[d3 + 48][col]);
    OLED_GRAM[startCol + 16 + col][startPage + 1] = pgm_read_byte(&asc2_1608[d3 + 48][col + 8]);
  }
}

// 局部刷新（指定页范围）
void OLED_PartialRefresh(uint8_t startRow, uint8_t endRow, uint8_t startCol, uint8_t endCol) {
  for (uint8_t i = startRow; i <= endRow; i++) {
    OLED_WR_Byte(0xb0 + i, OLED_CMD);
    // 设置列起始地址（低4位+高4位）
    OLED_WR_Byte(0x00 + (startCol & 0x0F), OLED_CMD);  // 列起始低4位
    OLED_WR_Byte(0x10 + (startCol >> 4), OLED_CMD);    // 列起始高4位
    // 仅刷新startCol到endCol的列（减少I2C通信量）
    for (uint8_t n = startCol; n <= endCol; n++) {
      OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
    }
  }
}


// 初始化OLED硬件
void OLED_Init(void) {
  pinMode(OLED_SCL, OUTPUT);
  pinMode(OLED_SDA, OUTPUT);
  pinMode(OLED_RES, OUTPUT);
  
  // 硬件复位
  OLED_RST_Set();
  delay(100);
  OLED_RST_Clr();
  delay(200);
  OLED_RST_Set();
  
  // 初始化命令
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

}

// 显示字符（用于绘制静态标签）
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

// 显示字符串（用于绘制静态标签）
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

// 初始化显示布局（绘制静态标签）
void initDisplayLayout() {
  OLED_Clear();
  OLED_ShowString(20, 15, "HR: ", 16);    // 心率标签
  OLED_ShowString(90, 15, "BPM", 16);     // 心率单位
  OLED_ShowString(20, 45, "HRV: ", 16);   // HRV标签
  OLED_ShowString(100, 45, "ms", 16);     // HRV单位

}

// 更新显示内容（增量刷新）
void updateDisplay() {
  if (currentHr == lastHrValue && currentHrv == lastHrvValue) {
    return;
  }
  lastHrValue = currentHr;
  lastHrvValue = currentHrv;

  // 局部清空并显示HR（行2-3，列55-89）
  OLED_ClearPartial(2, 3, 55, 89);
  OLED_Show16x8Num(55, 2, currentHr);
  OLED_PartialRefresh(2, 3, 55, 89);  // 修正：传入4个参数
  
  // 局部清空并显示HRV（行6-7，列65-99）
  OLED_ClearPartial(6, 7, 65, 99);
  OLED_Show16x8Num(65, 6, currentHrv);
  OLED_PartialRefresh(6, 7, 65, 99);  // 修正：传入4个参数
}

// -------------------------- 随机数生成任务 --------------------------
void randomDataTask(void *pvParameters) {
  // 初始化随机数种子（基于芯片唯一ID）
  uint64_t chipId = ESP.getEfuseMac();
  randomSeed(chipId);
  
  while (1) {
    // 生成随机HR（60-100）和HRV（20-100）
    currentHr = random(60, 101);    // 心率范围：60-100 BPM
    currentHrv = random(20, 101);   // HRV范围：20-100 ms
    
    vTaskDelay(500 / portTICK_PERIOD_MS);  // 每500ms更新一次随机数
  }
}

// -------------------------- OLED显示任务 --------------------------
void oledTask(void *pvParameters) {
  while (1) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousDisplayMillis >= displayInterval) {
      previousDisplayMillis = currentMillis;
      updateDisplay();
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

// -------------------------- 主函数 --------------------------
void setup() {
  Serial.begin(115200);
  OLED_Init();
  initDisplayLayout();
  Serial.println("OLED测试开始：显示随机HR和HRV数值");
  
  // 创建任务
  xTaskCreatePinnedToCore(
    randomDataTask, "RandomDataTask", 1024, NULL, 1, NULL, 1  // 核心1生成随机数
  );
  xTaskCreatePinnedToCore(
    oledTask, "OledTask", 2048, NULL, 1, NULL, 0  // 核心0显示
  );
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // 主循环空闲
}
