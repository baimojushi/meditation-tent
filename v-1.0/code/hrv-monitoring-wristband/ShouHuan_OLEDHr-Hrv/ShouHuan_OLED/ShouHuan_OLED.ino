/**
******************************************************************************
* @file   : ShouHuan_OLED.ino
* @brief  ：优化串口通信，解决周期性卡顿问题
******************************************************************************/  
#include "zeezPPG.h"
#include "font.h"

// -------------------------- 1. 硬件配置 --------------------------
#define INPUT_PIN 32        // PPG传感器引脚
#define SAMPLE_RATE 125     // 采样率（Hz）
CheezPPG ppg(INPUT_PIN, SAMPLE_RATE);

// OLED引脚定义
int scl = 22;  // OLED SCL(D0)
int sda = 21;  // OLED SDA(D1)
int res = 10; // OLED RES(复位)

// -------------------------- 2. 优化参数 --------------------------
const unsigned long OLED_REFRESH_INTERVAL = 500;  // OLED刷新间隔（500ms）
unsigned long last_oled_refresh = 0;              
bool oled_need_refresh = false;                   

// -------------------------- 3. 全局变量 --------------------------
// 环形缓冲区（FIFO）用于串口非阻塞发送
#define UART_RING_BUF_SIZE 256  // 环形缓冲区大小
uint8_t uart_ring_buf[UART_RING_BUF_SIZE];  // 缓冲区数组
volatile uint8_t uart_wr_idx = 0;           // 写指针
volatile uint8_t uart_rd_idx = 0;           // 读指针
volatile uint8_t uart_buf_count = 0;        // 缓冲区数据长度

uint8_t OLED_GRAM[128][8];  // 显存缓冲区
uint8_t current_page = 0;
unsigned long last_switch_time = 0;
const unsigned long PAGE_INTERVAL = 5000;

int current_raw = 0;
int current_avg = 0;
int current_filter = 0;
int current_peak = 0;
int current_hr = 0;
int current_hrv = 0;
bool page3_locked = false;
unsigned long raw_below_time = 0;

#define ASCII_1608_SIZE 16
#define CHINESE_SZ 16
extern const unsigned char asc2_1206[95][12] PROGMEM;
extern const unsigned char asc2_1608[95][16] PROGMEM;
extern const unsigned char Hzk5[16][16] PROGMEM;
extern const unsigned char Hzk6[26][16] PROGMEM;
extern const unsigned char Hzk7[22][16] PROGMEM;

// -------------------------- 4. 宏定义 --------------------------
#define OLED_SCLK_Clr() digitalWrite(scl, LOW)
#define OLED_SCLK_Set() digitalWrite(scl, HIGH)
#define OLED_SDIN_Clr() digitalWrite(sda, LOW)
#define OLED_SDIN_Set() digitalWrite(sda, HIGH)
#define OLED_RST_Clr() digitalWrite(res, LOW)
#define OLED_RST_Set() digitalWrite(res, HIGH)
#define OLED_CMD  0
#define OLED_DATA 1
#define RAW_THRESHOLD 100
#define EXIT_LOCK_DELAY 5000

// -------------------------- 5. 主函数 --------------------------
void setup() {
  Serial.begin(115200);
  OLED_Init();  // 初始化OLED
  ppg.setWearThreshold(-1);
  Show_Page(current_page);
  OLED_Refresh();  // 刷新显示
  last_oled_refresh = millis();
}

void loop() {
  static unsigned long last_uart_send = 0;
  
  processPPGData();       // 处理PPG数据
  handlePageSwitch();     // 处理页面切换
  refreshOLEDIfNeeded();  // 刷新OLED（条件触发）
  
  // 高频触发串口发送（每1ms至少检查1次）
  if (millis() - last_uart_send >= 1) {
    sendSerialDataNonBlocking();
    last_uart_send = millis();
  }
}

// -------------------------- 6. PPG数据处理 --------------------------
void processPPGData() {
  if (ppg.checkSampleInterval()) {  
    ppg.ppgProcess();
    
    // 更新数据
    current_raw = ppg.getRawPPG();
    current_avg = ppg.getAvgPPG();
    current_filter = ppg.getFilterPPG();
    current_peak = ppg.getPpgPeak();
    current_hr = ppg.getPpgHr();
    current_hrv = ppg.getPpgHrv();
    
    // 格式化数据帧到临时缓冲区
    char temp_buf[32];  // 临时缓冲区
    int temp_len = snprintf(temp_buf, sizeof(temp_buf), 
      "%d,%d,%d,%d,%d,%d\n",
      current_raw, current_avg, current_filter,
      current_peak, current_hr, current_hrv
    );
    if (temp_len <= 0 || temp_len >= sizeof(temp_buf)) {
      return;  // 格式化失败，跳过
    }
    
    // 逐字节写入环形缓冲区
    for (int i = 0; i < temp_len; i++) {
      uart_ring_buf_put(temp_buf[i]);  // 缓冲区满时自动丢弃
    }
    
    // 页3时标记OLED刷新
    if (current_page == 2) {
      Update_HRV_Number();
      oled_need_refresh = true;
    }
  }
}

// -------------------------- 7. 环形缓冲区工具函数 --------------------------
// 向环形缓冲区写入1字节（返回是否成功）
bool uart_ring_buf_put(uint8_t data) {
  if (uart_buf_count >= UART_RING_BUF_SIZE) {
    return false;  // 缓冲区满，丢弃数据
  }
  uart_ring_buf[uart_wr_idx] = data;
  uart_wr_idx = (uart_wr_idx + 1) % UART_RING_BUF_SIZE;  // 写指针循环
  uart_buf_count++;
  return true;
}

// 从环形缓冲区读取1字节（返回是否成功）
bool uart_ring_buf_get(uint8_t *data) {
  if (uart_buf_count == 0) {
    return false;  // 缓冲区空，无数据可读
  }
  *data = uart_ring_buf[uart_rd_idx];
  uart_rd_idx = (uart_rd_idx + 1) % UART_RING_BUF_SIZE;  // 读指针循环
  uart_buf_count--;
  return true;
}

// 判断环形缓冲区是否为空
bool uart_ring_buf_is_empty() {
  return (uart_buf_count == 0);
}

// -------------------------- 8. 串口非阻塞发送 --------------------------
void sendSerialDataNonBlocking() {
  // 只要串口有发送空间且缓冲区有数据，就持续发送（1字节/次）
  while (!uart_ring_buf_is_empty() && Serial.availableForWrite() > 0) {
    uint8_t data;
    if (uart_ring_buf_get(&data)) {
      Serial.write(data);  // 发送1字节
    }
  }
}

// -------------------------- 9. 页面切换逻辑 --------------------------
void handlePageSwitch() {
  if (current_raw > RAW_THRESHOLD) {
    if (!page3_locked) {
      page3_locked = true;
      current_page = 2;
      OLED_Clear();
      Show_Page(current_page);
      oled_need_refresh = true;
      last_switch_time = millis();
    }
    raw_below_time = 0;
  } 
  else if (page3_locked) {
    if (raw_below_time == 0) raw_below_time = millis();
    else if (millis() - raw_below_time >= EXIT_LOCK_DELAY) {
      page3_locked = false;
      current_page = 0;
      OLED_Clear();
      Show_Page(current_page);
      oled_need_refresh = true;
      last_switch_time = millis();
    }
  }
  else if (millis() - last_switch_time >= PAGE_INTERVAL) {
    current_page = (current_page + 1) % 2;
    OLED_Clear();
    Show_Page(current_page);
    oled_need_refresh = true;
    last_switch_time = millis();
  }
}

// -------------------------- 10. OLED刷新控制 --------------------------
void refreshOLEDIfNeeded() {
  // 若串口缓冲区有数据，优先发送，暂缓OLED刷新
  if (uart_buf_count > 0) return;
  
  if (oled_need_refresh && (millis() - last_oled_refresh >= OLED_REFRESH_INTERVAL)) {
    OLED_Refresh();
    last_oled_refresh = millis();
    oled_need_refresh = false;
  }
}

// -------------------------- 11. HRV数值局部更新 --------------------------
void Update_HRV_Number() {
  const uint8_t hrv_y = 10;
  const uint8_t total_char = 4 + 3;
  const uint16_t total_w = total_char * 8;
  const uint8_t start_x = (128 - total_w) / 2;
  const uint8_t num_x = start_x + 4*8;
  
  // 清除原有数值
  for (int x = num_x; x < num_x + 3*8; x++) {
    for (int y = hrv_y; y < hrv_y + 16; y++) {
      OLED_ClearPoint(x, y);
    }
  }
  // 绘制新数值
  OLED_ShowNum(num_x, hrv_y, current_hrv, 3, ASCII_1608_SIZE);
}

// -------------------------- 12. 页面显示函数 --------------------------
void Show_Page(uint8_t page) {
  switch (page) {
    case 0: Show_Page0(); break;
    case 1: Show_Page1(); break;
    case 2: Show_Page2(); break;
    default: Show_Page0(); break;
  }
}

void Show_Page0() {
  const uint8_t ch_num = 8;
  const uint16_t total_w = ch_num * CHINESE_SZ;
  const uint8_t x = (128 - total_w) / 2;
  const uint8_t y = (64 - CHINESE_SZ) / 2;

  OLED_ShowChinese(x + 0*CHINESE_SZ, y, 0, CHINESE_SZ, Hzk5);
  OLED_ShowChinese(x + 1*CHINESE_SZ, y, 1, CHINESE_SZ, Hzk5);
  OLED_ShowChinese(x + 2*CHINESE_SZ, y, 2, CHINESE_SZ, Hzk5);
  OLED_ShowChinese(x + 3*CHINESE_SZ, y, 3, CHINESE_SZ, Hzk5);
  OLED_ShowChinese(x + 4*CHINESE_SZ, y, 4, CHINESE_SZ, Hzk5);
  OLED_ShowChinese(x + 5*CHINESE_SZ, y, 5, CHINESE_SZ, Hzk5);
  OLED_ShowChinese(x + 6*CHINESE_SZ, y, 6, CHINESE_SZ, Hzk5);
  OLED_ShowChinese(x + 7*CHINESE_SZ, y, 7, CHINESE_SZ, Hzk5);
}

void Show_Page1() {
  const uint8_t y1 = 16, y2 = 16 + CHINESE_SZ;
  const uint8_t ch1_num = 6;
  const uint16_t w1 = ch1_num * CHINESE_SZ;
  const uint8_t x1 = (128 - w1) / 2;
  
  OLED_ShowChinese(x1 + 0*CHINESE_SZ, y1, 0, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x1 + 1*CHINESE_SZ, y1, 1, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x1 + 2*CHINESE_SZ, y1, 2, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x1 + 3*CHINESE_SZ, y1, 3, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x1 + 4*CHINESE_SZ, y1, 4, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x1 + 5*CHINESE_SZ, y1, 5, CHINESE_SZ, Hzk6);

  const uint8_t ch2_num = 6;
  const uint16_t w2_ch = ch2_num * CHINESE_SZ;
  const uint8_t x2 = (128 - (w2_ch + 6*2 + 2)) / 2;
  
  OLED_ShowChinese(x2 + 0*CHINESE_SZ, y2, 6, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x2 + 1*CHINESE_SZ, y2, 7, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x2 + 2*CHINESE_SZ, y2, 8, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x2 + 3*CHINESE_SZ, y2, 9, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x2 + 4*CHINESE_SZ, y2, 10, CHINESE_SZ, Hzk6);
  OLED_ShowChinese(x2 + 5*CHINESE_SZ, y2, 11, CHINESE_SZ, Hzk6);

  const uint8_t ascii_y = y2 + (CHINESE_SZ - 12)/2;
  const uint8_t semicolon_x = x2 + w2_ch;
  const uint8_t bracket_x = semicolon_x + 6 + 2;
  OLED_ShowChar(semicolon_x, ascii_y, ';', 12);
  OLED_ShowChar(bracket_x, ascii_y, ')', 12);
}

void Show_Page2() {
  const uint8_t hrv_y = 10;
  const uint8_t ch_y1 = 30;
  const uint8_t total_char = 4 + 3;
  const uint16_t total_w = total_char * 8;
  const uint8_t start_x = (128 - total_w) / 2;

  OLED_ShowString(start_x, hrv_y, "HRV:", ASCII_1608_SIZE);
  const uint8_t num_x = start_x + 4*8;
  OLED_ShowNum(num_x, hrv_y, current_hrv, 3, ASCII_1608_SIZE);

  const uint8_t ch1_num = 5;
  const uint16_t ch1_w = ch1_num * CHINESE_SZ;
  const uint8_t ch1_x = (128 - ch1_w) / 2;
  
  OLED_ShowChinese(ch1_x + 0*CHINESE_SZ, ch_y1, 0, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch1_x + 1*CHINESE_SZ, ch_y1, 1, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch1_x + 2*CHINESE_SZ, ch_y1, 2, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch1_x + 3*CHINESE_SZ, ch_y1, 3, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch1_x + 4*CHINESE_SZ, ch_y1, 4, CHINESE_SZ, Hzk7);

  const uint8_t ch2_num = 6;
  const uint16_t ch2_w = ch2_num * CHINESE_SZ;
  const uint8_t ch2_x = (128 - ch2_w) / 2;
  
  OLED_ShowChinese(ch2_x + 0*CHINESE_SZ, ch_y1 + CHINESE_SZ, 5, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch2_x + 1*CHINESE_SZ, ch_y1 + CHINESE_SZ, 6, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch2_x + 2*CHINESE_SZ, ch_y1 + CHINESE_SZ, 7, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch2_x + 3*CHINESE_SZ, ch_y1 + CHINESE_SZ, 8, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch2_x + 4*CHINESE_SZ, ch_y1 + CHINESE_SZ, 9, CHINESE_SZ, Hzk7);
  OLED_ShowChinese(ch2_x + 5*CHINESE_SZ, ch_y1 + CHINESE_SZ, 10, CHINESE_SZ, Hzk7);
}

// -------------------------- 13. OLED核心显示函数 --------------------------
// 16×16汉字显示
void OLED_ShowChinese(uint8_t x, uint8_t y, const uint8_t num, uint8_t sz, const uint8_t (*font)[16]) {
  if (sz != 16) return;
  uint8_t col, bit, row_idx;
  uint8_t temp, start_row = num * 2;

  for (row_idx = 0; row_idx < 2; row_idx++) {
    for (col = 0; col < 16; col++) {
      temp = (start_row + row_idx < 32) ? pgm_read_byte(&font[start_row + row_idx][col]) : 0x00;
      for (bit = 0; bit < 8; bit++) {
        const uint8_t cx = x + col;
        const uint8_t cy = y + (row_idx * 8) + bit;
        if (cx <= 127 && cy <= 63) {
          (temp & (1 << bit)) ? OLED_DrawPoint(cx, cy) : OLED_ClearPoint(cx, cy);
        }
      }
    }
  }
}

// 字符显示（支持12/16号字体）
void OLED_ShowChar(uint8_t x, uint8_t y, const char chr, uint8_t size) {
  uint8_t i, m, temp, byte_cnt;
  uint8_t y0 = y;
  const uint8_t char_off = chr - ' ';

  byte_cnt = (size == 12) ? 12 : (size == 16) ? 16 : 0;
  if (byte_cnt == 0) return;

  for (i = 0; i < byte_cnt; i++) {
    temp = (size == 12) ? pgm_read_byte(&asc2_1206[char_off][i]) : pgm_read_byte(&asc2_1608[char_off][i]);
    for (m = 0; m < 8; m++) {
      if (temp & 0x80) OLED_DrawPoint(x, y);
      else OLED_ClearPoint(x, y);
      temp <<= 1;
      y++;
      if ((y - y0) >= size) {
        y = y0;
        x++;
        break;
      }
    }
  }
}

// 字符串显示
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size) {
  while ((*str >= ' ') && (*str <= '~')) {
    OLED_ShowChar(x, y, *str, size);
    x += size / 2;
    if (x > 128 - (size / 2)) {
      x = 0;
      y += size;
    }
    str++;
  }
}

// 数字显示
void OLED_ShowNum(uint8_t x, uint8_t y, int num, uint8_t len, uint8_t size1) {
  uint8_t t, temp;
  if (num > OLED_Pow(10, len) - 1) num = OLED_Pow(10, len) - 1;
  if (num < 0) num = 0;

  for (t = 0; t < len; t++) {
    temp = (num / OLED_Pow(10, len - t - 1)) % 10;
    OLED_ShowChar(x + (size1 / 2) * t, y, temp + '0', size1);
  }
}

// 幂运算（数字显示用）
uint32_t OLED_Pow(uint8_t m, uint8_t n) {
  uint32_t result = 1;
  while (n--) result *= m;
  return result;
}

// -------------------------- 14. OLED底层驱动函数 --------------------------
// 画点（亮）
void OLED_DrawPoint(uint8_t x, uint8_t y) {
  if (x > 127 || y > 63) return;
  const uint8_t page = y / 8;
  const uint8_t bit = y % 8;
  OLED_GRAM[x][page] |= (1 << bit);
}

// 清点（暗）
void OLED_ClearPoint(uint8_t x, uint8_t y) {
  if (x > 127 || y > 63) return;
  const uint8_t page = y / 8;
  const uint8_t bit = y % 8;
  OLED_GRAM[x][page] &= ~(1 << bit);
}

// I2C起始信号
void I2C_Start(void) {
  OLED_SDIN_Set();
  OLED_SCLK_Set();
  delayMicroseconds(2);
  OLED_SDIN_Clr();
  delayMicroseconds(2);
  OLED_SCLK_Clr();
}

// I2C结束信号
void I2C_Stop(void) {
  OLED_SCLK_Clr();
  OLED_SDIN_Clr();
  delayMicroseconds(2);
  OLED_SCLK_Set();
  delayMicroseconds(2);
  OLED_SDIN_Set();
  delayMicroseconds(2);
}

// I2C等待响应
void I2C_WaitAck(void) {
  OLED_SDIN_Set();
  delayMicroseconds(1);
  OLED_SCLK_Set();
  delayMicroseconds(1);
  OLED_SCLK_Clr();
  delayMicroseconds(1);
}

// I2C发送1字节
void Send_Byte(uint8_t dat) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    OLED_SCLK_Clr();
    delayMicroseconds(1);
    (dat & 0x80) ? OLED_SDIN_Set() : OLED_SDIN_Clr();
    delayMicroseconds(1);
    OLED_SCLK_Set();
    delayMicroseconds(1);
    OLED_SCLK_Clr();
    dat <<= 1;
  }
}

// 向OLED写入1字节（命令/数据）
void OLED_WR_Byte(uint8_t dat, uint8_t mode) {
  I2C_Start();
  Send_Byte(0x78);  // OLED地址（0x3C的7位地址左移1位）
  I2C_WaitAck();
  Send_Byte(mode ? 0x40 : 0x00);  // 数据/命令标志
  I2C_WaitAck();
  Send_Byte(dat);
  I2C_WaitAck();
  I2C_Stop();
  delayMicroseconds(2);
}

// 刷新显存到屏幕
void OLED_Refresh(void) {
  uint8_t page, col;
  for (page = 0; page < 8; page++) {
    OLED_WR_Byte(0xB0 + page, OLED_CMD);  // 页地址
    OLED_WR_Byte(0x00, OLED_CMD);         // 列低4位
    OLED_WR_Byte(0x10, OLED_CMD);         // 列高4位
    for (col = 0; col < 128; col++) {
      OLED_WR_Byte(OLED_GRAM[col][page], OLED_DATA);
    }
  }
}

// 清屏（全屏变黑）
void OLED_Clear(void) {
  uint8_t page, col;
  for (page = 0; page < 8; page++) {
    for (col = 0; col < 128; col++) {
      OLED_GRAM[col][page] = 0x00;
    }
  }
}

// OLED初始化（SSD1306芯片）
void OLED_Init(void) {
  pinMode(scl, OUTPUT);
  pinMode(sda, OUTPUT);
  pinMode(res, OUTPUT);
  pinMode(INPUT_PIN, INPUT);

  // 复位时序
  OLED_RST_Set(); delay(100);
  OLED_RST_Clr(); delay(200);
  OLED_RST_Set(); delay(100);

  // 初始化命令
  OLED_WR_Byte(0xAE, OLED_CMD);  // 关闭显示
  OLED_WR_Byte(0x00, OLED_CMD);  // 列地址低4位
  OLED_WR_Byte(0x10, OLED_CMD);  // 列地址高4位
  OLED_WR_Byte(0x40, OLED_CMD);  // 起始行
  OLED_WR_Byte(0x81, OLED_CMD);  // 对比度设置
  OLED_WR_Byte(0xCF, OLED_CMD);  // 对比度值
  OLED_WR_Byte(0xA1, OLED_CMD);  // 段重映射
  OLED_WR_Byte(0xC8, OLED_CMD);  // 行重映射
  OLED_WR_Byte(0xA6, OLED_CMD);  // 正常显示
  OLED_WR_Byte(0xA8, OLED_CMD);  // 多路复用率
  OLED_WR_Byte(0x3F, OLED_CMD);  // 64行
  OLED_WR_Byte(0xD3, OLED_CMD);  // 显示偏移
  OLED_WR_Byte(0x00, OLED_CMD);  // 无偏移
  OLED_WR_Byte(0xD5, OLED_CMD);  // 时钟分频
  OLED_WR_Byte(0x80, OLED_CMD);  // 分频系数
  OLED_WR_Byte(0xD9, OLED_CMD);  // 预充电周期
  OLED_WR_Byte(0xF1, OLED_CMD);  // 预充电参数
  OLED_WR_Byte(0xDA, OLED_CMD);  // COM配置
  OLED_WR_Byte(0x12, OLED_CMD);  // 硬件配置
  OLED_WR_Byte(0xDB, OLED_CMD);  // VCOMH设置
  OLED_WR_Byte(0x40, OLED_CMD);  // VCOMH电压
  OLED_WR_Byte(0x20, OLED_CMD);  // 寻址模式
  OLED_WR_Byte(0x02, OLED_CMD);  // 页寻址
  OLED_WR_Byte(0x8D, OLED_CMD);  // 电荷泵
  OLED_WR_Byte(0x14, OLED_CMD);  // 开启电荷泵
  OLED_WR_Byte(0xAF, OLED_CMD);  // 开启显示
  delay(100);
}
