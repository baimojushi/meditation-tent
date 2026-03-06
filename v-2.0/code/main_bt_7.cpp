/**
 * ESP32 毫米波雷达解析器 v4
 * 算法：差分自相关 + 窄带BP + Hilbert CV
 * 新增：按键标注模块（吸气末端手动标记）+ 置信度优化
 *
 * ══════════════════════════════════════════════════════════════
 * 本次测试分析结论（234s，2342条有效JSON）
 * ══════════════════════════════════════════════════════════════
 *
 * 【br 精度】
 *   稳定段（t=200~215s）：br=14.4~14.5bpm，CV=0.22，amp=1.7rad → 可信
 *   整体 9~20bpm 占比 83%，>30bpm 异常占 9%（运动高频干扰误判）
 *
 * 【CV 指标再评估：CV 不等于干扰量】
 *   CV 高 有两种来源，互不可分：
 *     ① 真实身体运动干扰（BP 幅度非周期性波动）
 *     ② ACF 自身估频错误（BP 中心对错频率 → 真实呼吸被滤除 → amp≈0 → CV 虚高）
 *   实测反例：t=0~15s，raw_range=2.75rad（几乎不动），但CV=0.72
 *     原因：窗口初始填充期 ACF 估出 12.6bpm，BP 随即以此为中心，
 *           若真实呼吸在此频率但初始相位不稳，CV 仍高。
 *   实测正例：t=205~215s，raw_range=6.44rad（有晃动），但CV=0.22
 *     原因：ACF 稳定估到 14.4bpm，BP 中心对准，窗口内幅度均匀。
 *   结论：CV 是「ACF 与 BP 是否自洽」的指标，不是独立干扰传感器。
 *         只有当 ACF 本身可靠（conf_acf > 0.2）时，低 CV 才有意义。
 *
 * 【v4 综合可信度：amp_conf_score】
 *   score = amp × conf_acf
 *   物理意义：呼吸信号能量 × ACF 频率可靠性 的乘积
 *   实测分布：
 *     score > 0.3：t=180~215s 连续出现，br 稳定在 14~20bpm，CV<0.45 ← 最可信
 *     score < 0.05：窗口初期或高频干扰段 ← 静默不输出
 *   与 CV 联合：score > 0.1 AND cv < 0.55 → 输出；否则置 br=0
 *
 * ══════════════════════════════════════════════════════════════
 * 按键标注模块
 * ══════════════════════════════════════════════════════════════
 *
 * 【设计目标】
 *   提供精准的呼吸周期时间戳，作为雷达算法的独立参考基准，
 *   用于评估 br_radar 的绝对误差，并为后续 LSTM 训练提供标注数据。
 *
 * 【操作方法】
 *   在每次吸气到达最深处（吸气末端/呼气开始）时按下按键。
 *   按键时刻记录为一个完整呼吸周期的"峰值时间戳"。
 *   相邻两次按键的间隔 = 一个呼吸周期 T = 60/br_key (s)
 *
 * 【为什么用吸气末端而不是呼气末端】
 *   吸气末端有明确的「停顿感」（胸部充气最高点），时机更一致。
 *   呼气末端（FRC）随肌肉放松渐进，时刻感不如前者精确。
 *   注意：两者均可，只要全程统一即可。
 *
 * 【按键 br 计算算法】
 *   维护最近 8 次按键时间戳（毫秒精度），计算相邻间隔。
 *   使用中位数间隔（而非均值）抗偶发漏按或双击。
 *   br_key = 60000.0 / median(intervals) (bpm)
 *
 * 【Rdd 曲线拟合（LSTM 标注用）】
 *   每次按键后，以按键时间戳重建理论呼吸波形（正弦参考），
 *   与实测差分自相关 Rdd 曲线做对比，计算最优 lag 误差，
 *   输出 lag_error 供后续分析。
 *
 * ══════════════════════════════════════════════════════════════
 * JSON 输出格式
 * ══════════════════════════════════════════════════════════════
 *
 * 雷达帧（10Hz）：
 *   {"t":12345,"raw":1.23,"br":12.5,"conf":0.35,"amp":1.2,"cv":0.45,"score":0.42}
 *
 * 按键事件（即时，仅在按键时发送）：
 *   {"evt":"key","t":12345,"br_key":13.2,"n_keys":5,"lag_err":-0.8}
 *   lag_err = (ACF估计lag - 按键推算lag) / 按键推算lag × 100 (%)
 *
 * 字段说明：
 *   t        : millis() 时间戳 (ms)
 *   raw      : 原始相位 (rad)
 *   br       : 雷达估计速率 (bpm，0=不可信)
 *   conf     : ACF主峰归一化高度 [0,1]
 *   amp      : Hilbert瞬时幅度均值 (rad)
 *   cv       : 幅度变异系数（<0.45=稳定，>0.7=干扰或估频偏离）
 *   score    : amp×conf 综合置信度（>0.1 输出，>0.3 高可信）
 *   br_key   : 按键计算呼吸率 (bpm)
 *   lag_err  : ACF lag 与按键参考的相对误差 (%)
 *
 * ══════════════════════════════════════════════════════════════
 * LSTM 标注可行性评估
 * ══════════════════════════════════════════════════════════════
 *
 * 可用性：是，作为弱监督标注（不是精确样本级标注）
 *
 * 标注粒度：每个呼吸周期 1 个时间戳 → 窗口级标签（5~10s 分辨率）
 *   适合：时序分类（正常/干扰）、呼吸速率回归（每窗口 1 个 bpm 标签）
 *   不适合：逐样本（50Hz 帧级）波形重建
 *
 * 标注质量风险：
 *   ① 人工标注抖动：按键时机误差 ±0.3s → 对应 br 误差 ±1bpm（可接受）
 *   ② 遗漏按键：需在 JSON 中标记 gap（n_keys 不连续时置 br_key=0）
 *   ③ 与雷达的时间同步：两路数据共用 millis()，无需外部对时
 *
 * 采集流程建议：
 *   1. 静坐 2min 正常呼吸（基准段）
 *   2. 身体轻微晃动 1min（轻度干扰）
 *   3. 手臂大幅运动 1min（强干扰）
 *   4. 起立/坐下各 30s（姿态变化）
 *   全程按键，每段至少 10 个周期（约 80~120s @12bpm）
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled!"
#endif

// ==================== 硬件配置 ====================
#define BT_DEVICE_NAME  "ESP32-Radar-BT"
#define RADAR_RX_PIN    16
#define RADAR_TX_PIN    4
#define RADAR_BAUD      1382400UL
#define BTN_PIN         17       // GPIO0 = BOOT按钮（内置，常开，按下=LOW）
                                // 可改为任意带内部上拉的GPIO，如GPIO34不可用上拉
#define BTN_DEBOUNCE_MS 80      // 防抖时间
#define SEND_MS         100     // JSON 输出间隔 (10Hz)

// ==================== 滤波参数 ====================
#define LP_DENOISE_A    0.2f    // 一阶指数移动平均系数
#define RADAR_FS        50.0f

// ACF 窗口
#define ACF_WIN         700
#define ACF_LAG_MIN     100
#define ACF_LAG_MAX     375
#define ACF_UPDATE_N    250
#define ACF_CONF_MIN    0.06f   // 略降低阈值，让 score 来过滤

// Hilbert
#define HIL_BP_TAP      63
#define HIL_H_TAP       63

// 综合置信度阈值（v4 新增）
#define SCORE_MIN       0.04f   // amp×conf 最低阈值
#define CV_MAX          0.70f   // CV 上限（用于最终静默判断）

// 伪影
#define ART_DIFF        1.0f
#define ART_LOCKOUT     20

// ==================== 按键标注 ====================
#define KEY_BUF_SIZE    8       // 记录最近 N 个按键时间戳
#define KEY_MIN_INTERVAL_MS  3000  // 最短按键间隔（防止连击）40bpm=1.5s
#define KEY_MAX_INTERVAL_MS  6000  // 最长合法间隔（9bpm=6.67s）

// ==================== TF 协议 ====================
#define TF_SOF          0x01
#define TF_TYPE_HUMAN   0x0F09
#define TF_TYPE_PHASE   0x0A13

typedef enum {
    ST_SOF, ST_ID_H, ST_ID_L, ST_LEN_H, ST_LEN_L,
    ST_TYPE_H, ST_TYPE_L, ST_HEAD_CKSUM, ST_DATA, ST_DATA_CKSUM
} TF_State;

// ==================== 全局状态 ====================
static BluetoothSerial BT;
static volatile bool   spp_ready = false;

// TF 解析
static TF_State tf_state = ST_SOF;
static uint16_t tf_id, tf_len, tf_type;
static uint8_t  tf_data[64] = {};
static uint16_t tf_data_idx = 0;
static uint8_t  xor_acc     = 0;

// 雷达输出（volatile：跨任务读写）
static volatile float v_raw      = 0.0f;
static volatile float v_br       = 0.0f;
static volatile float v_conf     = 0.0f;
static volatile float v_amp      = 0.0f;
static volatile float v_cv       = 0.0f;
static volatile float v_score    = 0.0f;  // = amp × conf（v4 新增）
static volatile float v_lp       = 0.0f;  // LP 滤波后相位（调优用）
static volatile float v_hil_ph   = 0.0f;  // 窄带 BP 瞬时相位 atan2(hil,bp)（调优用）
static volatile bool  v_human    = false;

// 降噪 LPF
static float lp_y = 0.0f, lp_prev = 0.0f;
static bool  lp_init = false, lp_prev_init = false;

// ACF 环形缓冲（动态分配）
static float*    acf_buf        = nullptr;
static uint16_t  acf_wr         = 0;
static uint16_t  acf_cnt        = 0;
static uint16_t  acf_since_upd  = 0;

// Hilbert 带通系数
static float    bp_h[HIL_BP_TAP]  = {};
static float    hil_h[HIL_H_TAP]  = {};
static bool     hil_ready = false;
static float    last_f0   = -1.0f;

// 伪影
static uint16_t art_lockout = 0;

// ACF 最新 Rdd 数组（供按键模块对比用）
static float    rdd_snapshot[ACF_LAG_MAX + 1] = {};  // 每次 ACF 计算后快照
static float    rdd_best_lag_exact = 0.0f;           // 抛物线精确 lag

// 频率连续性追踪（v5 新增）
static float    br_stable     = 0.0f;   // 平滑输出频率（指数滑动）
static float    br_last_raw   = 0.0f;   // 上次ACF原始估计，用于连续性判断

// 频率变化方向追踪（v6 新增）
// 连续同向变化超过阈值次数时，直接重置 br_stable，抑制高bpm滞留
static int8_t   freq_dir_count = 0;    // 正=连续加速次数，负=连续减速次数

// 按键标注
static uint32_t key_timestamps[KEY_BUF_SIZE] = {};  // 按键时刻 ms
static uint8_t  key_head   = 0;   // 环形写指针
static uint8_t  key_count  = 0;   // 有效数量
static float    br_key     = 0.0f;
static float    lag_err_pct = 0.0f;
static volatile bool key_event = false;  // 按键事件标志（发送任务消费）
static uint32_t last_key_ms = 0;

// 工作数组（动态分配）
static float* s_diff    = nullptr;
static float* s_lp_lin  = nullptr;
static float* s_bp_out  = nullptr;
static float* s_hil_out = nullptr;

// ==================== 工具函数 ====================
static float bytes_to_float(const uint8_t* b) {
    uint32_t u = (uint32_t)b[0] | ((uint32_t)b[1]<<8)
               | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
    float f; memcpy(&f, &u, 4); return f;
}
static void tf_reset() { tf_state = ST_SOF; tf_data_idx = 0; xor_acc = 0; }
static float hann(int i, int N) {
    return 0.5f - 0.5f * cosf((float)M_PI * 2.0f * i / (N - 1));
}

// ==================== 按键 br 计算 ====================
static void compute_key_br() {
    if (key_count < 2) { br_key = 0.0f; return; }

    // 提取有效间隔
    float intervals[KEY_BUF_SIZE - 1];
    int n_int = 0;
    uint8_t n = min((int)key_count, KEY_BUF_SIZE);
    for (int i = 1; i < n; i++) {
        uint8_t a = (key_head - n + i - 1 + KEY_BUF_SIZE) % KEY_BUF_SIZE;
        uint8_t b = (key_head - n + i     + KEY_BUF_SIZE) % KEY_BUF_SIZE;
        float gap = (float)(key_timestamps[b] - key_timestamps[a]);
        if (gap >= KEY_MIN_INTERVAL_MS && gap <= KEY_MAX_INTERVAL_MS) {
            intervals[n_int++] = gap;
        }
    }
    if (n_int < 1) { br_key = 0.0f; return; }

    // 中位数间隔
    // 简单排序（n_int 最大 7，开销可忽略）
    for (int i = 0; i < n_int-1; i++)
        for (int j = 0; j < n_int-i-1; j++)
            if (intervals[j] > intervals[j+1]) {
                float t = intervals[j]; intervals[j] = intervals[j+1]; intervals[j+1] = t;
            }
    float median_gap = intervals[n_int / 2];
    br_key = 60000.0f / median_gap;

    // 计算 ACF lag 与按键参考的误差
    // 按键对应的真实 lag（全周期）：key_lag = median_gap_ms / 1000 × RADAR_FS
    float key_lag = (median_gap / 1000.0f) * RADAR_FS;
    if (rdd_best_lag_exact > 0.0f) {
        lag_err_pct = (rdd_best_lag_exact - key_lag) / key_lag * 100.0f;
    } else {
        lag_err_pct = 0.0f;
    }
}

// ==================== 按键 ISR（在任务中轮询，防 ISR 延时问题）====================
// 由 uart_task 在轮询中调用（50Hz 足够检测 80ms 按键）
static void check_button() {
    static bool last_state = HIGH;
    static uint32_t debounce_start = 0;

    bool cur = digitalRead(BTN_PIN);
    uint32_t now = millis();

    if (cur == LOW && last_state == HIGH) {
        // 下降沿检测
        if (now - last_key_ms >= KEY_MIN_INTERVAL_MS) {
            if (now - debounce_start >= BTN_DEBOUNCE_MS) {
                // 有效按键
                key_timestamps[key_head] = now;
                key_head = (key_head + 1) % KEY_BUF_SIZE;
                if (key_count < KEY_BUF_SIZE) key_count++;
                last_key_ms = now;
                compute_key_br();
                key_event = true;  // 通知发送任务
                Serial.printf("[KEY] t=%u br_key=%.1f lag_err=%.1f%%\n", now, br_key, lag_err_pct);
            }
        }
        debounce_start = now;
    }
    last_state = cur;
}

// ==================== 带通 FIR 构建 ====================
static void build_bp(float f0_hz) {
    if (fabsf(f0_hz - last_f0) < 0.005f && hil_ready) return;
    last_f0 = f0_hz;
    float fl = f0_hz - 0.08f; if (fl < 9.0f/60.0f)  fl = 9.0f/60.0f;
    float fh = f0_hz + 0.08f; if (fh > 50.0f/60.0f) fh = 50.0f/60.0f;
    int half = HIL_BP_TAP / 2;
    for (int i = 0; i < HIL_BP_TAP; i++) {
        int n = i - half;
        float v = (n == 0) ? 2.0f*(fh-fl)/RADAR_FS
                           : (sinf(2.0f*(float)M_PI*fh/RADAR_FS*n)
                             -sinf(2.0f*(float)M_PI*fl/RADAR_FS*n))
                             /((float)M_PI*n);
        bp_h[i] = v * hann(i, HIL_BP_TAP);
    }
    int hh = HIL_H_TAP / 2;
    for (int i = 0; i < HIL_H_TAP; i++) {
        int n = i - hh;
        float v = (n != 0 && n%2 == 1) ? 2.0f/((float)M_PI*n) : 0.0f;
        hil_h[i] = v * hann(i, HIL_H_TAP);
    }
    hil_ready = true;
}

// ==================== ACF + Hilbert 计算 ====================
static void update_rate() {
    if (!s_diff) return;

    // ── 差分自相关 ──────────────────────────────────────
    const int M = ACF_WIN - 1;
    for (int i = 0; i < M; i++) {
        float cur  = acf_buf[(acf_wr + i + 1) % ACF_WIN];
        float prev = acf_buf[(acf_wr + i)     % ACF_WIN];
        s_diff[i] = cur - prev;
    }
    float r0 = 0.0f;
    for (int i = 0; i < M; i++) r0 += s_diff[i] * s_diff[i];
    if (r0 < 1e-6f) { v_br = 0; v_conf = 0; v_score = 0; return; }

    float best_r = -1.0f; int best_k = ACF_LAG_MIN;

    // ── 多候选 lag：保留 top-3 局部极大值 ─────────────────────────────
    struct Cand { int k; float r; };
    Cand top[3] = {{ACF_LAG_MIN,0.0f},{ACF_LAG_MIN,0.0f},{ACF_LAG_MIN,0.0f}};

    for (int k = ACF_LAG_MIN; k <= ACF_LAG_MAX; k++) {
        float rk = 0.0f;
        int n = M - k;
        for (int i = 0; i < n; i++) rk += s_diff[i] * s_diff[i+k];
        float rkn = rk / r0;
        rdd_snapshot[k] = rkn;    // 快照供按键模块使用
        if (rkn > best_r) { best_r = rkn; best_k = k; }
    }
    top[0] = {best_k, best_r};

    // 找 top-2/3 局部极大值（与 best_k 距离 >20，避免选同一峰的旁瓣）
    float r1 = -1.0f, r2 = -1.0f;
    int   k1 = best_k, k2 = best_k;
    for (int k = ACF_LAG_MIN + 1; k < ACF_LAG_MAX; k++) {
        if (abs(k - best_k) <= 20) continue;
        if (rdd_snapshot[k] <= rdd_snapshot[k-1] || rdd_snapshot[k] <= rdd_snapshot[k+1]) continue;
        if (rdd_snapshot[k] > r1) {
            if (abs(k1 - best_k) > 20) { r2 = r1; k2 = k1; }
            r1 = rdd_snapshot[k]; k1 = k;
        } else if (rdd_snapshot[k] > r2 && abs(k - k1) > 20) {
            r2 = rdd_snapshot[k]; k2 = k;
        }
    }
    top[1] = {k1, r1};
    top[2] = {k2, r2};

    v_conf = best_r;
    if (best_r < ACF_CONF_MIN) { v_br = 0; v_score = 0; return; }

    // ── 谐波检测：lag×2 在范围内才检测 ───────────────────────────────
    {
        int lag_half = best_k * 2;
        if (lag_half <= ACF_LAG_MAX) {
            float rdd_half = rdd_snapshot[lag_half];
            if (rdd_half > rdd_snapshot[best_k] * 0.40f) {
                best_k = lag_half;
                best_r = rdd_half;
                v_conf = best_r;
                top[0] = {best_k, best_r};
            }
        }
    }

    // ── 频率连续性约束：过渡期防大幅跳变 ────────────────────────────
    // ACF窗口惯性14s，快速变化时旧频率残留会产生虚假峰，
    // 用上次稳定频率约束，当最优峰跳变>4bpm时优先选更连续的次优候选
    if (br_last_raw > 0.0f) {
        float bpm_best = RADAR_FS / (float)best_k * 60.0f;
        float jump = fabsf(bpm_best - br_last_raw);
        if (jump > 4.0f) {
            int alt_k = best_k; float alt_r = best_r;
            for (int ci = 1; ci < 3; ci++) {
                if (top[ci].r < ACF_CONF_MIN * 0.6f) continue;
                float bpm_ci = RADAR_FS / (float)top[ci].k * 60.0f;
                float jump_ci = fabsf(bpm_ci - br_last_raw);
                if (jump_ci < jump && top[ci].r >= best_r * 0.70f) {
                    alt_k = top[ci].k; alt_r = top[ci].r;
                    jump = jump_ci;
                }
            }
            if (alt_k != best_k) {
                best_k = alt_k; best_r = alt_r;
                v_conf = best_r;
            }
        }
    }

    // ── 抛物线插值（基于最终 best_k）────────────────────────────────
    float y0, y1 = rdd_snapshot[best_k], y2;
    {
        y0 = (best_k > ACF_LAG_MIN) ? rdd_snapshot[best_k-1] : y1;
        y2 = (best_k < ACF_LAG_MAX) ? rdd_snapshot[best_k+1] : y1;
    }
    float pk = (float)best_k;
    float den = y0 - 2.0f*y1 + y2;
    if (fabsf(den) > 1e-6f) pk += 0.5f*(y0-y2)/den;
    rdd_best_lag_exact = pk;   // 供按键模块使用

    float br_hz  = RADAR_FS / pk;
    float br_bpm = br_hz * 60.0f;
    if (br_bpm < 9.0f || br_bpm > 50.0f) { v_br = 0; v_score = 0; return; }

    // ── 窄带 BP → Hilbert → CV ──────────────────────────
    build_bp(br_hz);
    if (!hil_ready) { v_br = br_bpm; return; }

    for (int i = 0; i < ACF_WIN; i++)
        s_lp_lin[i] = acf_buf[(acf_wr + i) % ACF_WIN];

    int half_bp = HIL_BP_TAP / 2;
    for (int i = 0; i < ACF_WIN; i++) {
        float s = 0.0f;
        for (int j = 0; j < HIL_BP_TAP; j++) {
            int idx = i + j - half_bp;
            float xv = (idx < 0) ? s_lp_lin[0]
                      :(idx >= ACF_WIN) ? s_lp_lin[ACF_WIN-1]
                      : s_lp_lin[idx];
            s += bp_h[j] * xv;
        }
        s_bp_out[i] = s;
    }
    int half_h = HIL_H_TAP / 2;
    for (int i = 0; i < ACF_WIN; i++) {
        float s = 0.0f;
        for (int j = 0; j < HIL_H_TAP; j++) {
            int idx = i + j - half_h;
            float xv = (idx < 0) ? s_bp_out[0]
                      :(idx >= ACF_WIN) ? s_bp_out[ACF_WIN-1]
                      : s_bp_out[idx];
            s += hil_h[j] * xv;
        }
        s_hil_out[i] = s;
    }

    // Bug B 修复：trim = HIL_BP_TAP/2 = 31
    int trim = HIL_BP_TAP / 2;

    // 最新有效样本的瞬时相位（调优用，取窗口末端 trim 处）
    {
        int idx_ph = ACF_WIN - trim - 1;
        v_hil_ph = atan2f(s_hil_out[idx_ph], s_bp_out[idx_ph]);
    }
    float sum_a = 0.0f, sum_a2 = 0.0f;
    int cnt_a = 0;
    for (int i = trim; i < ACF_WIN - trim; i++) {
        float a = sqrtf(s_bp_out[i]*s_bp_out[i] + s_hil_out[i]*s_hil_out[i]);
        sum_a  += a;
        sum_a2 += a * a;
        cnt_a++;
    }
    if (cnt_a < 4) { v_br = br_bpm; return; }
    float mean_a = sum_a / cnt_a;
    float var_a  = fmaxf(0.0f, sum_a2/cnt_a - mean_a*mean_a);
    float cv     = sqrtf(var_a) / (mean_a + 1e-6f);
    float score  = mean_a * best_r;   // amp × conf = 综合置信度 (v4)

    v_amp   = mean_a;
    v_cv    = cv;
    v_score = score;

    // ── v6 输出判断 ───────────────────────────────────────────────────────
    // 问题1修复：断档
    //   BP带通中心滞后14s，过渡期 cv 虚高（不是真实干扰），不应静默。
    //   当 ACF 检测到频率跳变 > 3bpm 时，放宽 CV_MAX 从 0.70 → 0.90。
    //   0.90 足以覆盖实测最大虚高值 0.779，同时保留对严重干扰（>0.90）的过滤。
    float cv_thresh = CV_MAX;
    bool in_transition = (br_last_raw > 0.0f && fabsf(br_bpm - br_last_raw) > 3.0f);
    if (in_transition) cv_thresh = 0.90f;

    // score 阈值：过渡期同样放宽，因为 BP 中心偏移导致 amp 降低
    float score_thresh = in_transition ? SCORE_MIN * 0.5f : SCORE_MIN;

    if (score >= score_thresh && cv <= cv_thresh) {
        // 问题2修复：高bpm滞留
        //   连续 ≥3 次 ACF 估计都指向同一方向（br_bpm 持续 > 或 < br_stable），
        //   说明真实频率已经单调变化，立即重置 br_stable 为最新 br_bpm，
        //   放弃惰性平滑，追踪速度从指数收敛（30s+）缩短到 15s（3次×5s）。
        if (br_stable < 1.0f) {
            br_stable = br_bpm;     // 首次初始化
            freq_dir_count = 0;
        } else {
            // 更新方向计数：同向累加，反向归零
            float delta = br_bpm - br_stable;
            if (delta > 1.0f) {
                freq_dir_count = (freq_dir_count > 0) ? freq_dir_count + 1 : 1;
            } else if (delta < -1.0f) {
                freq_dir_count = (freq_dir_count < 0) ? freq_dir_count - 1 : -1;
            } else {
                freq_dir_count = 0;  // 收敛，清零
            }

            if (freq_dir_count >= 3 || freq_dir_count <= -3) {
                // 连续3次同向：直接对齐，不再指数收敛
                br_stable = br_bpm;
                freq_dir_count = 0;
            } else {
                // 普通平滑：快变时 α 大（追踪快），慢变时 α 小（更稳定）
                float jump = fabsf(delta);
                float alpha = (jump > 3.0f) ? 0.55f : 0.30f;
                br_stable = alpha * br_bpm + (1.0f - alpha) * br_stable;
            }
        }
        br_last_raw = br_bpm;
        v_br = br_stable;
    } else {
        v_br = 0.0f;
        // br_last_raw 保留，用于下次连续性判断（不清零）
        freq_dir_count = 0;  // 断档时重置方向计数，避免积累错误趋势
    }
}

// ==================== 样本处理 ====================
static void process_sample(float raw) {
    if (!lp_init) { lp_y = raw; lp_init = true; lp_prev = raw; lp_prev_init = true; return; }
    lp_y = LP_DENOISE_A * raw + (1.0f - LP_DENOISE_A) * lp_y;
    v_lp = lp_y;  // 实时更新滤波后相位

    if (lp_prev_init && fabsf(lp_y - lp_prev) > ART_DIFF) {
        acf_cnt = acf_wr = acf_since_upd = 0;
        art_lockout = ART_LOCKOUT;
        v_br = v_conf = v_amp = v_cv = v_score = 0.0f;
        lp_prev = lp_y; return;
    }
    lp_prev = lp_y;
    if (art_lockout > 0) { art_lockout--; return; }

    if (acf_buf) {
        acf_buf[acf_wr] = lp_y;
        acf_wr = (acf_wr + 1) % ACF_WIN;
        if (acf_cnt < ACF_WIN) acf_cnt++;
        acf_since_upd++;
        if (acf_cnt >= ACF_WIN && acf_since_upd >= ACF_UPDATE_N) {
            acf_since_upd = 0;
            update_rate();
        }
    }
}

// ==================== TF 帧处理 ====================
static void on_frame() {
    switch (tf_type) {
        case TF_TYPE_HUMAN: {
            if (tf_len >= 2) {
                bool det = (tf_data[0] | ((uint16_t)tf_data[1]<<8)) == 0x0001;
                if (!det && v_human) {
                    lp_init = false; lp_prev_init = false; hil_ready = false;
                    acf_cnt = acf_wr = acf_since_upd = art_lockout = 0;
                    v_br = v_conf = v_amp = v_cv = v_score = 0.0f;
                    v_lp = v_hil_ph = 0.0f;
                    br_stable = 0.0f; br_last_raw = 0.0f; freq_dir_count = 0;
                }
                v_human = det;
            }
            break;
        }
        case TF_TYPE_PHASE: {
            if (tf_len >= 4) {
                float raw = bytes_to_float(&tf_data[0]);
                v_raw = raw;
                process_sample(raw);
            }
            break;
        }
    }
}

// ==================== TF 解析 ====================
static void parse_byte(uint8_t b) {
    switch (tf_state) {
        case ST_SOF:       if(b==TF_SOF){xor_acc=b;tf_state=ST_ID_H;} break;
        case ST_ID_H:      xor_acc^=b;tf_id=(uint16_t)b<<8;tf_state=ST_ID_L; break;
        case ST_ID_L:      xor_acc^=b;tf_id|=b;tf_state=ST_LEN_H; break;
        case ST_LEN_H:     xor_acc^=b;tf_len=(uint16_t)b<<8;tf_state=ST_LEN_L; break;
        case ST_LEN_L:
            xor_acc^=b;tf_len|=b;
            if(tf_len>sizeof(tf_data)){tf_reset();break;}
            tf_state=ST_TYPE_H; break;
        case ST_TYPE_H:    xor_acc^=b;tf_type=(uint16_t)b<<8;tf_state=ST_TYPE_L; break;
        case ST_TYPE_L:    xor_acc^=b;tf_type|=b;tf_state=ST_HEAD_CKSUM; break;
        case ST_HEAD_CKSUM:
            if(b!=(uint8_t)(~xor_acc)){tf_reset();break;}
            tf_data_idx=0;xor_acc=0;
            tf_state=(tf_len>0)?ST_DATA:ST_DATA_CKSUM; break;
        case ST_DATA:
            tf_data[tf_data_idx++]=b;xor_acc^=b;
            if(tf_data_idx>=tf_len)tf_state=ST_DATA_CKSUM; break;
        case ST_DATA_CKSUM:
            if(tf_len==0||b==(uint8_t)(~xor_acc))on_frame();
            tf_reset(); break;
    }
}

// ==================== UART 任务（Core 0）====================
static void uart_task(void*) {
    uint8_t buf[512];
    while (true) {
        // 轮询按键（50Hz：每20ms的雷达帧检测一次）
        check_button();

        int n = Serial2.available();
        if (n > 0) {
            n = Serial2.readBytes(buf, min(n, (int)sizeof(buf)));
            for (int i = 0; i < n; i++) parse_byte(buf[i]);
        } else {
            vTaskDelay(1);
        }
    }
}

// ==================== 发送任务（Core 1）====================
static void send_task(void*) {
    char msg[192];
    bool was_ready = false;
    uint32_t last_send = 0;

    while (true) {
        bool ready = spp_ready;
        uint32_t now = millis();

        if (ready && !was_ready) {
            BT.println("{\"info\":\"ESP32 Radar v4 + KeyAnnotation\"}");
            Serial.println("[BT] SPP open");
        } else if (!ready && was_ready) {
            Serial.println("[BT] SPP closed");
        }
        was_ready = ready;

        // ── 按键事件：即时发送 ──────────────────────────
        if (key_event) {
            key_event = false;
            snprintf(msg, sizeof(msg),
                "{\"evt\":\"key\",\"t\":%lu,\"br_key\":%.1f,\"n_keys\":%d,\"lag_err\":%.1f}\n",
                (unsigned long)last_key_ms, br_key, (int)key_count, lag_err_pct);
            if (ready) BT.print(msg);
            Serial.print(msg);
        }

        // ── 定时雷达帧 ───────────────────────────────────
        if (now - last_send >= SEND_MS) {
            last_send = now;
            snprintf(msg, sizeof(msg),
                "{\"t\":%lu,\"raw\":%.3f,\"lp\":%.3f,\"hil_ph\":%.3f,\"br\":%.1f,\"conf\":%.3f,\"amp\":%.3f,\"cv\":%.3f,\"score\":%.3f}\n",
                (unsigned long)now, v_raw, v_lp, v_hil_ph, v_br, v_conf, v_amp, v_cv, v_score);
            if (ready) BT.print(msg);
            Serial.print(msg);
        }
        vTaskDelay(5);
    }
}

// ==================== SPP 回调 ====================
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t*) {
    if      (event == ESP_SPP_SRV_OPEN_EVT) spp_ready = true;
    else if (event == ESP_SPP_CLOSE_EVT)    spp_ready = false;
}

// ==================== setup ====================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== ESP32 Radar v4 | DiffACF + Hilbert + KeyAnnotation ===");
    Serial.printf("  ACF: WIN=%d(%.1fs) LAG=%d~%d(%.1f~%.1fbpm) UPDATE=%d(%.1fs)\n",
        ACF_WIN, (float)ACF_WIN/RADAR_FS,
        ACF_LAG_MIN, ACF_LAG_MAX,
        RADAR_FS/ACF_LAG_MAX*60, RADAR_FS/ACF_LAG_MIN*60,
        ACF_UPDATE_N, (float)ACF_UPDATE_N/RADAR_FS);
    Serial.println("  Reliable: score>0.08 && cv<0.70 && 9<br<50");
    Serial.println("  KEY: press at each inhale peak (吸气末端)");
    Serial.printf("  KEY PIN: GPIO%d (LOW=press)\n", BTN_PIN);

    // 按键 GPIO
    pinMode(BTN_PIN, INPUT_PULLUP);

    // 动态分配缓冲区（共 700×5×4 = 14KB SRAM）
    acf_buf    = (float*)heap_caps_malloc(ACF_WIN * sizeof(float), MALLOC_CAP_8BIT);
    s_diff     = (float*)heap_caps_malloc((ACF_WIN-1) * sizeof(float), MALLOC_CAP_8BIT);
    s_lp_lin   = (float*)heap_caps_malloc(ACF_WIN * sizeof(float), MALLOC_CAP_8BIT);
    s_bp_out   = (float*)heap_caps_malloc(ACF_WIN * sizeof(float), MALLOC_CAP_8BIT);
    s_hil_out  = (float*)heap_caps_malloc(ACF_WIN * sizeof(float), MALLOC_CAP_8BIT);

    if (!acf_buf || !s_diff || !s_lp_lin || !s_bp_out || !s_hil_out) {
        Serial.println("MALLOC FAILED!"); while(1) delay(1000);
    }
    memset(acf_buf, 0, ACF_WIN * sizeof(float));
    memset(rdd_snapshot, 0, sizeof(rdd_snapshot));
    Serial.printf("  Heap free: %u bytes\n", ESP.getFreeHeap());

    // UART
    Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial2.setRxBufferSize(8192);

    // BT
    BT.register_callback(spp_callback);
    if (!BT.begin(BT_DEVICE_NAME)) {
        Serial.println("BT FAILED"); delay(2000); ESP.restart();
    }
    Serial.printf("  BT: %s\n", BT_DEVICE_NAME);

    xTaskCreatePinnedToCore(uart_task, "UART", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(send_task, "Send", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(5000); }
