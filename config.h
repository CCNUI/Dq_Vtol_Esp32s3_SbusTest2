#pragma once

// ======================================================================
// ==                           DEBUGGING
// ======================================================================

// 启用固定的 PWM 输出进行调试
// 0 = 禁用 (PID 正常运行)
// 1 = 启用 (S5, S6, S7 将被覆盖为固定值)
#define DEBUG_FIXED_PWM_OUTPUT 0

// ======================================================================
// ==                    BOARD LED CONFIGURATION
// ======================================================================
#define LED_SIG_BRIGHTNESS 15
#define LED_MAX_BRIGHTNESS 15

// ======================================================================
// ==                      PIN CONFIGURATION
// ======================================================================
#define NUM_SERVOS 8

// ADC 反馈引脚
const int ADC_PINS[NUM_SERVOS] = {
  1, 2, 4, 5, 6, 7, 8, 9
};
/*
 * * S0 -> GPIO 1
 * S1 -> GPIO 2
 * S2 -> GPIO 4
 * S3 -> GPIO 5
 * S4 -> GPIO 6
 * S5 -> GPIO 7
 * S6 -> GPIO 8
 * S7 -> GPIO 9
 */

// PWM 输出引脚
const int PWM_PINS[NUM_SERVOS] = {
  42, 41, 40, 39, 38, 37, 36, 35
};

// SBUS TX 引脚
#define SBUS_PIN 14

// 板载 NeoPixel LED (ESP32-S3 DevKitC-1 通常是 GPIO 48)
#define NEOPIXEL_PIN 48

// ======================================================================
// ==                 SERVO & PID CONFIGURATION
// ======================================================================

// --- [START] 已修改: 频率匹配 ---
// 1kHz (1000µs) 环路
// #define PID_LOOP_PERIOD_US 1000
// 200Hz (5000µs) 环路 (匹配 200Hz PWM)
#define PID_LOOP_PERIOD_US 5000
// --- [END] 已修改: 频率匹配 ---

// PID 输出限制。舵机速度范围为 1000-1400 (反) 和 1600-2000 (正).
// 1500 是中点。我们设置PID输出为 +/- 500.
// (1500 - 500 = 1000)
// (1500 + 500 = 2000)
#define PID_OUTPUT_LIMIT 500.0

// 舵机死区 (1401-1599). 如果PID输出的绝对值小于此值，则强制为0.
// (1600 - 1500 = 100)
#define PID_VELOCITY_DEADZONE 50.0

// --- [START] 新增: 摆动锁定 ---
// 接收 'T' 命令后，PID速度输出改变符号N次后，将PWM锁定在1500us
// (N = 摆动次数)
#define PID_OSCILLATION_LIMIT 10
// --- [END] 新增: 摆动锁定 ---

// 默认PID增益 (将在自动调参时被覆盖)
#define DEFAULT_KP 4.0
#define DEFAULT_KI 0.0
#define DEFAULT_KD 0.005
//P 2 1 0 0.01
//P 2 1 0.0005 0.01

// --- [START] 新增舵机反向设置 ---
// 0 = QuickPID::Action::direct (正向): TGT > ADC 时, PWM > 1500
// 1 = QuickPID::Action::reverse (反向): TGT > ADC 时, PWM < 1500

const uint8_t g_pid_reverse_action[NUM_SERVOS] = {
  0,  // 舵机 0 (正向)
  0,  // 舵机 1 (正向)
  1,  // 舵机 2 (反向)
  0,  // 舵机 3 (正向)
  0,  // 舵机 4 (正向)
  0,  // 舵机 5 (正向)
  0,  // 舵机 6 (正向)
  0   // 舵机 7 (正向)
};
// --- [END] 新增舵机反向设置 ---

// ----------------------------------------------------------------------
// --            !! 用户必须手动校准这些值 !!
// ----------------------------------------------------------------------

// 舵机正常工作范围 (例如 0° 和 90° 对应的ADC读数)
// 仅用于参考或未来功能，PID环路使用下面的极限值。
struct ServoWorkingRange {
  int min_adc;  // 0° 对应的ADC值
  int max_adc;  // 90° 对应的ADC值
};

const ServoWorkingRange g_servo_working_range[NUM_SERVOS] = {
  // {min, max}
  { 1000, 3000 },  // 舵机 0
  { 1000, 3000 },  // 舵机 1
  { 1500, 2400 },  // 舵机 2
  { 1000, 3000 },  // 舵机 3
  { 1000, 3000 },  // 舵机 4
  { 1000, 3000 },  // 舵机 5
  { 1000, 3000 },  // 舵机 6
  { 1000, 3000 },  // 舵机 7
};

// 舵机极限限位ADC范围 (硬停止)
// PID环路将防止舵机移出此范围
struct ServoLimitRange {
  int min_adc;    // 机械最小极限
  int max_adc;    // 机械最大极限
  int center_adc; // 用于自动调参的目标点
};

const ServoLimitRange g_servo_limits[NUM_SERVOS] = {
  // {min, max, center}
  { 500, 3500, 2000 },   // 舵机 0
  { 500, 3500, 2000 },   // 舵机 1
  { 1400, 2450, 2000 },  // 舵机 2
  { 500, 3500, 2000 },   // 舵机 3
  { 500, 3500, 2000 },   // 舵机 4
  { 500, 3500, 2000 },   // 舵机 5
  { 500, 3500, 2000 },   // 舵机 6
  { 500, 3500, 2000 },   // 舵机 7
};


// ======================================================================
// ==                      COMMUNICATION
// ======================================================================
// USB 串行波特率
#define SERIAL_BAUD 115200

// 遥测数据输出频率 (Hz)
#define TELEMETRY_OUTPUT_HZ 10

// SBUS 设置
#define SBUS_BAUD 100000
// *** 已修复: 移除 Core 3.x 中未定义的反相宏 ***
#define SBUS_CONFIG (SERIAL_8E2)  // 8E2
#define SBUS_FRAME_PERIOD_MS 10   // SBUS 帧周期
#define SBUS_MIN_VAL 172
#define SBUS_MAX_VAL 1811
#define SBUS_CENTER_VAL 992