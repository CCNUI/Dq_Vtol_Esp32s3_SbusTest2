//@FILE config.h START
#pragma once

// ======================================================================
// ==                           DEBUGGING
// ======================================================================

// 启用固定的 PWM 输出进行调试
// 0 = 禁用 (PID 正常运行)
// 1 = 启用 (S6, S7, S8 将被覆盖为固定值)
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
// 数组大小为 9，索引 0 被废弃，使用 1-8
const int ADC_PINS[NUM_SERVOS + 1] = {
  -1, // [0] 废弃/占位
  2,  // [1] Servo 1 -> GPIO 2
  1,  // [2] Servo 2 -> GPIO 1
  4,  // [3] Servo 3 -> GPIO 4
  5,  // [4] Servo 4 -> GPIO 5
  6,  // [5] Servo 5 -> GPIO 6
  7,  // [6] Servo 6 -> GPIO 7
  8,  // [7] Servo 7 -> GPIO 8
  9   // [8] Servo 8 -> GPIO 9
};

// PWM 输出引脚
// 数组大小为 9，索引 0 被废弃，使用 1-8
const int PWM_PINS[NUM_SERVOS + 1] = {
  -1, // [0] 废弃/占位
  42, // [1] Servo 1
  41, // [2] Servo 2
  40, // [3] Servo 3
  39, // [4] Servo 4
  38, // [5] Servo 5
  37, // [6] Servo 6
  36, // [7] Servo 7
  35  // [8] Servo 8
};

// SBUS TX 引脚
#define SBUS_PIN 14

// 板载 NeoPixel LED (ESP32-S3 DevKitC-1 通常是 GPIO 48)
#define NEOPIXEL_PIN 48

// ======================================================================
// ==                 SERVO & PID CONFIGURATION
// ======================================================================

// 200Hz (5000µs) 环路 (匹配 200Hz PWM)
#define PID_LOOP_PERIOD_US 5000

// PID 输出限制 (+/- 500)
#define PID_OUTPUT_LIMIT 500.0

// 舵机死区
#define PID_VELOCITY_DEADZONE 50.0

// 摆动锁定限制
#define PID_OSCILLATION_LIMIT 10

// 默认PID增益
#define DEFAULT_KP 4.0
#define DEFAULT_KI 0.0
#define DEFAULT_KD 0.005

// --- 舵机反向设置 ---
// 数组大小为 9，索引 0 被废弃
// 0 = 正向, 1 = 反向
const uint8_t g_pid_reverse_action[NUM_SERVOS + 1] = {
  0,  // [0] 废弃
  1,  // [1] Servo 1
  0,  // [2] Servo 2
  1,  // [3] Servo 3
  0,  // [4] Servo 4
  0,  // [5] Servo 5
  0,  // [6] Servo 6
  0,  // [7] Servo 7
  0   // [8] Servo 8
};

// ----------------------------------------------------------------------
// --            !! 用户必须手动校准这些值 !!
// ----------------------------------------------------------------------

struct ServoWorkingRange {
  int min_adc;
  int max_adc;
};

// 数组大小为 9，索引 0 被废弃
const ServoWorkingRange g_servo_working_range[NUM_SERVOS + 1] = {
  {0, 0},           // [0] 废弃
  { 1000, 3000 },   // [1] Servo 1
  { 1000, 3000 },   // [2] Servo 2
  { 1500, 2400 },   // [3] Servo 3
  { 1000, 3000 },   // [4] Servo 4
  { 1000, 3000 },   // [5] Servo 5
  { 1000, 3000 },   // [6] Servo 6
  { 1000, 3000 },   // [7] Servo 7
  { 1000, 3000 },   // [8] Servo 8
};

// 舵机极限限位ADC范围 (硬停止)
struct ServoLimitRange {
  int min_adc;
  int max_adc;
  int center_adc;
};

// 数组大小为 9，索引 0 被废弃
const ServoLimitRange g_servo_limits[NUM_SERVOS + 1] = {
  {0, 0, 0},             // [0] 废弃
  { 500, 3500, 2000 },   // [1] Servo 1
  { 500, 3500, 2000 },   // [2] Servo 2
  { 1400, 2450, 2000 },  // [3] Servo 3
  { 500, 3500, 2000 },   // [4] Servo 4
  { 500, 3500, 2000 },   // [5] Servo 5
  { 500, 3500, 2000 },   // [6] Servo 6
  { 500, 3500, 2000 },   // [7] Servo 7
  { 500, 3500, 2000 },   // [8] Servo 8
};


// ======================================================================
// ==                      COMMUNICATION
// ======================================================================
#define SERIAL_BAUD 115200
#define TELEMETRY_OUTPUT_HZ 10

// SBUS 设置
#define SBUS_BAUD 100000
#define SBUS_CONFIG (SERIAL_8E2)
#define SBUS_FRAME_PERIOD_MS 10
#define SBUS_MIN_VAL 172
#define SBUS_MAX_VAL 1811
#define SBUS_CENTER_VAL 992
//@FILE config.h END