//@FILE Dq_Vtol_Esp32s3_SbusTest2.ino START
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "QuickPID.h"
#include "pid-autotune.h"  // 包含 Autotune 库
#include "config.h"        // 包含所有配置

// ======================================================================
// ==                           GLOBAL VARIABLES
// ======================================================================

// -- Core 1 (PID) 变量 --
// PID 控制器实例
QuickPID pid_controllers[NUM_SERVOS];

// PID 计算的输入、输出和设定点
// 这些是 QuickPID 对象内部指针将指向的变量
float g_current_adc[NUM_SERVOS];
// 输入 (Input)
float g_pid_velocity_out[NUM_SERVOS];  // 输出 (Output)
float g_target_adc[NUM_SERVOS];
// 设定点 (Setpoint)

// 最终发送到舵机的PWM脉宽 (1000-2000us)
volatile float g_final_pwm_us[NUM_SERVOS];

// *** 新增: 存储 LEDC 通道号 ***
int g_ledc_channels[NUM_SERVOS];

// 硬件定时器 (用于触发 8kHz 环路)
hw_timer_t *pid_timer = NULL;

// Core 1 PID 任务句柄
TaskHandle_t g_pid_task_handle = NULL;

// 保护 PID 变量的互斥锁/临界区
portMUX_TYPE g_pid_mux = portMUX_INITIALIZER_UNLOCKED;

// -- Core 0 (Comms/LED) 变量 --

// 板载 LED
Adafruit_NeoPixel onboard_led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// SBUS 串口 (UART1)
HardwareSerial SbusSerial(1);

// SBUS 通道数据 (16个通道, 0-2047)
uint16_t g_sbus_channels[16];

// SBUS 任务句柄
TaskHandle_t g_sbus_task_handle = NULL;

// 遥测数据输出计时器
unsigned long g_last_telemetry_time = 0;

// --- [START] 修改: 增大缓冲区并改为 char (用于 ASCII) ---
#define RX_BUFFER_SIZE 64  // 增加缓冲区大小以容纳 ASCII 命令 (例如 "P 0 1.234 0.5 0.01")
char g_rx_buffer[RX_BUFFER_SIZE];
// --- [END] 修改: 增大缓冲区并改为 char (用于 ASCII) ---
uint8_t g_rx_buffer_idx = 0;

// Autotune
pid_tuner g_autotuner;
PID g_tuning_pid_instance;
// Autotuner 库使用的 PID 实例
volatile bool g_is_autotuning = false;
volatile int g_autotune_servo_idx = 0;

// ======================================================================
// ==                           PROTOTYPES
// ======================================================================

// Core 1
void IRAM_ATTR onPidTimer();
void pidLoopTask(void *pvParameters);
void setup_pid_controllers();
void setup_pwm();
void read_all_adcs();

// Core 0
void sbusTask(void *pvParameters);
void setup_sbus();
void update_sbus_frame();
void ledBreathingTask(void *pvParameters);
void handle_telemetry();
// --- [START] 修改: 更改函数名称 ---
void handle_usb_input();
void parseUsbCommand();  // 重命名 parseUsbPacket
// --- [END] 修改: 更改函数名称 ---


// Autotune
void start_autotune(int servo_index);
void stop_autotune();
double autotune_input_func(int servo_index);
void autotune_output_func(double output);

// ======================================================================
// ==                           SETUP (CORE 0)
// ======================================================================
void setup() {
  // 启动 USB 串口
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32-S3 PID Servo Controller Booting...");
  Serial.println("Using ASCII Command Protocol (Newline-terminated).");
  Serial.println("Commands:");
  Serial.println("  P <idx> <Kp> <Ki> <Kd>  (Set PID)");
  Serial.println("  T <idx> <target_adc>    (Set Target)");
  Serial.println("  A <idx>                 (Start Autotune)");

  // --- [START] 新增 DEBUG 模式信息 ---
#if DEBUG_FIXED_PWM_OUTPUT == 1
  Serial.println("------------------------------------------------------");
  Serial.println("--- WARNING: DEBUG_FIXED_PWM_OUTPUT IS ENABLED ---");
  Serial.println("--- PID 环路已禁用。");
  Serial.println("--- 初始值: S5=1400, S6=1500, S7=1600");
  Serial.println("--- 使用 'S <idx> <pwm_us>' (例如 'S 5 1200') 来覆盖。");
  Serial.println("------------------------------------------------------");
#endif
  // --- [END] 新增 DEBUG 模式信息 ---

  // 设置 ADC (注意: S3 的 ADC 精度可能需要校准)
  // ESP32-S3 上的 ADC2 (TOUCH 引脚使用) 不能与 WiFi 同时使用
  // 我们在这里设置11dB衰减 (0-3.3V) 和12位宽度 (0-4095)
  for (int i = 0; i < NUM_SERVOS; i++) {
    analogSetAttenuation(ADC_11db);
    pinMode(ADC_PINS[i], INPUT);
  }

  // 1. 设置 LEDC (PWM)
  setup_pwm();

  // 2. 初始化 PID 设定点
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 默认启动时目标为中心点
    g_target_adc[i] = g_servo_limits[i].center_adc;
  }

  // 3. 设置 PID 控制器
  setup_pid_controllers();

  // 4. 设置 SBUS (Core 0)
  setup_sbus();

  // 5. 设置板载 LED (Core 0)
  onboard_led.begin();
  onboard_led.setBrightness(LED_SIG_BRIGHTNESS);
  onboard_led.fill(onboard_led.Color(0, 0, 255));
  // 蓝色表示启动
  onboard_led.show();

  // 6. 设置 USB CDC 回调 (Core 0)
  // *** 已修复: 移除 onData 回调，我们将使用 loop() 轮询 ***
  // Serial.onData(onUsbData);

  // 7. 创建 Core 1 PID 任务
  Serial.println("Creating Core 1 PID Task...");
  xTaskCreatePinnedToCore(
    pidLoopTask,               // 任务函数
    "PIDLoop",                 // 任务名称
    8192,                      // 堆栈大小 (PID + aalogRead 需要较大堆栈)
    NULL,                      // 任务参数
    configMAX_PRIORITIES - 1,  // 最高优先级
    &g_pid_task_handle,        // 任务句柄
    1                          // 固定在 Core 1
  );

  // 8. 创建 Core 0 SBUS 任务
  Serial.println("Creating Core 0 SBUS Task...");
  xTaskCreatePinnedToCore(
    sbusTask,             // 任务函数
    "SBUSTx",             // 任务名称
    2048,                 // 堆栈大小
    NULL,                 // 任务参数
    5,                    // 中等优先级
    &g_sbus_task_handle,  // 任务句柄
    0                     // 固定在 Core 0
  );

  // 9. 设置 8kHz 硬件定时器
  Serial.println("Starting 8kHz PID Timer...");
  // *** 已修复: ESP32 Core 3.x 定时器 API ***
  pid_timer = timerBegin(1000000);
  // 1MHz (1µs 精度)
  timerAttachInterrupt(pid_timer, &onPidTimer);
  // *** 已修复: 添加第4个参数 (0) 用于自动重载 ***
  timerAlarm(pid_timer, PID_LOOP_PERIOD_US, true, 0);
  // 125µs, 自动重载, 0 = 无限
  timerStart(pid_timer);

  onboard_led.fill(onboard_led.Color(0, 255, 0));
  // 绿色表示运行
  onboard_led.show();
  Serial.println("Setup Complete. Running.");
}

// ======================================================================
// ==                           LOOP (CORE 0)
// ======================================================================
void loop() {
  // Core 0 的主循环只负责低优先级的任务：
  // 1. USB 串行输入 (新)
  // 2. 遥测数据输出
  // 3. 呼吸灯
  // (SBUS 输出由它自己的任务处理)

  if (!g_is_autotuning) {
    // *** 新增: 轮询 USB 输入 ***
    handle_usb_input();

    // 正常遥测
    handle_telemetry();
  }

  // 呼吸灯
  uint8_t brightness = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * (float)LED_MAX_BRIGHTNESS;
  onboard_led.setBrightness(brightness);
  if (g_is_autotuning) {
    onboard_led.fill(onboard_led.Color(255, 0, 0));
    // 自动调参时为红色

  } else {
    onboard_led.fill(onboard_led.Color(0, 255, 0));
    // 正常运行时为绿色
  }
  onboard_led.show();

  // 允许其他任务运行
  vTaskDelay(pdMS_TO_TICKS(5));
}


// ======================================================================
// ==                     CORE 1 - PID LOOP TASK
// ======================================================================

// 8kHz 定时器中断
void IRAM_ATTR onPidTimer() {
  // 只是通知 PID 任务运行
  // 不要在 ISR 中做任何实际工作!
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(g_pid_task_handle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// PID 环路任务 (运行在 Core 1)
void pidLoopTask(void *pvParameters) {
  Serial.println("pidLoopTask running on Core 1.");

// --- [START] 新增: 舵机测试模式 ---
#if DEBUG_FIXED_PWM_OUTPUT == 1
  // --- DEBUG MODE ---
  // PID 任务在此模式下仅负责应用 g_final_pwm_us (由串口设置)
  // 并将它们写入 LEDC 和 SBUS
  Serial.println("PID Task running in DEBUG_FIXED_PWM_OUTPUT mode.");

  // 1. (仅一次) 设置 S5, S6, S7 的初始覆盖值
  // g_final_pwm_us 已经在 setup_pwm() 中被初始化为 1500
  portENTER_CRITICAL(&g_pid_mux);
  g_final_pwm_us[5] = 1400.0;
  g_final_pwm_us[6] = 1500.0; // 冗余，但明确
  g_final_pwm_us[7] = 1600.0;
  portEXIT_CRITICAL(&g_pid_mux);

  while (true) {
    // 等待 onPidTimer 的通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 在此模式下，我们不读取ADC或计算PID
    // 我们只获取 g_final_pwm_us (由 Core 0 的串口命令设置) 
    // 并将其写入 PWM 和 SBUS

    float local_pwm_us[NUM_SERVOS];
    portENTER_CRITICAL(&g_pid_mux);
    memcpy(local_pwm_us, (void *)g_final_pwm_us, sizeof(g_final_pwm_us));
    portEXIT_CRITICAL(&g_pid_mux);

    for (int i = 0; i < NUM_SERVOS; i++) {
      // 1. 更新 LEDC (硬件 PWM)
      // (20000µs = 50Hz 周期)
      uint32_t duty = (uint32_t)((local_pwm_us[i] / 20000.0) * 65535.0);
      ledcWrite(g_ledc_channels[i], duty);

      // 2. 存储值供 SBUS 任务使用
      portENTER_CRITICAL(&g_pid_mux);
      // g_final_pwm_us 已经被 Core 0 设置了，这里只更新 sbus channels
      g_sbus_channels[i] = map(local_pwm_us[i], 1000, 2000, SBUS_MIN_VAL, SBUS_MAX_VAL);
      portEXIT_CRITICAL(&g_pid_mux);
    }
  }

#else // 正常 PID 模式
// --- [END] 新增: 舵机测试模式 ---

  // --- NORMAL PID MODE ---
  while (true) {
    // 等待 onPidTimer 的通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // --- 125µs 预算开始 ---

    // 1. 读取所有 ADC
    read_all_adcs();

    // 2. 运行 8 个 PID 控制器
    for (int i = 0; i < NUM_SERVOS; i++) {

      // 检查是否在硬限位
      bool at_min_limit = (g_current_adc[i] <= g_servo_limits[i].min_adc);
      bool at_max_limit = (g_current_adc[i] >= g_servo_limits[i].max_adc);

      // QuickPID 计算
      pid_controllers[i].Compute();
      // g_pid_velocity_out[i] 现在包含了 PID 输出 (范围 -500 到 500)

      // 3. 应用限位和死区
      float target_velocity = g_pid_velocity_out[i];

      // 应用硬限位 (已修复反向舵机逻辑)
      bool is_reversed = (g_pid_reverse_action[i] == 1);

      if (is_reversed) {
        // --- 反向舵机逻辑 ---
        // target_velocity > 0 (PWM > 1500) 是 "减小 ADC"
        // target_velocity < 0 (PWM < 1500) 是 "增大 ADC"

        if (at_min_limit && target_velocity > 0) {
          target_velocity = 0;
        } else if (at_max_limit && target_velocity < 0) {
          target_velocity = 0;
        }
      } else {
        // --- 正向舵机逻辑 (原始代码) ---
        // target_velocity > 0 (PWM > 1500) 是 "增大 ADC"
        // target_velocity < 0 (PWM < 1500) 是 "减小 ADC"

        if (at_min_limit && target_velocity < 0) {
          target_velocity = 0;
        } else if (at_max_limit && target_velocity > 0) {
          target_velocity = 0;
        }
      }

      // --- [START] 逻辑修改: 应用死区补偿 (Jump) ---
      // (旧逻辑: if (abs(target_velocity) < PID_VELOCITY_DEADZONE) { target_velocity = 0; } )
      
      // 新逻辑: "跳过" 死区
      if (target_velocity > 0.0) {
        target_velocity += PID_VELOCITY_DEADZONE;
      } else if (target_velocity < 0.0) {
        target_velocity -= PID_VELOCITY_DEADZONE;
      }
      // 如果 target_velocity 是 0.0, 它将保持 0.0.
      // --- [END] 逻辑修改 ---

      // 4. 计算最终PWM (us)
      float pwm_us = 1500.0 + target_velocity;

      // 最终钳位
      pwm_us = constrain(pwm_us, 1000.0, 2000.0);

      // 5. 更新 LEDC (硬件 PWM)
      uint32_t duty = (uint32_t)((pwm_us / 20000.0) * 65535.0);
      ledcWrite(g_ledc_channels[i], duty);

      // 6. 存储值供其他任务使用
      portENTER_CRITICAL(&g_pid_mux);
      g_final_pwm_us[i] = pwm_us;
      // 将 1000-2000us 映射到 SBUS 范围
      g_sbus_channels[i] = map(pwm_us, 1000, 2000, SBUS_MIN_VAL, SBUS_MAX_VAL);
      portEXIT_CRITICAL(&g_pid_mux);
    }

    // --- 125µs 预算结束 ---
  }
#endif // 结束 DEBUG_FIXED_PWM_OUTPUT 的 #if
}

// 快速读取所有 ADC
void IRAM_ATTR read_all_adcs() {
  // 这是非 DMA 版本，它会很慢
  for (int i = 0; i < NUM_SERVOS; i++) {
    portENTER_CRITICAL(&g_pid_mux);
    g_current_adc[i] = (float)analogRead(ADC_PINS[i]);
    portEXIT_CRITICAL(&g_pid_mux);
  }
}

// 设置 8 个 LEDC 通道
void setup_pwm() {
  Serial.println("Setting up 8 LEDC channels...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    // *** 已修复: ESP32 3.x API - ledcAttach 替代 ledcSetup/ledcAttachPin ***
    // ledcAttach(pin, freq, resolution) 返回分配的通道
    g_ledc_channels[i] = ledcAttach(PWM_PINS[i], 50, 16);
    // 引脚, 50Hz, 16位

    // 设置初始中点
    uint32_t center_duty = (uint32_t)((1500.0 / 20000.0) * 65535.0);
    ledcWrite(g_ledc_channels[i], center_duty);
    // 使用返回的通道
    g_final_pwm_us[i] = 1500.0;
  }
}

// 设置 8 个 PID 控制器
void setup_pid_controllers() {
  Serial.println("Setting up 8 QuickPID controllers...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 链接 QuickPID 到我们的全局变量
    pid_controllers[i] = QuickPID(
      &g_current_adc[i],                         // *Input
      &g_pid_velocity_out[i],                    // *Output
      &g_target_adc[i],                          // *Setpoint
      DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,        // Tunings
      QuickPID::pMode::pOnError,                 // Proportional on Error
      QuickPID::dMode::dOnMeas,                  // Derivative on Measurement
      QuickPID::iAwMode::iAwCondition,           // Anti-windup
                                                 //QuickPID::Action::direct      // Direct action
      (QuickPID::Action)g_pid_reverse_action[i]  // <-- 从 config.h 读取设置
    );

    // 设置 PID 采样时间 (虽然我们用定时器触发，但设置一下没坏处)
    pid_controllers[i].SetSampleTimeUs(PID_LOOP_PERIOD_US);
    // 设置输出限制 (速度)
    pid_controllers[i].SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    // 切换到自动模式
    pid_controllers[i].SetMode(QuickPID::Control::automatic);
  }
}


// ======================================================================
// ==               CORE 0 - COMMUNICATION & OTHER
// ======================================================================

// -----------------
// USB Serial
// -----------------

// --- [START] 修改: USB 轮询函数 (ASCII 协议, 以 \n 结尾) ---
void handle_usb_input() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // 忽略回车符，只关心换行符
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      // 收到行尾
      if (g_rx_buffer_idx > 0) {
        // 确保字符串以 null 结尾
        g_rx_buffer[g_rx_buffer_idx] = '\0';

        // 解析命令
        parseUsbCommand();

        // 重置缓冲区
        g_rx_buffer_idx = 0;
      }

    } else {
      // 将字符添加到缓冲区
      if (g_rx_buffer_idx < (RX_BUFFER_SIZE - 1)) {
        g_rx_buffer[g_rx_buffer_idx++] = c;

      } else {
        // 缓冲区溢出，丢弃此行
        Serial.println("Error: USB command buffer overflow.");
        g_rx_buffer_idx = 0;
      }
    }
  }
}
// --- [END] 修改: USB 轮询函数 (ASCII 协议, 以 \n 结尾) ---


// --- [START] 修改: ASCII 命令解析 (替换 parseUsbPacket) ---
// 解析收到的 ASCII 命令
void parseUsbCommand() {
  // g_rx_buffer 现在包含一个以 null 结尾的命令字符串
  // 格式:
  // P <idx> <kp> <ki> <kd>
  // T <idx> <target>
  // A <idx>
  // S <idx> <pwm_us>  <-- 新增

  char cmd_char;
  int items = sscanf(g_rx_buffer, "%c", &cmd_char);

  if (items < 1) {
    Serial.println("Error: Empty command.");
    return;
  }

  if (cmd_char == 'P') {
    // 设置 PID: P <idx> <kp> <ki> <kd>
    int servo_idx = -1;
    float kp = 0, ki = 0, kd = 0;

    // sscanf 会返回成功匹配和分配的项数
    int parsed_items = sscanf(g_rx_buffer, "P %d %f %f %f", &servo_idx, &kp, &ki, &kd);

    if (parsed_items == 4) {
      // 成功解析所有 4 个参数
      if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {

        // 安全地更新 PID 控制器
        portENTER_CRITICAL(&g_pid_mux);
        pid_controllers[servo_idx].SetTunings(kp, ki, kd);
        portEXIT_CRITICAL(&g_pid_mux);

        Serial.printf("Set Servo %d Tunings: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", servo_idx, kp, ki, kd);

      } else {
        Serial.printf("Error: Invalid servo index %d\n", servo_idx);
      }

    } else {
      Serial.printf("Error: Invalid 'P' command format. Expected: P <idx> <kp> <ki> <kd>\n", g_rx_buffer);
    }


  } else if (cmd_char == 'T') {
    // 设置目标: T <idx> <target>
    int servo_idx = -1;
    int target_val_int = 0;

    int parsed_items = sscanf(g_rx_buffer, "T %d %d", &servo_idx, &target_val_int);

    if (parsed_items == 2) {
      if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
        uint16_t target_val = (uint16_t)target_val_int;

        // 限制目标在安全范围内
        target_val = constrain(target_val, g_servo_limits[servo_idx].min_adc, g_servo_limits[servo_idx].max_adc);

        // 安全地更新目标值
        portENTER_CRITICAL(&g_pid_mux);
        g_target_adc[servo_idx] = (float)target_val;
        portEXIT_CRITICAL(&g_pid_mux);

        Serial.printf("Set Servo %d Target to %d\n", servo_idx, target_val);

      } else {
        Serial.printf("Error: Invalid servo index %d\n", servo_idx);
      }

    } else {
      Serial.printf("Error: Invalid 'T' command format. Expected: T <idx> <target>\n", g_rx_buffer);
    }


  } else if (cmd_char == 'A') {
    // 自动调参: A <idx>
    int servo_idx = -1;

    int parsed_items = sscanf(g_rx_buffer, "A %d", &servo_idx);

    if (parsed_items == 1) {
      if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
        if (!g_is_autotuning) {
          Serial.printf("Starting Autotune for Servo %d...\n", servo_idx);
          start_autotune(servo_idx);

        } else {
          Serial.println("Error: Autotune already in progress.");
        }

      } else {
        Serial.printf("Error: Invalid servo index %d\n", servo_idx);
      }

    } else {
      Serial.printf("Error: Invalid 'A' command format. Expected: A <idx>\n", g_rx_buffer);
    }

    // --- [START] 新增舵机测试命令 'S' ---

  } else if (cmd_char == 'S') {

#if DEBUG_FIXED_PWM_OUTPUT == 1
    // 舵机测试: S <idx> <pwm_us>
    int servo_idx = -1;
    int pwm_val_int = 0;

    int parsed_items = sscanf(g_rx_buffer, "S %d %d", &servo_idx, &pwm_val_int);

    if (parsed_items == 2) {
      if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
        // 限制 PWM 在 1000-2000us
        float pwm_us = (float)constrain(pwm_val_int, 1000, 2000);

        // 安全地更新 g_final_pwm_us
        portENTER_CRITICAL(&g_pid_mux);
        g_final_pwm_us[servo_idx] = pwm_us;
        portEXIT_CRITICAL(&g_pid_mux);

        Serial.printf("DEBUG_PWM: Set Servo %d PWM to %.0f us\n", servo_idx, pwm_us);

      } else {
        Serial.printf("Error: Invalid servo index %d\n", servo_idx);
      }

    } else {
      Serial.printf("Error: Invalid 'S' command format. Expected: S <idx> <pwm_us>\n", g_rx_buffer);
    }
#else Serial.println("Error: 'S' command is only available when DEBUG_FIXED_PWM_OUTPUT is 1.");
#endif  // --- [END] 新增舵机测试命令 'S' ---


  } else {
    Serial.printf("Error: Unknown command '%s'\n", g_rx_buffer);
  }
}
// --- [END] 修改: ASCII 命令解析 (替换 parseUsbPacket) ---


// 遥测数据输出
void handle_telemetry() {
  if (millis() - g_last_telemetry_time < (1000 / TELEMETRY_OUTPUT_HZ)) {
    return;
  }
  g_last_telemetry_time = millis();

  // 快速复制数据，避免长时间锁定
  float adc_copy[NUM_SERVOS];
  float pwm_copy[NUM_SERVOS];
  float tgt_copy[NUM_SERVOS];

  portENTER_CRITICAL(&g_pid_mux);
  memcpy(adc_copy, g_current_adc, sizeof(adc_copy));
  memcpy(pwm_copy, (void *)g_final_pwm_us, sizeof(pwm_copy));
  memcpy(tgt_copy, g_target_adc, sizeof(tgt_copy));
  portEXIT_CRITICAL(&g_pid_mux);

  // 打印复制的数据
  // 格式: [S0: ADC=1234, TGT=1235, PWM=1500]
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("[S%d: ADC=%.0f, TGT=%.0f, PWM=%.0f] ",
                  i,
                  adc_copy[i],
                  tgt_copy[i],
                  pwm_copy[i]);
  }
  Serial.println();
}

// -----------------
// SBUS
// -----------------

void setup_sbus() {
  // 配置 SBUS 串口
  // *** 已修复: 使用 'invert'布尔参数 (true) ***
  SbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_PIN, true);
  // RX, TX, invert

  // 初始化通道数据
  for (int i = 0; i < 16; i++) {
    g_sbus_channels[i] = SBUS_CENTER_VAL;
  }
}

// SBUS 任务 (运行在 Core 0)
void sbusTask(void *pvParameters) {
  Serial.println("sbusTask running on Core 0.");
  uint8_t sbus_frame[25];

  while (true) {
    // 1. 准备 SBUS 帧
    sbus_frame[0] = 0x0F;
    // 帧头

    // 11位通道数据被打包到 22 个字节中
    // 我们需要安全地读取 g_sbus_channels
    uint16_t channels_copy[16];
    portENTER_CRITICAL(&g_pid_mux);
    memcpy(channels_copy, g_sbus_channels, 8 * sizeof(uint16_t));
    // 只复制前8个
    portEXIT_CRITICAL(&g_pid_mux);

    // 手动打包前8个通道
    // 这是 SBUS 协议的标准打包方式
    sbus_frame[1] = (uint8_t)(channels_copy[0] & 0x07FF);
    sbus_frame[2] = (uint8_t)((channels_copy[0] & 0x07FF) >> 8 | (channels_copy[1] & 0x07FF) << 3);
    sbus_frame[3] = (uint8_t)((channels_copy[1] & 0x07FF) >> 5 | (channels_copy[2] & 0x07FF) << 6);
    sbus_frame[4] = (uint8_t)((channels_copy[2] & 0x07FF) >> 2);
    sbus_frame[5] = (uint8_t)((channels_copy[2] & 0x07FF) >> 10 | (channels_copy[3] & 0x07FF) << 1);
    sbus_frame[6] = (uint8_t)((channels_copy[3] & 0x07FF) >> 7 | (channels_copy[4] & 0x07FF) << 4);
    sbus_frame[7] = (uint8_t)((channels_copy[4] & 0x07FF) >> 4 | (channels_copy[5] & 0x07FF) << 7);
    sbus_frame[8] = (uint8_t)((channels_copy[5] & 0x07FF) >> 1);
    sbus_frame[9] = (uint8_t)((channels_copy[5] & 0x07FF) >> 9 | (channels_copy[6] & 0x07FF) << 2);
    sbus_frame[10] = (uint8_t)((channels_copy[6] & 0x07FF) >> 6 | (channels_copy[7] & 0x07FF) << 5);
    sbus_frame[11] = (uint8_t)((channels_copy[7] & 0x07FF) >> 3);

    // 清零其余通道字节 (9-15)
    for (int i = 12; i < 23; i++) {
      sbus_frame[i] = 0x00;
    }

    sbus_frame[23] = 0x00;
    // 标志位
    sbus_frame[24] = 0x00;
    // 帧尾

    // 2. 发送
    SbusSerial.write(sbus_frame, 25);

    // 3. 等待下一个帧周期
    vTaskDelay(pdMS_TO_TICKS(SBUS_FRAME_PERIOD_MS));
  }
}

// ======================================================================
// ==                     AUTOTUNE (BLOCKING)
// ======================================================================

// 自动调参输入函数 (供 autotune 库回调)
double autotune_input_func(int servo_index) {
  return (double)analogRead(ADC_PINS[servo_index]);
}

// 自动调参输出函数 (供 autotune 库回调)
// 自动调参输出函数 (供 autotune 库回调)
void autotune_output_func(double output) {
  // --- [START] 修复 2: 喂狗 (WDT Reset) ---
  // Autotune 是一个长时阻塞操作，在 Core 0 上运行
  // 调用 yield() 会交出控制权 (vTaskDelay(1))
  // 这会重置任务看门狗并防止重启
  yield(); 
  // --- [END] 修复 2 ---

  // autotune 库的输出被配置为 1000-2000
  float pwm_us = constrain(output, 1000.0, 2000.0);

  // --- [START] 逻辑修改: 应用死区补偿 (Jump) ---
  // (旧逻辑: if (abs(velocity) < 100) pwm_us = 1500; )

  // 将 PWM (1000-2000) 转换回 速度 (-500, +500)
  float velocity_out = pwm_us - 1500.0; 

  // 新逻辑: "跳过" 死区
  // (与 pidLoopTask 中的逻辑保持一致)
  if (velocity_out > 0.0) {
    velocity_out += PID_VELOCITY_DEADZONE;
  } else if (velocity_out < 0.0) {
    velocity_out -= PID_VELOCITY_DEADZONE;
  }
  
  // 将补偿后的速度转换回 PWM
  pwm_us = 1500.0 + velocity_out;

  // 再次钳位，因为我们增加了速度值
  pwm_us = constrain(pwm_us, 1000.0, 2000.0);
  // --- [END] 逻辑修改 ---

  // 直接写入 PWM
  uint32_t duty = (uint32_t)((pwm_us / 20000.0) * 65535.0);
  // *** 已修复: 使用 g_ledc_channels 数组 ***
  ledcWrite(g_ledc_channels[g_autotune_servo_idx], duty);
}
// 启动自动调参的函数
void start_autotune(int servo_index) {
  if (g_is_autotuning) return;

  g_is_autotuning = true;
  g_autotune_servo_idx = servo_index;

  Serial.printf("Suspending PID Task on Core 1 for Autotune (Servo %d).\n", servo_index);
  // 1. 暂停 8kHz PID 环路
  vTaskSuspend(g_pid_task_handle);

  // 2. 配置调谐器
  // 注意：我们正在使用 g_tuning_pid_instance (PID 类)
  // 而不是 g_pid_controllers (QuickPID 类)
  g_tuning_pid_instance = PID(
    g_servo_limits[servo_index].center_adc,  // 目标值
    10000,                                   // 环路间隔 (us) -> 10ms
    1000, 2000                               // 输出范围 (pwm us)
  );

  g_autotuner.setPID(g_tuning_pid_instance);
  g_autotuner.setTargetValue(g_servo_limits[servo_index].center_adc);
  g_autotuner.setConstrains(1000.0, 2000.0);  // 输出是 PWM us
  g_autotuner.setLoopInterval(10000);         // 10ms  <-- [!] 已修正此行
  g_autotuner.setTuningCycles(10);
  g_autotuner.setMode(pid_tuner::mode_t::CLASSIC_PID);

  Serial.println("Autotuner configured. Starting tuning loop (this is blocking)...");

  // 3. 运行阻塞的调谐
  // 这将接管, 直到调谐完成
  g_autotuner.tune(autotune_input_func, servo_index, autotune_output_func);

  Serial.println("Tuning complete.");

  // 4. 获取调谐结果
  float Kp = g_autotuner.getKp();
  float Ki = g_autotuner.getKi();
  float Kd = g_autotuner.getKd();

  Serial.printf("Tuned Values: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", Kp, Ki, Kd);

  // 5. 将新值应用到 *QuickPID* 控制器
  portENTER_CRITICAL(&g_pid_mux);
  pid_controllers[servo_index].SetTunings(Kp, Ki, Kd);
  portEXIT_CRITICAL(&g_pid_mux);

  Serial.printf("New tunings applied to QuickPID Servo %d.\n", servo_index);

  // 6. 恢复 8kHz PID 环路
  vTaskResume(g_pid_task_handle);
  Serial.println("Resumed PID Task on Core 1.");

  g_is_autotuning = false;
}
//@FILE Dq_Vtol_Esp32s3_SbusTest2.ino END