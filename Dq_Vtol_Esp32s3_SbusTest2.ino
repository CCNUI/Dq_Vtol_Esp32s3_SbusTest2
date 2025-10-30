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
float g_current_adc[NUM_SERVOS];      // 输入 (Input)
float g_pid_velocity_out[NUM_SERVOS]; // 输出 (Output)
float g_target_adc[NUM_SERVOS];       // 设定点 (Setpoint)

// 最终发送到舵机的PWM脉宽 (1000-2000us)
volatile float g_final_pwm_us[NUM_SERVOS];

// *** 新增: 存储 LEDC 通道号 ***
int g_ledc_channels[NUM_SERVOS];

// 硬件定时器 (用于触发 1kHz 环路)
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

// USB 串口接收缓冲区
#define RX_BUFFER_SIZE 64
char g_rx_buffer[RX_BUFFER_SIZE];
uint8_t g_rx_buffer_idx = 0;

// Autotune
pid_tuner g_autotuner;
PID g_tuning_pid_instance;  // Autotuner 库使用的 PID 实例
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
void handle_usb_input();
void parseUsbCommand();


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
#else
  Serial.println("--- DEBUG_FIXED_PWM_OUTPUT is DISABLED. ---");
  Serial.println("--- Running normal PID control loop. ---");
#endif
  // --- [END] 新增 DEBUG 模式信息 ---

  // 设置 ADC
  for (int i = 0; i < NUM_SERVOS; i++) {
    analogSetAttenuation(ADC_11db);
    pinMode(ADC_PINS[i], INPUT);
  }

  // 1. 设置 LEDC (PWM)
  setup_pwm();

  // 2. 初始化 PID 设定点
  for (int i = 0; i < NUM_SERVOS; i++) {
    g_target_adc[i] = g_servo_limits[i].center_adc;
  }

  // 3. 设置 PID 控制器
  setup_pid_controllers();

  // 4. 设置 SBUS (Core 0)
  setup_sbus();

  // 5. 设置板载 LED (Core 0)
  onboard_led.begin();
  onboard_led.setBrightness(LED_SIG_BRIGHTNESS);
  onboard_led.fill(onboard_led.Color(0, 0, 255));  // 蓝色表示启动
  onboard_led.show();

  // 6. 创建 Core 1 PID 任务
  Serial.println("Creating Core 1 PID Task...");
  xTaskCreatePinnedToCore(
    pidLoopTask,            // 任务函数
    "PIDLoop",              // 任务名称
    8192,                   // 堆栈大小
    NULL,                   // 任务参数
    configMAX_PRIORITIES - 1, // 最高优先级
    &g_pid_task_handle,     // 任务句柄
    1                       // 固定在 Core 1
  );

  // 7. 创建 Core 0 SBUS 任务
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

  // 8. 设置 1kHz 硬件定时器
  Serial.println("Starting 1kHz PID Timer...");
  pid_timer = timerBegin(1000000);  // 1MHz (1µs 精度)
  timerAttachInterrupt(pid_timer, &onPidTimer);
  timerAlarm(pid_timer, PID_LOOP_PERIOD_US, true, 0);  // 1000µs (1kHz), 自动重载
  timerStart(pid_timer);

  onboard_led.fill(onboard_led.Color(0, 255, 0));  // 绿色表示运行
  onboard_led.show();
  Serial.println("Setup Complete. Running.");
}

// ======================================================================
// ==                           LOOP (CORE 0)
// ======================================================================
void loop() {
  // Core 0 的主循环只负责低优先级的任务：
  // 1. USB 串行输入
  // 2. 遥测数据输出 (如果不在 Autotune 期间)
  // 3. 呼吸灯
  // (SBUS 输出由它自己的任务处理)
  // (Autotune 会阻塞此循环)

  if (!g_is_autotuning) {
    // USB 输入始终处理
    handle_usb_input();
    
    // 仅在不在 Autotune 时打印遥测 (防止阻塞)
    handle_telemetry();
  } else {
    // Autotune 期间，仍然检查 USB 输入
    // (虽然 Autotune 函数 g_autotuner.tune() 是阻塞的, 
    // 但它内部调用的 autotune_output_func 包含 yield(), 
    // 所以 loop 理论上仍有机会运行)
    handle_usb_input();
  }


  // 呼吸灯
  uint8_t brightness = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * (float)LED_MAX_BRIGHTNESS;
  onboard_led.setBrightness(brightness);
  if (g_is_autotuning) {
    onboard_led.fill(onboard_led.Color(255, 0, 0));  // 自动调参时为红色
  } else {
    onboard_led.fill(onboard_led.Color(0, 255, 0));  // 正常运行时为绿色
  }
  onboard_led.show();

  // 允许其他任务运行
  vTaskDelay(pdMS_TO_TICKS(5));
}


// ======================================================================
// ==                     CORE 1 - PID LOOP TASK
// ======================================================================

// 1kHz 定时器中断
void IRAM_ATTR onPidTimer() {
  // 只是通知 PID 任务运行
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(g_pid_task_handle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// PID 环路任务 (运行在 Core 1)
void pidLoopTask(void *pvParameters) {
  Serial.println("pidLoopTask running on Core 1.");

// --- [START] 舵机测试模式 ---
#if DEBUG_FIXED_PWM_OUTPUT == 1
  // --- DEBUG MODE ---
  // PID 任务在此模式下仅负责应用 g_final_pwm_us (由串口设置)
  // 并将它们写入 LEDC 和 SBUS
  Serial.println("PID Task running in DEBUG_FIXED_PWM_OUTPUT mode.");

  // 1. (仅一次) 设置 S5, S6, S7 的初始覆盖值
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
      uint32_t duty = (uint32_t)((local_pwm_us[i] / 20000.0) * 65535.0);
      ledcWrite(g_ledc_channels[i], duty);

      // 2. 存储值供 SBUS 任务使用
      portENTER_CRITICAL(&g_pid_mux);
      g_sbus_channels[i] = map(local_pwm_us[i], 1000, 2000, SBUS_MIN_VAL, SBUS_MAX_VAL);
      portEXIT_CRITICAL(&g_pid_mux);
    }
  }

#else // 正常 PID 模式
// --- [END] 舵机测试模式 ---

  // --- NORMAL PID MODE ---
  while (true) {
    // 等待 onPidTimer 的通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 1. 读取所有 ADC (始终运行)
    read_all_adcs();

    // 2. 运行 8 个 PID 控制器
    for (int i = 0; i < NUM_SERVOS; i++) {

      // --- [START] AUTOTUNE 修复: 检查是否跳过PID ---
      if (g_is_autotuning && i == g_autotune_servo_idx) {
        // --- 自动调参覆盖模式 ---
        // 在此模式下，我们跳过PID计算
        // 只应用 g_final_pwm_us (由 autotune_output_func 设置)
        
        // 1. 从 g_final_pwm_us 获取 PWM
        float pwm_us;
        portENTER_CRITICAL(&g_pid_mux);
        pwm_us = g_final_pwm_us[i];
        portEXIT_CRITICAL(&g_pid_mux);
        
        // 2. 更新 LEDC
        uint32_t duty = (uint32_t)((pwm_us / 20000.0) * 65535.0);
        ledcWrite(g_ledc_channels[i], duty);

        // 3. 存储值供 SBUS 任务使用
        portENTER_CRITICAL(&g_pid_mux);
        g_sbus_channels[i] = map(pwm_us, 1000, 2000, SBUS_MIN_VAL, SBUS_MAX_VAL);
        portEXIT_CRITICAL(&g_pid_mux);
        
        // 4. 跳过此舵机的PID计算
        continue; 
      }
      // --- [END] AUTOTUNE 修复 ---


      // --- 正常PID计算 ---
      // 检查是否在硬限位
      bool at_min_limit = (g_current_adc[i] <= g_servo_limits[i].min_adc);
      bool at_max_limit = (g_current_adc[i] >= g_servo_limits[i].max_adc);

      // QuickPID 计算
      pid_controllers[i].Compute();
      float target_velocity = g_pid_velocity_out[i];

      // 应用硬限位
      bool is_reversed = (g_pid_reverse_action[i] == 1);
      if (is_reversed) {
        if (at_min_limit && target_velocity > 0) {
          target_velocity = 0;
        } else if (at_max_limit && target_velocity < 0) {
          target_velocity = 0;
        }
      } else {
        if (at_min_limit && target_velocity < 0) {
          target_velocity = 0;
        } else if (at_max_limit && target_velocity > 0) {
          target_velocity = 0;
        }
      }

      // 应用死区补偿 (Jump) (使用 config.h 中的 50)
      if (target_velocity > 0.0) {
        target_velocity += PID_VELOCITY_DEADZONE;
      } else if (target_velocity < 0.0) {
        target_velocity -= PID_VELOCITY_DEADZONE;
      }

      // 计算最终PWM (us)
      float pwm_us = 1500.0 + target_velocity;
      pwm_us = constrain(pwm_us, 1000.0, 2000.0);

      // 更新 LEDC (硬件 PWM)
      uint32_t duty = (uint32_t)((pwm_us / 20000.0) * 65535.0);
      ledcWrite(g_ledc_channels[i], duty);

      // 存储值供其他任务使用
      portENTER_CRITICAL(&g_pid_mux);
      g_final_pwm_us[i] = pwm_us;
      g_sbus_channels[i] = map(pwm_us, 1000, 2000, SBUS_MIN_VAL, SBUS_MAX_VAL);
      portEXIT_CRITICAL(&g_pid_mux);
    } // 结束 for 循环
  } // 结束 while(true)
#endif // 结束 DEBUG_FIXED_PWM_OUTPUT 的 #if
}


// 快速读取所有 ADC
void IRAM_ATTR read_all_adcs() {
  // 这是非 DMA 版本
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
    g_ledc_channels[i] = ledcAttach(PWM_PINS[i], 50, 16);  // 引脚, 50Hz, 16位

    // 设置初始中点
    uint32_t center_duty = (uint32_t)((1500.0 / 20000.0) * 65535.0);
    ledcWrite(g_ledc_channels[i], center_duty);
    g_final_pwm_us[i] = 1500.0;
  }
}

// 设置 8 个 PID 控制器
void setup_pid_controllers() {
  Serial.println("Setting up 8 QuickPID controllers...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 链接 QuickPID 到我们的全局变量
    pid_controllers[i] = QuickPID(
      &g_current_adc[i],          // *Input
      &g_pid_velocity_out[i],     // *Output
      &g_target_adc[i],           // *Setpoint
      DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, // Tunings
      QuickPID::pMode::pOnError,      // Proportional on Error
      QuickPID::dMode::dOnMeas,       // Derivative on Measurement
      QuickPID::iAwMode::iAwCondition,  // Anti-windup
      (QuickPID::Action)g_pid_reverse_action[i]  // 从 config.h 读取设置
    );

    pid_controllers[i].SetSampleTimeUs(PID_LOOP_PERIOD_US);
    pid_controllers[i].SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    pid_controllers[i].SetMode(QuickPID::Control::automatic);
  }
}


// ======================================================================
// ==               CORE 0 - COMMUNICATION & OTHER
// ======================================================================

// -----------------
// USB Serial
// -----------------

// USB 轮询函数 (ASCII 协议, 以 \n 结尾)
void handle_usb_input() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (g_rx_buffer_idx > 0) {
        g_rx_buffer[g_rx_buffer_idx] = '\0';
        parseUsbCommand();
        g_rx_buffer_idx = 0;
      }
    } else {
      if (g_rx_buffer_idx < (RX_BUFFER_SIZE - 1)) {
        g_rx_buffer[g_rx_buffer_idx++] = c;
      } else {
        Serial.println("Error: USB command buffer overflow.");
        g_rx_buffer_idx = 0;
      }
    }
  }
}


// 解析收到的 ASCII 命令
void parseUsbCommand() {
  // 格式:
  // P <idx> <kp> <ki> <kd>
  // T <idx> <target>
  // A <idx>
  // S <idx> <pwm_us>

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
    int parsed_items = sscanf(g_rx_buffer, "P %d %f %f %f", &servo_idx, &kp, &ki, &kd);

    if (parsed_items == 4) {
      if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
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
        target_val = constrain(target_val, g_servo_limits[servo_idx].min_adc, g_servo_limits[servo_idx].max_adc);

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
  
  } else if (cmd_char == 'S') {
  
  #if DEBUG_FIXED_PWM_OUTPUT == 1
    // 舵机测试: S <idx> <pwm_us>
    int servo_idx = -1;
    int pwm_val_int = 0;
    int parsed_items = sscanf(g_rx_buffer, "S %d %d", &servo_idx, &pwm_val_int);

    if (parsed_items == 2) {
      if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
        float pwm_us = (float)constrain(pwm_val_int, 1000, 2000);
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
  #else
    Serial.println("Error: 'S' command is only available when DEBUG_FIXED_PWM_OUTPUT is 1.");
  #endif

  } else {
    Serial.printf("Error: Unknown command '%s'\n", g_rx_buffer);
  }
}


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
  SbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_PIN, true);  // RX, TX, invert

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
    sbus_frame[0] = 0x0F;  // 帧头

    // 我们需要安全地读取 g_sbus_channels
    uint16_t channels_copy[16];
    portENTER_CRITICAL(&g_pid_mux);
    memcpy(channels_copy, g_sbus_channels, 8 * sizeof(uint16_t));  // 只复制前8个
    portEXIT_CRITICAL(&g_pid_mux);

    // 手动打包前8个通道
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

    sbus_frame[23] = 0x00;  // 标志位
    sbus_frame[24] = 0x00;  // 帧尾

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
  // [!] 修复: 不再调用 analogRead()
  // 而是从主PID环路 (Core 1) 已经更新的全局数组中读取
  double adc_value;
  portENTER_CRITICAL(&g_pid_mux);
  adc_value = (double)g_current_adc[servo_index];
  portEXIT_CRITICAL(&g_pid_mux);
  return adc_value;
}

// 自动调参输出函数 (供 autotune 库回调)
void autotune_output_func(double output) {
  // --- 修复: 喂狗 (WDT Reset) ---
  // Autotune 是一个长时阻塞操作，在 Core 0 上运行
  // 调用 yield() 会交出控制权 (vTaskDelay(1))
  // 这会重置任务看门狗并防止重启
  yield(); 

  // autotune 库的输出被配置为 1000-2000
  float pwm_us = constrain(output, 1000.0, 2000.0);

  // --- 应用死区补偿 (Jump) (使用 config.h 中的 50) ---
  float velocity_out = pwm_us - 1500.0; 
  if (velocity_out > 0.0) {
    velocity_out += PID_VELOCITY_DEADZONE;
  } else if (velocity_out < 0.0) {
    velocity_out -= PID_VELOCITY_DEADZONE;
  }
  
  pwm_us = 1500.0 + velocity_out;
  pwm_us = constrain(pwm_us, 1000.0, 2000.0);
  // --- 结束死区逻辑 ---

  // [!] 修复: 不再直接写入 ledcWrite
  // 而是更新全局变量，让 pidLoopTask (Core 1) 来写入
  portENTER_CRITICAL(&g_pid_mux);
  g_final_pwm_us[g_autotune_servo_idx] = pwm_us;
  portEXIT_CRITICAL(&g_pid_mux);
}

// 启动自动调参的函数
void start_autotune(int servo_index) {
  if (g_is_autotuning) return;

  g_is_autotuning = true;
  g_autotune_servo_idx = servo_index;

  // --- [START] 修复: 移除 vTaskSuspend ---
  // 我们不再暂停 PID 任务，以避免 ADC 死锁
  Serial.printf("Autotuning Servo %d. PID task (Core 1) will continue running.\n", servo_index);
  // --- [END] 修复 ---


  // 2. 配置调谐器
  g_tuning_pid_instance = PID(
    g_servo_limits[servo_index].center_adc,  // 目标值
    10000,                  // 环路间隔 (us) -> 10ms
    1000, 2000                // 输出范围 (pwm us)
  );
  
  // 确保 Autotune 使用正确的舵机方向
  if (g_pid_reverse_action[servo_index] == 1) {
    g_tuning_pid_instance.SetControllerDirection(REVERSE);
    Serial.println("Autotune PID set to REVERSE mode.");
  } else {
    g_tuning_pid_instance.SetControllerDirection(DIRECT);
    Serial.println("Autotune PID set to DIRECT mode.");
  }

  g_autotuner.setPID(g_tuning_pid_instance);
  g_autotuner.setTargetValue(g_servo_limits[servo_index].center_adc);
  g_autotuner.setConstrains(1000.0, 2000.0);  // 输出是 PWM us
  g_autotuner.setLoopInterval(10000);       // 10ms
  g_autotuner.setTuningCycles(10);
  g_autotuner.setMode(pid_tuner::mode_t::CLASSIC_PID);

  Serial.println("Autotuner configured. Starting tuning loop (this is blocking)...");
  Serial.println("Telemetry will pause during tuning.");

  // 3. 运行阻塞的调谐
  // 这将接管, 直到调谐完成
  // 它现在调用我们修改过的 autotune_input_func 和 autotune_output_func
  g_autotuner.tune(autotune_input_func, servo_index, autotune_output_func);

  Serial.println("Tuning complete.");
  Serial.println("Telemetry will now resume.");

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

  // --- [START] 修复: 移除 vTaskResume ---
  // vTaskResume(g_pid_task_handle); // <-- 移除
  // --- [END] 修复 ---

  g_is_autotuning = false;
}
//@FILE Dq_Vtol_Esp32s3_SbusTest2.ino END