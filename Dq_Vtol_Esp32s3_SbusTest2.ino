//@FILE Dq_Vtol_Esp32s3_SbusTest2.ino START
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "QuickPID.h"
#include "pid-autotune.h" // 包含 Autotune 库
#include "config.h"       // 包含所有配置

// ======================================================================
// ==                         GLOBAL VARIABLES
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

// USB 输入缓冲区
#define RX_BUFFER_SIZE 10
uint8_t g_rx_buffer[RX_BUFFER_SIZE];
uint8_t g_rx_buffer_idx = 0;

// Autotune
pid_tuner g_autotuner;
PID       g_tuning_pid_instance; // Autotuner 库使用的 PID 实例
volatile bool g_is_autotuning = false;
volatile int g_autotune_servo_idx = 0;

// ======================================================================
// ==                      PROTOTYPES
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
// *** 更改: 移除 onUsbData, 添加 handle_usb_input ***
void handle_usb_input();
void parseUsbPacket();

// Autotune
void start_autotune(int servo_index);
void stop_autotune();
double autotune_input_func(int servo_index);
void autotune_output_func(double output);

// ======================================================================
// ==                         SETUP (CORE 0)
// ======================================================================
void setup() {
  // 启动 USB 串口
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32-S3 PID Servo Controller Booting...");

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
  for(int i=0; i<NUM_SERVOS; i++) {
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
  onboard_led.fill(onboard_led.Color(0, 0, 255)); // 蓝色表示启动
  onboard_led.show();

  // 6. 设置 USB CDC 回调 (Core 0)
  // *** 已修复: 移除 onData 回调，我们将使用 loop() 轮询 ***
  // Serial.onData(onUsbData); 

  // 7. 创建 Core 1 PID 任务
  Serial.println("Creating Core 1 PID Task...");
  xTaskCreatePinnedToCore(
    pidLoopTask,         // 任务函数
    "PIDLoop",           // 任务名称
    8192,                // 堆栈大小 (PID + aalogRead 需要较大堆栈)
    NULL,                // 任务参数
    configMAX_PRIORITIES - 1, // 最高优先级
    &g_pid_task_handle,  // 任务句柄
    1                    // 固定在 Core 1
  );

  // 8. 创建 Core 0 SBUS 任务
  Serial.println("Creating Core 0 SBUS Task...");
  xTaskCreatePinnedToCore(
    sbusTask,            // 任务函数
    "SBUSTx",            // 任务名称
    2048,                // 堆栈大小
    NULL,                // 任务参数
    5,                   // 中等优先级
    &g_sbus_task_handle, // 任务句柄
    0                    // 固定在 Core 0
  );

  // 9. 设置 8kHz 硬件定时器
  Serial.println("Starting 8kHz PID Timer...");
  // *** 已修复: ESP32 Core 3.x 定时器 API ***
  pid_timer = timerBegin(1000000); // 1MHz (1µs 精度)
  timerAttachInterrupt(pid_timer, &onPidTimer);
  // *** 已修复: 添加第4个参数 (0) 用于自动重载 ***
  timerAlarm(pid_timer, PID_LOOP_PERIOD_US, true, 0); // 125µs, 自动重载, 0 = 无限
  timerStart(pid_timer);
  
  onboard_led.fill(onboard_led.Color(0, 255, 0)); // 绿色表示运行
  onboard_led.show();
  Serial.println("Setup Complete. Running.");
}

// ======================================================================
// ==                         LOOP (CORE 0)
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
    onboard_led.fill(onboard_led.Color(255, 0, 0)); // 自动调参时为红色
  } else {
    onboard_led.fill(onboard_led.Color(0, 255, 0)); // 正常运行时为绿色
  }
  onboard_led.show();

  // 允许其他任务运行
  vTaskDelay(pdMS_TO_TICKS(5));
}


// ======================================================================
// ==                    CORE 1 - PID LOOP TASK
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
  
  while(true) {
    // 等待 onPidTimer 的通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // --- 125µs 预算开始 ---

    // 1. 读取所有 ADC
    // !! 警告: 这是环路的主要瓶颈 !!
    // !! 要达到 8kHz, 必须用 DMA ADC 替换此部分 !!
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

      // 应用硬限位
      if (at_min_limit && target_velocity < 0) {
        target_velocity = 0; // 在最小处，不允许再反转
      } else if (at_max_limit && target_velocity > 0) {
        target_velocity = 0; // 在最大处，不允许再正转
      }
      
      // 应用速度死区
      if (abs(target_velocity) < PID_VELOCITY_DEADZONE) {
         target_velocity = 0;
      }

      // 4. 计算最终PWM (us)
      float pwm_us = 1500.0 + target_velocity;
      
      // 最终钳位
      pwm_us = constrain(pwm_us, 1000.0, 2000.0);

      // 5. 更新 LEDC (硬件 PWM)
      // (20000µs = 50Hz 周期)
      uint32_t duty = (uint32_t)((pwm_us / 20000.0) * 65535.0);
      
      // *** 已修复: 使用存储的 g_ledc_channels 数组 ***
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
    g_ledc_channels[i] = ledcAttach(PWM_PINS[i], 50, 16); // 引脚, 50Hz, 16位
    
    // 设置初始中点
    uint32_t center_duty = (uint32_t)((1500.0 / 20000.0) * 65535.0);
    ledcWrite(g_ledc_channels[i], center_duty); // 使用返回的通道
    g_final_pwm_us[i] = 1500.0;
  }
}

// 设置 8 个 PID 控制器
void setup_pid_controllers() {
  Serial.println("Setting up 8 QuickPID controllers...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 链接 QuickPID 到我们的全局变量
    pid_controllers[i] = QuickPID(
      &g_current_adc[i],      // *Input
      &g_pid_velocity_out[i], // *Output
      &g_target_adc[i],       // *Setpoint
      DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, // Tunings
      QuickPID::pMode::pOnError,      // Proportional on Error
      QuickPID::dMode::dOnMeas,       // Derivative on Measurement
      QuickPID::iAwMode::iAwCondition, // Anti-windup
      QuickPID::Action::direct        // Direct action
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
// ==                 CORE 0 - COMMUNICATION & OTHER
// ======================================================================

// -----------------
// USB Serial
// -----------------

// *** 新增: USB 轮询函数，在 loop() 中调用 ***
void handle_usb_input() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    
    if (g_rx_buffer_idx == 0 && b != 'S') {
      continue; // 等待包头
    }
    
    g_rx_buffer[g_rx_buffer_idx++] = b;

    if (g_rx_buffer_idx >= RX_BUFFER_SIZE) {
      // 缓冲区溢出，重置
      g_rx_buffer_idx = 0;
      continue;
    }

    if (b == 'E') {
      // 收到包尾
      parseUsbPacket();
      g_rx_buffer_idx = 0; // 重置
    }
  }
}

// 解析收到的数据包
void parseUsbPacket() {
  // 包格式: 'S' + <cmd_char> + ... + 'E'
  // 1. 设置目标: 'T' + <servo_idx> (1 byte) + <target_H> (1 byte) + <target_L> (1 byte) + 'E' (总共 5 字节)
  //    例如: S T 0 0x07 0xD0 E  -> 设置舵机0的目标为 2000 (0x07D0)
  // 2. 启动调参: 'A' + <servo_idx> (1 byte) + 'E' (总共 3 字节)
  //    例如: S A 1 E -> 开始自动调谐舵机 1
  
  if (g_rx_buffer[0] != 'S' || g_rx_buffer[g_rx_buffer_idx - 1] != 'E') {
    return; // 包无效
  }

  char cmd = g_rx_buffer[1];
  
  if (cmd == 'T' && g_rx_buffer_idx == 6) {
    // 设置目标
    int servo_idx = g_rx_buffer[2];
    if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
      uint16_t target_val = (g_rx_buffer[3] << 8) | g_rx_buffer[4];
      
      // 限制目标在安全范围内
      target_val = constrain(target_val, g_servo_limits[servo_idx].min_adc, g_servo_limits[servo_idx].max_adc);
      
      // 安全地更新目标值
      portENTER_CRITICAL(&g_pid_mux);
      g_target_adc[servo_idx] = (float)target_val;
      portEXIT_CRITICAL(&g_pid_mux);
      
      Serial.printf("Set Servo %d Target to %d\n", servo_idx, target_val);
    }
  } 
  else if (cmd == 'A' && g_rx_buffer_idx == 3) {
    // 自动调参
    int servo_idx = g_rx_buffer[2];
    if (servo_idx >= 0 && servo_idx < NUM_SERVOS) {
      if (!g_is_autotuning) {
        Serial.printf("Starting Autotune for Servo %d...\n", servo_idx);
        // *** 已修复: 拼写错误 (servo_index -> servo_idx) ***
        start_autotune(servo_idx);
      } else {
        Serial.println("Error: Autotune already in progress.");
      }
    }
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
  memcpy(pwm_copy, (void*)g_final_pwm_us, sizeof(pwm_copy));
  memcpy(tgt_copy, g_target_adc, sizeof(tgt_copy));
  portEXIT_CRITICAL(&g_pid_mux);

  // 打印复制的数据
  // 格式: [S0: ADC=1234, TGT=1235, PWM=1500]
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("[S%d: ADC=%.0f, TGT=%.0f, PWM=%.0f] ",
      i,
      adc_copy[i],
      tgt_copy[i],
      pwm_copy[i]
    );
  }
  Serial.println();
}

// -----------------
// SBUS
// -----------------

void setup_sbus() {
  // 配置 SBUS 串口
  // *** 已修复: 使用 'invert'布尔参数 (true) ***
  SbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_PIN, true); // RX, TX, invert
  
  // 初始化通道数据
  for(int i=0; i<16; i++) {
    g_sbus_channels[i] = SBUS_CENTER_VAL;
  }
}

// SBUS 任务 (运行在 Core 0)
void sbusTask(void *pvParameters) {
  Serial.println("sbusTask running on Core 0.");
  uint8_t sbus_frame[25];
  
  while(true) {
    // 1. 准备 SBUS 帧
    sbus_frame[0] = 0x0F; // 帧头
    
    // 11位通道数据被打包到 22 个字节中
    // 我们需要安全地读取 g_sbus_channels
    uint16_t channels_copy[16];
    portENTER_CRITICAL(&g_pid_mux);
    memcpy(channels_copy, g_sbus_channels, 8 * sizeof(uint16_t)); // 只复制前8个
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
    for(int i=12; i<23; i++) {
        sbus_frame[i] = 0x00;
    }

    sbus_frame[23] = 0x00; // 标志位
    sbus_frame[24] = 0x00; // 帧尾

    // 2. 发送
    SbusSerial.write(sbus_frame, 25);
    
    // 3. 等待下一个帧周期
    vTaskDelay(pdMS_TO_TICKS(SBUS_FRAME_PERIOD_MS));
  }
}

// ======================================================================
// ==                       AUTOTUNE (BLOCKING)
// ======================================================================

// 自动调参输入函数 (供 autotune 库回调)
double autotune_input_func(int servo_index) {
  return (double)analogRead(ADC_PINS[servo_index]);
}

// 自动调参输出函数 (供 autotune 库回调)
void autotune_output_func(double output) {
  // autotune 库的输出被配置为 1000-2000
  float pwm_us = constrain(output, 1000.0, 2000.0);
  
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
      g_servo_limits[servo_index].center_adc, // 目标值
      10000, // 环路间隔 (us) -> 10ms
      1000, 2000 // 输出范围 (pwm us)
  );

  g_autotuner.setPID(g_tuning_pid_instance);
  g_autotuner.setTargetValue(g_servo_limits[servo_index].center_adc);
  g_autotuner.setConstrains(1000.0, 2000.0); // 输出是 PWM us
  g_autotuner.setLoopInterval(10000); // 10ms
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