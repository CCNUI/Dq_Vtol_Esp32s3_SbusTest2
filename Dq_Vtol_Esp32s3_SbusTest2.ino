//@FILE Dq_Vtol_Esp32s3_SbusTest2.ino START
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

#include "pid.h"
#include "QuickPID.h"
#include "pid-autotune.h"
#include "config.h"

// ======================================================================
// ==                      GLOBAL VARIABLES
// ======================================================================

// -- Core 1 (PID) 变量 --
// 数组大小为 9 (NUM_SERVOS + 1)，索引 0 废弃，使用 1-8
QuickPID pid_controllers[NUM_SERVOS + 1];
// PID 计算的输入、输出和设定点
float g_current_adc[NUM_SERVOS + 1];      // 输入
float g_pid_velocity_out[NUM_SERVOS + 1]; // 输出
float g_target_adc[NUM_SERVOS + 1];
// 设定点

// 摆动锁定计数器
volatile int g_oscillation_count[NUM_SERVOS + 1];
volatile int g_last_velocity_sign[NUM_SERVOS + 1];
volatile bool g_servo_locked[NUM_SERVOS + 1];
// 最终 PWM 脉宽
volatile float g_final_pwm_us[NUM_SERVOS + 1];

// 硬件定时器
hw_timer_t *pid_timer = NULL;
// Core 1 PID 任务句柄
TaskHandle_t g_pid_task_handle = NULL;
// 互斥锁
portMUX_TYPE g_pid_mux = portMUX_INITIALIZER_UNLOCKED;
// -- Core 0 (Comms/LED) 变量 --

Adafruit_NeoPixel onboard_led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial SbusSerial(1);
// SBUS 通道数据
// 数组大小 17，索引 0 废弃，使用 1-16
uint16_t g_sbus_channels[17];

TaskHandle_t g_sbus_task_handle = NULL;
unsigned long g_last_telemetry_time = 0;
#define RX_BUFFER_SIZE 64
char g_rx_buffer[RX_BUFFER_SIZE];
uint8_t g_rx_buffer_idx = 0;

// Autotune
pid_tuner g_autotuner;
PID g_tuning_pid_instance;
volatile bool g_is_autotuning = false;
volatile int g_autotune_servo_idx = 1; // 默认为 1

// ======================================================================
// ==                       PROTOTYPES
// ======================================================================

void IRAM_ATTR onPidTimer();
void pidLoopTask(void *pvParameters);
void setup_pid_controllers();
void setup_pwm();
void read_all_adcs();

void sbusTask(void *pvParameters);
void setup_sbus();
void handle_telemetry();
void handle_usb_input();
void parseUsbCommand();
void start_autotune(int servo_index);
double autotune_input_func(int servo_index);
void autotune_output_func(double output);

// ======================================================================
// ==                      SETUP (CORE 0)
// ======================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(2000); 

  Serial.println("ESP32-S3 PID Servo Controller Booting...");
  Serial.println("System uses 1-based indexing (Servo 1 - 8). Index 0 is wasted.");
  Serial.printf("Core Version: %s\n", ESP.getSdkVersion());
  
  Serial.println("Commands:");
  Serial.println("  P <id> <Kp> <Ki> <Kd>   (Set PID, id: 1-8)");
  Serial.println("  T <id> <target_adc>     (Set Target & Unlock, id: 1-8)");
  Serial.println("  A <id>                  (Start Autotune, id: 1-8)");
#if DEBUG_FIXED_PWM_OUTPUT == 1
  Serial.println("--- WARNING: DEBUG_FIXED_PWM_OUTPUT IS ENABLED ---");
  Serial.println("--- PID Disabled. Initial: S6=1400, S7=1500, S8=1600");
  Serial.println("--- Use 'S <id> <pwm>' to override.");
#else
  Serial.println("--- INFO: Normal PID Mode. Servos locked at 1500us.");
  Serial.println("--- Send 'T <id> <target>' to unlock.");
#endif

  // 设置 ADC
  for (int i = 1; i <= NUM_SERVOS; i++) {
    analogSetAttenuation(ADC_11db);
    pinMode(ADC_PINS[i], INPUT);
  }

  // 1. 设置 LEDC
  setup_pwm();
  // 2. 初始化变量
  for (int i = 1; i <= NUM_SERVOS; i++) {
    g_target_adc[i] = g_servo_limits[i].center_adc;
    g_oscillation_count[i] = 0;
    g_last_velocity_sign[i] = 0;
    g_servo_locked[i] = true; 
  }

  // 3. 设置 PID
  setup_pid_controllers();
  // 4. 设置 SBUS
  setup_sbus();

  // 5. LED
  onboard_led.begin();
  onboard_led.setBrightness(LED_SIG_BRIGHTNESS);
  onboard_led.fill(onboard_led.Color(0, 0, 255));
  onboard_led.show();
  // 6. 任务
  Serial.println("Creating Core 1 PID Task...");
  xTaskCreatePinnedToCore(pidLoopTask, "PIDLoop", 8192, NULL, configMAX_PRIORITIES - 1, &g_pid_task_handle, 1);
  Serial.println("Creating Core 0 SBUS Task...");
  xTaskCreatePinnedToCore(sbusTask, "SBUSTx", 2048, NULL, 5, &g_sbus_task_handle, 0);
  // 7. 定时器
  Serial.printf("Starting PID Timer ( %d us )...\n", PID_LOOP_PERIOD_US);
  pid_timer = timerBegin(1000000);
  timerAttachInterrupt(pid_timer, &onPidTimer);
  timerAlarm(pid_timer, PID_LOOP_PERIOD_US, true, 0);
  timerStart(pid_timer);

  onboard_led.fill(onboard_led.Color(0, 255, 0));
  onboard_led.show();
  Serial.println("Setup Complete.");
}

// ======================================================================
// ==                      LOOP (CORE 0)
// ======================================================================
void loop() {
  if (!g_is_autotuning) {
    handle_usb_input();
    handle_telemetry();
  }

  // 呼吸灯
  uint8_t brightness = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * (float)LED_MAX_BRIGHTNESS;
  onboard_led.setBrightness(brightness);
  if (g_is_autotuning) {
    onboard_led.fill(onboard_led.Color(255, 0, 0));
  } else {
    onboard_led.fill(onboard_led.Color(0, 255, 0));
  }
  onboard_led.show();
  vTaskDelay(pdMS_TO_TICKS(5));
}

// ======================================================================
// ==                 CORE 1 - PID LOOP TASK
// ======================================================================

void IRAM_ATTR onPidTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(g_pid_task_handle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void pidLoopTask(void *pvParameters) {
  Serial.println("pidLoopTask running on Core 1.");
#if DEBUG_FIXED_PWM_OUTPUT == 1
  // --- DEBUG MODE ---
  Serial.println("PID Task: DEBUG_FIXED_PWM_OUTPUT mode.");
  // 初始化测试值 (Servo 6, 7, 8)
  portENTER_CRITICAL(&g_pid_mux);
  g_final_pwm_us[6] = 1400.0;
  g_final_pwm_us[7] = 1500.0;
  g_final_pwm_us[8] = 1600.0;
  portEXIT_CRITICAL(&g_pid_mux);
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    float local_pwm_us[NUM_SERVOS + 1];
    portENTER_CRITICAL(&g_pid_mux);
    // 从索引 1 开始复制
    memcpy(&local_pwm_us[1], (const void*)&g_final_pwm_us[1], NUM_SERVOS * sizeof(float));
    portEXIT_CRITICAL(&g_pid_mux);
    for (int i = 1; i <= NUM_SERVOS; i++) {
      // [FIX] 200Hz 修改：周期从 20000.0 改为 5000.0
      uint32_t duty = (uint32_t)((local_pwm_us[i] / 5000.0) * 16383.0);
      ledcWrite(PWM_PINS[i], duty); // PWM_PINS[i] 正确对应

      portENTER_CRITICAL(&g_pid_mux);
      // 直接映射: Servo i -> SBUS Channel i
      // 这里的 g_sbus_channels[i] 实际上对应 SBUS 的第 i 通道
      g_sbus_channels[i] = map(local_pwm_us[i], 1000, 2000, SBUS_MIN_VAL, SBUS_MAX_VAL);
      portEXIT_CRITICAL(&g_pid_mux);
    }
  }

#else
  // --- NORMAL PID MODE ---
  // [FIX] 200Hz 修改：周期从 20000.0 改为 5000.0
  const uint32_t center_duty = (uint32_t)((1500.0 / 5000.0) * 16383.0);
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // 1. 读取 ADC
    read_all_adcs();
    // 2. 运行 PID (循环 1 到 8)
    for (int i = 1; i <= NUM_SERVOS; i++) {

      // Check lock
      if (g_servo_locked[i]) {
        portENTER_CRITICAL(&g_pid_mux);
        g_final_pwm_us[i] = 1500.0;
        g_sbus_channels[i] = SBUS_CENTER_VAL; // Write to SBUS Ch i
        portEXIT_CRITICAL(&g_pid_mux);
        ledcWrite(PWM_PINS[i], center_duty);
        continue;
      }

      // Check limits
      bool at_min_limit = (g_current_adc[i] <= g_servo_limits[i].min_adc);
      bool at_max_limit = (g_current_adc[i] >= g_servo_limits[i].max_adc);

      pid_controllers[i].Compute();

      // Oscillation Logic
      int current_sign = 0;
      if (g_pid_velocity_out[i] > PID_VELOCITY_DEADZONE) current_sign = 1;
      else if (g_pid_velocity_out[i] < -PID_VELOCITY_DEADZONE) current_sign = -1;
      if (current_sign != 0 && current_sign != g_last_velocity_sign[i] && g_last_velocity_sign[i] != 0) {
        portENTER_CRITICAL(&g_pid_mux);
        g_oscillation_count[i]++;
        portEXIT_CRITICAL(&g_pid_mux);
      }

      if (current_sign != 0) {
          portENTER_CRITICAL(&g_pid_mux);
          g_last_velocity_sign[i] = current_sign;
          portEXIT_CRITICAL(&g_pid_mux);
      }
      
      if (g_oscillation_count[i] >= PID_OSCILLATION_LIMIT) {
        portENTER_CRITICAL(&g_pid_mux);
        g_servo_locked[i] = true;
        g_target_adc[i] = g_current_adc[i];
        g_final_pwm_us[i] = 1500.0;
        g_sbus_channels[i] = SBUS_CENTER_VAL;
        portEXIT_CRITICAL(&g_pid_mux);
        ledcWrite(PWM_PINS[i], center_duty);
        continue;
      }

      // Calculate output
      float target_velocity = g_pid_velocity_out[i];
      bool is_reversed = (g_pid_reverse_action[i] == 1);
      
      if (is_reversed) {
        if (at_min_limit && target_velocity > 0) target_velocity = 0;
        else if (at_max_limit && target_velocity < 0) target_velocity = 0;
      } else {
        if (at_min_limit && target_velocity < 0) target_velocity = 0;
        else if (at_max_limit && target_velocity > 0) target_velocity = 0;
      }

      // Deadzone Jump
      if (target_velocity > 0.0) target_velocity += PID_VELOCITY_DEADZONE;
      else if (target_velocity < 0.0) target_velocity -= PID_VELOCITY_DEADZONE;

      // PWM Calculation
      float pwm_us = 1500.0 + target_velocity;
      pwm_us = constrain(pwm_us, 1000.0, 2000.0);
      
      // [FIX] 200Hz 修改：周期从 20000.0 改为 5000.0
      uint32_t duty = (uint32_t)((pwm_us / 5000.0) * 16383.0);
      ledcWrite(PWM_PINS[i], duty);

      portENTER_CRITICAL(&g_pid_mux);
      g_final_pwm_us[i] = pwm_us;
      // 直接映射到 SBUS 索引 i
      g_sbus_channels[i] = map(pwm_us, 1000, 2000, SBUS_MIN_VAL, SBUS_MAX_VAL);
      portEXIT_CRITICAL(&g_pid_mux);
    }
  }
#endif
}

void IRAM_ATTR read_all_adcs() {
  for (int i = 1; i <= NUM_SERVOS; i++) {
    portENTER_CRITICAL(&g_pid_mux);
    g_current_adc[i] = (float)analogRead(ADC_PINS[i]);
    portEXIT_CRITICAL(&g_pid_mux);
  }
}

void setup_pwm() {
  // [FIX] 更新打印信息
  Serial.println("Setting up 8 PWM pins (200 Hz, 14-bit)...");
  for (int i = 1; i <= NUM_SERVOS; i++) {
    // [FIX] 200Hz 修改：将频率参数从 50 改为 200
    if (ledcAttach(PWM_PINS[i], 200, 14)) {
        Serial.printf("[INFO] PWM Attached to GPIO %d (Servo %d)\n", PWM_PINS[i], i);
    } else {
        Serial.printf("[FAIL] PWM Failed on GPIO %d (Servo %d)\n", PWM_PINS[i], i);
    }
    // [FIX] 200Hz 修改：周期从 20000.0 改为 5000.0
    uint32_t center_duty = (uint32_t)((1500.0 / 5000.0) * 16383.0);
    ledcWrite(PWM_PINS[i], center_duty); 
    g_final_pwm_us[i] = 1500.0;
  }
}

void setup_pid_controllers() {
  Serial.println("Setting up 8 QuickPID controllers...");
  for (int i = 1; i <= NUM_SERVOS; i++) {
    pid_controllers[i] = QuickPID(
      &g_current_adc[i],
      &g_pid_velocity_out[i],
      &g_target_adc[i],
      DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
      QuickPID::pMode::pOnError,
      QuickPID::dMode::dOnMeas,
      QuickPID::iAwMode::iAwCondition,
      (QuickPID::Action)g_pid_reverse_action[i]
    );
    pid_controllers[i].SetSampleTimeUs(PID_LOOP_PERIOD_US);
    pid_controllers[i].SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    pid_controllers[i].SetMode(QuickPID::Control::automatic);
  }
}

// ======================================================================
// ==               CORE 0 - COMMUNICATION & OTHER
// ======================================================================

void handle_usb_input() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
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

void parseUsbCommand() {
  // 格式 (编号从 1 开始，直接使用):
  // P <id> <kp> <ki> <kd>
  // T <id> <target>
  // A <id>
  // S <id> <pwm_us>

  char cmd_char;
  int items = sscanf(g_rx_buffer, "%c", &cmd_char);
  if (items < 1) return;
  if (cmd_char == 'P') {
    int idx = 0;
    float kp = 0, ki = 0, kd = 0;
    if (sscanf(g_rx_buffer, "P %d %f %f %f", &idx, &kp, &ki, &kd) == 4) {
      if (idx >= 1 && idx <= NUM_SERVOS) {
        portENTER_CRITICAL(&g_pid_mux);
        pid_controllers[idx].SetTunings(kp, ki, kd);
        portEXIT_CRITICAL(&g_pid_mux);
        Serial.printf("Set Servo %d Tunings: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", idx, kp, ki, kd);
      } else {
        Serial.printf("Error: Invalid servo id %d (1-8)\n", idx);
      }
    } else {
      Serial.println("Error: Format P <id> <kp> <ki> <kd>");
    }

  } else if (cmd_char == 'T') {
    int idx = 0;
    int target_int = 0;
    if (sscanf(g_rx_buffer, "T %d %d", &idx, &target_int) == 2) {
      if (idx >= 1 && idx <= NUM_SERVOS) {
        uint16_t target = constrain(target_int, g_servo_limits[idx].min_adc, g_servo_limits[idx].max_adc);
        portENTER_CRITICAL(&g_pid_mux);
        g_target_adc[idx] = (float)target;
        g_oscillation_count[idx] = 0;
        g_last_velocity_sign[idx] = 0;
        g_servo_locked[idx] = false;
        portEXIT_CRITICAL(&g_pid_mux);
        Serial.printf("Set Servo %d Target to %d (Unlocked)\n", idx, target);
      } else {
        Serial.printf("Error: Invalid servo id %d (1-8)\n", idx);
      }
    } else {
      Serial.println("Error: Format T <id> <target>");
    }

  } else if (cmd_char == 'A') {
    int idx = 0;
    if (sscanf(g_rx_buffer, "A %d", &idx) == 1) {
      if (idx >= 1 && idx <= NUM_SERVOS) {
        if (!g_is_autotuning) {
          Serial.printf("Starting Autotune for Servo %d...\n", idx);
          start_autotune(idx);
        } else {
          Serial.println("Error: Autotune in progress.");
        }
      } else {
        Serial.printf("Error: Invalid servo id %d (1-8)\n", idx);
      }
    } else {
      Serial.println("Error: Format A <id>");
    }
 
  } else if (cmd_char == 'S') {
  #if DEBUG_FIXED_PWM_OUTPUT == 1
    int idx = 0;
    int pwm_val = 0;
    if (sscanf(g_rx_buffer, "S %d %d", &idx, &pwm_val) == 2) {
      if (idx >= 1 && idx <= NUM_SERVOS) {
        float pwm_us = (float)constrain(pwm_val, 1000, 2000);
        portENTER_CRITICAL(&g_pid_mux);
        g_final_pwm_us[idx] = pwm_us;
        portEXIT_CRITICAL(&g_pid_mux);
        Serial.printf("DEBUG: Set Servo %d PWM to %.0f\n", idx, pwm_us);
      } else {
        Serial.printf("Error: Invalid servo id %d (1-8)\n", idx);
      }
    } else {
      Serial.println("Error: Format S <id> <pwm>");
    }
  #else
    Serial.println("Error: Enable DEBUG_FIXED_PWM_OUTPUT first.");
  #endif
  }
}

void handle_telemetry() {
  if (millis() - g_last_telemetry_time < (1000 / TELEMETRY_OUTPUT_HZ)) return;
  g_last_telemetry_time = millis();
  // 复制数组 (注意大小 NUM_SERVOS + 1)
  float adc_copy[NUM_SERVOS + 1];
  float pwm_copy[NUM_SERVOS + 1];
  float tgt_copy[NUM_SERVOS + 1];
  int count_copy[NUM_SERVOS + 1];

  portENTER_CRITICAL(&g_pid_mux);
  memcpy(adc_copy, g_current_adc, sizeof(adc_copy));
  memcpy(pwm_copy, (void *)g_final_pwm_us, sizeof(pwm_copy));
  memcpy(tgt_copy, g_target_adc, sizeof(tgt_copy));
  memcpy(count_copy, (void*)g_oscillation_count, sizeof(count_copy));
  portEXIT_CRITICAL(&g_pid_mux);
  // 直接使用索引 1-8 打印
  for (int i = 1; i <= NUM_SERVOS; i++) {
    Serial.printf("[S%d: ADC=%.0f, TGT=%.0f, PWM=%.0f, O=%d] ", 
                  i, adc_copy[i], tgt_copy[i], pwm_copy[i], count_copy[i]);
  }
  Serial.println();
}

// -----------------
// SBUS
// -----------------

void setup_sbus() {
  SbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_PIN, true);
  // 初始化 1-16
  for (int i = 1; i <= 16; i++) {
    g_sbus_channels[i] = SBUS_CENTER_VAL;
  }
}

void sbusTask(void *pvParameters) {
  Serial.println("sbusTask running on Core 0.");
  uint8_t sbus_frame[25];
  while (true) {
    sbus_frame[0] = 0x0F;

    // 局部复制。SBUS 帧生成逻辑是基于 0 索引的位操作
    // 我们需要将 g_sbus_channels[1..8] 复制到 channels_copy[0..7]
    uint16_t channels_copy[16];
    // 临时数组，0-indexed
    
    portENTER_CRITICAL(&g_pid_mux);
    // [重要] 从 g_sbus_channels[1] 开始复制 8 个通道
    memcpy(channels_copy, (const void*)&g_sbus_channels[1], 8 * sizeof(uint16_t));
    portEXIT_CRITICAL(&g_pid_mux);
    // 下面的位操作代码保持不变，因为它操作的是 channels_copy[0]
    // 此时 channels_copy[0] 实际上持有 g_sbus_channels[1] (Servo 1) 的值
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
    // 清零 9-16
    for (int i = 12; i < 23; i++) sbus_frame[i] = 0x00;
    sbus_frame[23] = 0x00;
    sbus_frame[24] = 0x00;

    SbusSerial.write(sbus_frame, 25);
    vTaskDelay(pdMS_TO_TICKS(SBUS_FRAME_PERIOD_MS));
  }
}

// ======================================================================
// ==                    AUTOTUNE (BLOCKING)
// ======================================================================

double autotune_input_func(int servo_index) {
  // 直接使用 servo_index (1-8)
  return (double)analogRead(ADC_PINS[servo_index]);
}

void autotune_output_func(double output) {
  yield();
  float pwm_us = constrain(output, 1000.0, 2000.0);
  if (g_pid_reverse_action[g_autotune_servo_idx] == 1) {
    pwm_us = 3000.0 - pwm_us;
  }

  float velocity_out = pwm_us - 1500.0;
  if (velocity_out > 0.0) velocity_out += PID_VELOCITY_DEADZONE;
  else if (velocity_out < 0.0) velocity_out -= PID_VELOCITY_DEADZONE;
 
  pwm_us = constrain(1500.0 + velocity_out, 1000.0, 2000.0);
  
  // [FIX] 200Hz 修改：周期从 20000.0 改为 5000.0
  uint32_t duty = (uint32_t)((pwm_us / 5000.0) * 16383.0);
  // 直接使用全局索引
  ledcWrite(PWM_PINS[g_autotune_servo_idx], duty);
}

void start_autotune(int servo_index) {
  if (g_is_autotuning) return;
  g_is_autotuning = true;
  g_autotune_servo_idx = servo_index;
  // 存储 1-8

  Serial.printf("Suspending PID Task on Core 1 for Autotune (Servo %d).\n", servo_index);
  vTaskSuspend(g_pid_task_handle);

  g_tuning_pid_instance.setSetPoint(g_servo_limits[servo_index].center_adc);
  g_tuning_pid_instance.setConstrains(1000.0, 2000.0);

  g_autotuner.setPID(g_tuning_pid_instance);
  g_autotuner.setTargetValue(g_servo_limits[servo_index].center_adc);
  g_autotuner.setConstrains(1000.0, 2000.0);
  g_autotuner.setLoopInterval(10000);
  g_autotuner.setTuningCycles(10);
  g_autotuner.setMode(pid_tuner::mode_t::CLASSIC_PID);

  if (g_pid_reverse_action[servo_index] == 1) {
    Serial.println("Autotune: Logic inverted (REVERSE).");
  } else {
    Serial.println("Autotune: Logic DIRECT.");
  }

  Serial.println("Starting tuning loop...");
  g_autotuner.tune(autotune_input_func, servo_index, autotune_output_func);

  Serial.println("Tuning complete.");
  float Kp = g_autotuner.getKp();
  float Ki = g_autotuner.getKi();
  float Kd = g_autotuner.getKd();
  Serial.printf("Tuned: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", Kp, Ki, Kd);

  portENTER_CRITICAL(&g_pid_mux);
  pid_controllers[servo_index].SetTunings(Kp, Ki, Kd);
  portEXIT_CRITICAL(&g_pid_mux);
  Serial.printf("Applied to QuickPID Servo %d.\n", servo_index);

  vTaskResume(g_pid_task_handle);
  Serial.println("Resumed PID Task.");

  g_is_autotuning = false;
}
//@FILE Dq_Vtol_Esp32s3_SbusTest2.ino END