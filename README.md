# ESP32-S3 高性能PID舵机控制器
### GitHub README.md

**中文:**
基于ESP32-S3的8通道闭环PID舵机控制器，集成SBUS输出、200Hz高速环路、自动调参和串口指令遥测。

**English:**
ESP32-S3 8-Channel closed-loop PID servo controller with SBUS output, 200Hz high-speed loop, autotuning, and serial command telemetry.

这是一个基于 ESP32-S3 DevKitC-1 开发的8通道高性能闭环PID舵机控制器。项目利用 ESP32-S3 的双核特性，将高速PID环路（200Hz）和通信/SBUS任务分离在不同的核心上运行，以实现高精度和高响应速度。

控制器通过ADC读取8个通道的舵机位置反馈（例如电位器），计算PID后输出8路PWM信号（200Hz）以闭环方式驱动舵机。同时，它还将PWM信号实时打包成SBUS帧，通过独立串口（UART）发送出去。

## 已知问题（Commit: Third Succeed Run)
* PID AutoTune不起作用：长时间无法结束AutoTune。

## 核心特性

* **多核架构:** 8路PID闭环控制运行在 Core 1，串口通信和SBUS输出任务运行在 Core 0，确保PID环路不受干扰。
* **高速环路:** PWM 输出频率为 200Hz，PID环路同步运行在 200Hz (5000µs 周期)，适用于需要快速响应的舵机。
* **双路输出:** 8路 PWM 信号直接输出，同时生成 SBUS 帧信号 (UART1, 100000 8E2)，可用于遥测或菊花链式连接。
* **高级PID控制:**
    * 使用 `QuickPID` 库。
    * 支持每个通道单独配置舵机正反向 (`g_pid_reverse_action`)。
    * 支持死区补偿 (Deadzone "Jump") 逻辑 (`PID_VELOCITY_DEADZONE`)，以克服舵机启动死区。
* **实时串口控制:** 通过 USB 串口 (115200) 发送ASCII指令，实时调整PID参数或设置目标点。
* **自动PID调参:** 集成 `pid-autotune` 库，可通过 `A <idx>` 指令对特定舵机进行PID自动调参。
* **防抖锁定 (Jitter Lock):** 舵机在达到目标点后，若摆动次数超过设定值 (`PID_OSCILLATION_LIMIT`)，将自动锁定在 1500us，防止高增益P值导致的高频抖动和噪音，直到收到下一个 `T` 命令。
* **Debug模式:** 包含一个编译时标志 `DEBUG_FIXED_PWM_OUTPUT`，用于绕过PID，直接进行原始PWM测试。
* **实时遥测:** 实时遥测输出，包括 ADC 读数、目标值、PWM 输出和摆动计数值 (Oscillation Count)。

## 硬件配置 (`config.h`)

* **开发板:** ESP32S3-DEVKIT-C-1
* **ADC 反馈引脚:**
    * `S0` - `S7`: GPIO 1, 2, 4, 5, 6, 7, 8, 9
* **PWM 输出引脚:**
    * `S0` - `S7`: GPIO 42, 41, 40, 39, 38, 37, 36, 35
* **SBUS TX 引脚:** GPIO 14
* **状态指示灯:** GPIO 48 (板载 NeoPixel)

## 如何使用

1.  **硬件连接:** 根据 `config.h` 中的引脚定义连接8个舵机和8个电位器（或其他反馈传感器）。
2.  **配置 `config.h`:**
    * `PWM_PINS`, `ADC_PINS`: 确认引脚。
    * `g_pid_reverse_action`: 为反向舵机设置 `1`。
    * `g_servo_limits`: **(重要)** 设置每个舵机的ADC物理极限范围。
    * `PID_VELOCITY_DEADZONE`: 设置舵机死区补偿值 (例如 100.0 对应 1500±100us)。
    * `PID_OSCILLATION_LIMIT`: 设置锁定前的摆动次数 (例如 20)。
    * `DEBUG_FIXED_PWM_OUTPUT`: 设为 `0` 启用PID，设为 `1` 启用舵机测试模式。
3.  **编译和上传:** 使用 Arduino IDE 编译并上传。
4.  **操作:**
    * 打开串口监视器，波特率 115200，**确保发送时包含换行符 (Newline)**。
    * 固件启动时，所有舵机默认锁定在 1500us (遥测显示 `O=0`)。
    * 发送 `T 2 2000` (设置2号舵机目标为ADC 2000)。
    * 舵机将移动，遥测中 `O=...` 开始变化。
    * 舵机在目标点附近摆动，`O` 计数增加。
    * 当 `O` 达到 20 (或 `PID_OSCILLATION_LIMIT` 的值) 时，舵机锁定在 1500us，`O` 停止增加。
    * 发送新的 `T` 命令将重复此过程。

## 串口指令 API (ASCII)

**重要:** 所有指令均以**换行符 (Newline)** 结尾。

---
### 设置目标 (解锁)
设置舵机目标ADC位置，并解锁“防抖锁定”。

**格式:** `T <idx> <target_adc>`
* `<idx>`: 舵机索引 (0-7)
* `<target_adc>`: 目标ADC读数 (例如 1000-3000)

**示例:**
```

T 0 2500

```
> 设置舵机0的目标为ADC 2500，并解锁。

---
### 设置 PID
实时设置指定通道的PID增益。

**格式:** `P <idx> <Kp> <Ki> <Kd>`
* `<idx>`: 舵机索引 (0-7)
* `<Kp>`, `<Ki>`, `<Kd>`: PID浮点数值

**示例:**
```

P 0 1.5 0.05 0.1

```
> 设置舵机0的 Kp=1.5, Ki=0.05, Kd=0.1。

---
### 自动调参
启动指定通道的自动PID调参。

**格式:** `A <idx>`
* `<idx>`: 舵机索引 (0-7)

**示例:**
```

A 1

```
> 开始对舵机1进行自动调参。此过程将阻塞主循环（Core 0）直到调参完成，并通过串口输出结果。

---
### 舵机测试 (Debug模式)
**注意:** 仅在 `config.h` 中 `DEBUG_FIXED_PWM_OUTPUT` 设为 `1` 时有效。

**格式:** `S <idx> <pwm_us>`
* `<idx>`: 舵机索引 (0-7)
* `<pwm_us>`: 原始PWM脉宽 (1000-2000)

**示例:**
```

S 5 1400

```
> 绕过PID，强制设置舵机5的PWM为 1400us。

## 遥测数据格式

遥测数据以 10Hz 的频率打印到串口，格式如下：

`[S<idx>: ADC=<adc_val>, TGT=<tgt_val>, PWM=<pwm_val>, O=<count>]`

* `S<idx>`: 舵机索引 (0-7)
* `ADC`: 当前ADC反馈值
* `TGT`: 当前ADC目标值
* `PWM`: 最终输出的PWM脉宽 (us)
* `O`: 当前的摆动计数值 (达到 `PID_OSCILLATION_LIMIT` 时将锁定)

## 依赖库

* `QuickPID`
* `pid-autotune` (已包含在 `libraries` 目录中)
* `Adafruit_NeoPixel`
