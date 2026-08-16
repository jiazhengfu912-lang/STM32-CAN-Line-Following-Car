# STM32 CAN Line-Following Car

基于 STM32F103 的差速循迹小车固件工程。主工程通过 CAN 控制 DJI M2006 电机，读取八路循迹传感器，并经 USART3 与 DX-LR03 LoRa 模块通信。

## 工程能力

- CAN1 电机闭环：向 `0x200` 发送电流命令，接收 `0x201`、`0x202` 电机反馈。
- 八路循迹：右侧 D01–D04 与左侧 D01–D04 形成线位掩码，经过多数滤波、误差跳变限制、直线补偿和丢线搜索后输出差速目标转速。
- 状态机：电机反馈确认 → LoRa READY → 启动延迟 → 等待有效黑线 → 循迹/丢线搜索 → A 点制动 → 完成或故障。
- LoRa 协议：使用 `AA 55` 帧头；支持 READY、START、START_ACK、STOPPED 和运行遥测帧。上电稳定 500 ms 后发送固定 READY 帧，发送完成后等待 7.5 s 才开始检测黑线。
- 诊断采集：提供 PowerShell 串口脚本，将 LoRa 遥测解析为 CSV。
- 辅助验证：包含独立的 TIM3 CH2 PWM 固定占空比测试工程。

## 目录说明

```text
.
├─ Sanmotai0917/
│  ├─ Core/                 CubeMX 生成的应用入口、外设初始化和中断
│  ├─ bsp/                  CAN 收发、M2006 反馈与 PID 实现
│  ├─ Drivers/              STM32F1 HAL 与 CMSIS 依赖
│  ├─ MDK-ARM/              主 Keil 工程：pid2308v1.uvprojx
│  ├─ PWM_Fixed_Test/       独立 PWM 固定占空比测试工程
│  ├─ Tools/                主机侧诊断脚本
│  └─ pid2308v1.ioc         CubeMX 配置
├─ *.pdf                    LoRa、M2006、C610 与需求参考资料
├─ .gitignore               Keil 构建输出及本地配置忽略规则
└─ README.md
```

`MDK-ARM/pid2308v1/`、`PWM_Fixed_Test/Objects/` 和 `Listings/` 是构建输出位置，不是应编辑的源目录。

## 硬件与接口

| 功能 | STM32 引脚 | 配置 |
| --- | --- | --- |
| CAN1 | PA11 RX / PA12 TX | 1 Mbit/s，CAN RX FIFO1 中断 |
| LoRa USART3 | PB10 TX / PB11 RX | 115200 bit/s，收发中断 |
| LoRa M1 | PA5 | GPIO 输出 |
| LoRa AUX | PA6 | GPIO 输入；当前固件按 `LORA_USE_AUX=0` 运行，未接 AUX 时下拉 |
| 循迹右侧 | PB6–PB9 | D01–D04，上拉输入 |
| 循迹左侧 | PB15–PB12 | D01–D04，上拉输入 |
| PWM 测试输出 | PA7 / TIM3_CH2 | 辅助 PWM 测试工程使用 |

LoRa、CAN 收发器和 MCU 必须共地。LoRa 供电与电平要求以根目录模块资料为准；不要将模块电源要求直接等同于 MCU 的 3.3 V 供电。

## 构建与使用

1. 使用 Keil MDK 打开 `Sanmotai0917\\MDK-ARM\\pid2308v1.uvprojx`，构建目标 `pid2308v1`。
2. 需要单独验证 PWM 时，打开 `Sanmotai0917\\PWM_Fixed_Test\\PWM_Fixed_Test.uvprojx`。
3. 需要记录循迹遥测时，在 Windows PowerShell 运行：

   ```powershell
   pwsh -File ".\\Sanmotai0917\\Tools\\capture_line_telemetry.ps1" -Port COMx
   ```

   可附加 `-DurationSeconds 60` 或 `-OutputPath <csv-path>`。

生成的固件通常位于 Keil 的目标输出目录。构建成功只证明源代码和工具链通过；下载、CAN 电机反馈、LoRa 收发和实际循迹仍需要在真实硬件上分别验证。

## 关键调参位置

主循迹与 LoRa 参数集中在 `Sanmotai0917/Core/Src/main.c` 的 `LINE_FOLLOW_*`、`LORA_*` 和 `MOTOR_SPEED_PID_*` 宏中：

- 速度与转向：`LINE_FOLLOW_BASE_OUTPUT_RPM`、`LINE_FOLLOW_KP_*`、`LINE_FOLLOW_KD_RPM_PER_ERROR_DELTA`。
- 起步与保护：`LINE_FOLLOW_START_DELAY_MS`、`LINE_FOLLOW_FEEDBACK_TIMEOUT_MS`、`LINE_FOLLOW_RUN_TIMEOUT_MS`。
- LoRa：`LORA_BOOT_STABILIZE_MS`、`LORA_USE_AUX`、`LORA_TELEMETRY_PERIOD_MS`。

在修改 CubeMX 配置后，应保留 `USER CODE` 区域，并确认 `HAL_Init()`、时钟配置和所有 `MX_*_Init()` 的初始化顺序未被破坏。

## 当前工程边界

- 主工程的 `.ioc` 标识 MCU 为 `STM32F103C8T6`，而 Keil 工程目标显示为 `STM32F103T6`；在重新生成 CubeMX 代码或下载前，应先按实际板卡核对该差异。
- 仓库当前没有项目级 `LICENSE` 文件；HAL/CMSIS 和 BSP 文件中的第三方版权声明仍适用。
- 本次仓库整理仅涉及名称与文档，不包含业务源代码、CubeMX 配置、Keil 工程配置或硬件行为的修改。
