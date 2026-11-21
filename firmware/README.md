# OpenOBW 固件

OpenOBW 使用 STM32F103C8T6 和 BMI088，通过 RS485 应答主机查询。工程从内部 Flash 起始地址 `0x08000000` 运行。

## 工程结构

| 文件 | 职责 |
|---|---|
| `Core/Src/main.c`、`stm32f1xx_hal_msp.c`、`stm32f1xx_it.c` | 外设、FreeRTOS 任务和中断入口 |
| `Core/Src/bmi088.c` | SPI 访问、BMI088 配置回读和状态检查 |
| `Core/Src/acquisition.c` | `defaultTask` 采集、更新最新快照，`eventTask` 处理告警 |
| `Core/Src/protocol.c` | USART3 接收中断、`commandTask` 解析、应答及事件发送 |
| `Core/Src/filters.c`、`params.c` | FIR、卡尔曼滤波及非易失参数 |
| `Drivers/`、`Middlewares/` | STM32 HAL、CMSIS 和 FreeRTOS |
| `OpenOBW.ioc`、`OpenOBW.pdf` | 外设配置及原理图 |

## 采集与指示灯

- LED1 上电点亮。BMI088 初始化校验失败或 ADC 初始化校准失败时，LED2 持续闪烁，采集和通信任务停止启动。
- SPI1 使用 Mode 3、4 MHz，PA3 和 PB12 分别为陀螺仪和加速度计片选。加速度计原生 50 Hz，陀螺仪原生 100 Hz。PB13 和 PB0 的数据就绪中断唤醒采集任务。陀螺仪每次中断都读出，每两笔保留一笔，得到 50 Hz 的平均输入。每 100 ms 对窗口内的样本求平均，缓冲区只保留最新快照。FIR 和卡尔曼模式对窗口平均值追加滤波。
- PA2 的热敏电阻使用 10 kΩ、B=3450，温度由分压比换算。PA1 的输入电压使用 33 kΩ / 3 kΩ 分压，按同一次 ADC 扫描中采集的内部参考换算，`VIN = 12 × 1.20 V × ADC_VIN / ADC_VREFINT`。这里测的是保护二极管后的 VPP，绝对精度受 VREFINT 的个体偏差和分压电阻误差影响。
- 运行期间，BMI088 通信或采集异常、模拟采集失败、低于 10 V、高于 36 V、低于 −20 °C 或高于 75 °C 时，LED1 闪烁。电压和温度的越限、恢复会分别发出事件。收到本机地址的有效请求时，LED2 在处理和回复期间点亮。

采样率及寄存器配置依据 [Bosch BMI088 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)。

## RS485 通信

USART3 为 **9600 bps、8N1**。按原理图，PB10 是 TX，PB11 是 RX。板载 MS2548 自动控制收发方向。

协议依据 BTA-6 的 `DESIGN.md` 和 `Core/Src/command.c`，因为任务清单中的 `protocol.txt` 路径下未找到该文件。多字节字段为大端序，以下帧均以十六进制表示。

```text
请求  AA ADDR CMD LEN [参数] XOR
应答  55 ADDR CMD LEN STATUS [数据] XOR
事件  55 ADDR 7F 03 EVENT VALUE_H VALUE_L XOR
```

- `LEN` 是载荷长度，应答包含一个状态字节。校验值为 `ADDR` 到载荷最后一字节的逐字节异或。
- `STATUS` 为 `00` 成功、`01` 未知命令、`02` 参数错误。OpenOBW 在最新数据无效或 Flash 保存失败时也返回 `02`。地址或校验不匹配时忽略该帧。
- 默认地址是 `01`。广播地址 `FF` 允许所有节点执行并用各自地址回复。广播查询及修改宜在单节点条件下使用，以避免回复冲突。
- 主机应连续发送一帧，字节间隔小于 20 ms，并等待回复后发送下一条命令。事件帧可在命令之间出现。

| CMD | 操作 | 参数 | 成功数据 |
|---|---|---|---|
| `01` | 读加速度 | 无 | X/Y/Z，3 × int16，0.061 mg/LSB |
| `02` | 读角速度 | 无 | X/Y/Z，3 × int16，8.75 mdps/LSB |
| `03` | 读温度和电压 | 无 | 温度 int16，0.01 °C；电压 uint16，0.01 V |
| `04` | 读地址 | 无 | 地址 1 字节 |
| `05` | 改地址并保存 | 新地址 1–254 | 无，以新地址回复 |
| `06` | 读滤波模式 | 无 | 模式 1 字节 |
| `07` | 改滤波模式并保存 | `00` 平均、`01` FIR、`02` 卡尔曼 | 无 |
| `08` | 保存参数 | 无 | 无 |
| `09` | 读固件版本 | 无 | 主、次、补丁各 1 字节，当前 `01 00 00` |
| `0A` | 读设备 ID | 无 | 96 位芯片 UID 的 FNV-1a64 哈希，8 字节 |

加速度原生量程为 ±3 g，编码沿用 BTA-6 的 int16 范围，超出约 ±2 g 的结果会饱和。事件 `01`、`02`、`11` 表示电压过高、过低和恢复，数值单位为 0.01 V。事件 `03`、`04`、`12` 表示温度过高、过低和恢复，数值单位为 0.01 °C。事件数值为 int16。

### 命令示例

| 用途 | 发送 | 应答示例 |
|---|---|---|
| 广播读地址 | `AA FF 04 00 FB` | `55 01 04 02 00 01 06` |
| 读加速度 | `AA 01 01 00 00` | `55 01 01 07 00 00 00 00 00 40 0A 4D` |
| 读角速度 | `AA 01 02 00 03` | `55 01 02 07 00 00 00 00 00 00 00 04` |
| 读温度和电压 | `AA 01 03 00 02` | `55 01 03 05 00 09 E8 04 B4 56`（25.36 °C、12.04 V） |
| 设卡尔曼滤波 | `AA 01 07 01 02 05` | `55 01 07 01 00 07` |
| 地址改为 7 | `AA 01 05 01 07 02` | `55 07 05 01 00 03` |

## 编译和烧录

1. 在 STM32CubeIDE 中选择 **File → Import → General → Existing Projects into Workspace**，导入包含 `.project` 的固件目录。
2. 直接编译现有工程。选择 **Project → Build Project**，产物为 `Debug/OpenOBW.elf` 或 `Release/OpenOBW.elf`。如果需要从 CubeMX 重新生成，请先提交或备份现有源码，生成后复核 SPI、ADC DMA、启动顺序及自定义中断。
3. 使用 ST-LINK 接 J1 的 SWDIO、SWCLK、GND 和目标板电压参考，确认目标板供电后，在 CubeIDE 中运行 **Debug As → STM32 C/C++ Application**，或用 STM32CubeProgrammer 通过 SWD 写入 ELF 并复位。若转为 BIN，加载地址为 `0x08000000`。
4. 最后一页 `0x0800FC00` 保留给地址和滤波参数。全片擦除会使配置恢复默认。
5. 连接 USB-RS485 转接器并使用 HEX 收发，先查地址，再读温度、电压和两组运动数据。首次运动应答需等待首个有效采集窗口。

已用 GNU Arm Embedded 工具链完成 Cortex-M3 编译和链接。传感器中断、ADC 精度及 RS485 收发仍需实板验证。
