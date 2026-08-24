# 2021 年全国大学生电子设计竞赛 A 题——THD 测量模块

本仓库保存 2021 年全国大学生电子设计竞赛 A 题相关的嵌入式测量固件。工程以 **STM32H750VBT6** 为核心，针对输入信号完成高速采样、频率与幅值估计、谐波分析和总谐波失真（THD）计算，并将结果通过串口发送给上位机或显示端。

> 当前目录对应 `THDMeasurementB` 工程，是整机中用于信号采集与 THD 分析的模块。

## 功能概览

- TIM2 触发 ADC1 采样，DMA 连续采集 4096 点 14 位数据。
- 使用 CMSIS-DSP 的 4096 点复数 FFT 提取基波。
- 采用加窗、频谱峰值搜索和相邻频点插值，估计基波频率与幅值。
- 计算第 2 至第 5 次谐波相对基波的归一化幅值及 THD。
- 根据估计频率动态调整采样率，适配不同频段的输入信号。
- 通过 USART3 以 ValuePack 二进制帧上报采样点和分析结果。

## 硬件与外设配置

| 项目 | 配置 |
| --- | --- |
| 主控 | STM32H750VBT6（Cortex-M7，LQFP100） |
| ADC 输入 | ADC1 / INP3，PA6 |
| ADC 触发 | TIM2 更新事件（TRGO） |
| 采样缓冲区 | 4096 点，`uint16_t` |
| 调试串口 | USART1，115200 bps，PB14（TX）/ PB15（RX） |
| 数据上报串口 | USART3，9600 bps，PD8（TX）/ PB11（RX） |
| I²C | I2C1，PB6（SCL）/ PB7（SDA），可连接 SSD1306 OLED |
| 外部触发 | PB8，下降沿中断 |
| 调试接口 | SWD，PA13（SWDIO）/ PA14（SWCLK） |

## 信号处理流程

```text
模拟输入
   │
   ▼
ADC1 ← TIM2 定时触发 ← 动态调整采样率
   │
   ▼
DMA 采集 4096 点
   │
   ▼
加窗与 4096 点 FFT
   │
   ├── 基波频率 / 幅值估计
   ├── 2～5 次谐波幅值计算
   └── THD 计算
   │
   ▼
USART3 ValuePack 数据帧 → 上位机 / 显示端
```

核心算法位于 [`user/dsp_algo/dsp_algo.c`](user/dsp_algo/dsp_algo.c)：

1. 对采样数据加窗后进行 FFT，搜索频谱中的最大基波峰值；
2. 利用基波附近频点进行插值，得到更精确的频率估计；
3. 在第 2～5 次谐波附近搜索局部峰值，并计算各次谐波相对基波的幅值；
4. 按下式计算总谐波失真：

```math
\mathrm{THD}=\frac{\sqrt{A_2^2+A_3^2+A_4^2+A_5^2}}{A_1}\times100\%
```

## 目录说明

```text
.
├── Core/                         # STM32CubeMX 生成的启动、HAL 与外设初始化代码
│   ├── Inc/
│   └── Src/
├── Drivers/                      # STM32 HAL、CMSIS 与 DSP 库
├── Middlewares/                  # X-CUBE-ALGOBUILD / CMSIS-DSP 相关中间件
├── user/
│   ├── dsp_algo/                 # FFT、频率、谐波与 THD 算法
│   ├── bsp_comm/                 # 串口数据封装与发送接口
│   ├── USART+Interruption/       # ValuePack 二进制打包实现
│   └── oledproject_ssd1306/      # SSD1306 OLED 驱动
├── MDK-ARM/                      # Keil MDK 工程、启动文件与构建产物
│   └── THDMeasurementB.uvprojx   # Keil 工程文件
├── THDMeasurementB.ioc           # STM32CubeMX 配置文件
└── README.md
```

## 构建与烧录

### 开发环境

- STM32CubeMX 6.15.0（如需修改引脚或时钟配置）
- STM32Cube Firmware Package H7 v1.12.1
- Keil MDK-ARM（打开并编译工程）
- ARM 编译器及 CMSIS-DSP 库
- ST-Link 调试器

### 操作步骤

1. 使用 Keil MDK 打开 [`MDK-ARM/THDMeasurementB.uvprojx`](MDK-ARM/THDMeasurementB.uvprojx)。
2. 确认已安装 STM32H7 的 Device Family Pack 与工程所需的 ARM/CMSIS 组件。
3. 编译工程；成功后可在 `MDK-ARM/THDMeasurementB/` 找到 `THDMeasurementB.hex`。
4. 通过 ST-Link 连接 SWD 接口，下载并运行固件。

如需重新生成外设初始化代码，请先打开 `THDMeasurementB.ioc`。重新生成前请保留 CubeMX 标记的 `/* USER CODE BEGIN */` 与 `/* USER CODE END */` 区域中的业务代码。

## 串口数据协议

USART3 发送 ValuePack 帧，采用小端字节序：

| 字段 | 长度 | 说明 |
| --- | ---: | --- |
| 帧头 | 1 字节 | 固定为 `0xA5` |
| 原始采样值 | 2 字节 | `uint16_t` |
| 二次谐波比 | 4 字节 | `float` |
| 三次谐波比 | 4 字节 | `float` |
| 四次谐波比 | 4 字节 | `float` |
| 五次谐波比 | 4 字节 | `float` |
| THD | 4 字节 | `float`，百分比 |
| 频率 | 4 字节 | `float`，Hz |
| 校验和 | 1 字节 | 从原始采样值至频率字段的字节累加和 |
| 帧尾 | 1 字节 | 固定为 `0x5A` |

单帧总长度为 29 字节。字段打包实现见 [`user/USART+Interruption/valuepack.c`](user/USART+Interruption/valuepack.c)，上报顺序见 [`user/bsp_comm/bsp_comm.c`](user/bsp_comm/bsp_comm.c)。

## 关键参数

| 参数 | 当前值 | 位置 |
| --- | ---: | --- |
| FFT 点数 | 4096 | `user/dsp_algo/dsp_algo.h` 中的 `LEN` |
| 初始采样率 | 2 MHz | `user/dsp_algo/dsp_algo.c` 中的 `SAMPLE_RATE` |
| 默认上报采样点数 | 25 | `Core/Src/main.c` 中的 `period` |
| 谐波分析范围 | 2～5 次 | `DSP_ProcessSignal()` |

## 注意事项

- ADC 输入必须满足 STM32H750 的电压范围；接入交流或高压信号前，需要使用适当的隔离、衰减和偏置电路。
- 工程的频率、幅值及 THD 结果依赖于前端模拟链路、采样时钟和窗口参数，投入实际测量前应使用标准信号源校准。
- `MDK-ARM/THDMeasurementB/` 内含历史构建产物，重新编译会更新该目录中的文件。
- 仓库暂未声明开源许可证；在复用或分发前，请先取得作者授权。

## 参考

- [仓库主页](https://github.com/Junhousheng-Serena/Electronic-Design-Competition-21-A)
- [STM32CubeMX 工程配置](THDMeasurementB.ioc)
