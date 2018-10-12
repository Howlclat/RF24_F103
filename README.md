#  NRF24L01模块驱动库示例程序

## 硬件信息

MCU型号：`STM32F103ZET6`

开发板：野火ISO STM32开发板

时钟：72M

## 使用管脚

- SPI_CS -> G15
- SPI_SCK -> A5
- SPI_MISO -> A6
- SPI_MOSI -> A7
- NRF_CE -> G8
- NRF_IRQ -> C4
- USART_TX -> A9
- USART_RX -> A10
- KEY1 -> A0
- KEY2 -> C13
- LED1 -> F7
- LED2 -> F8
- LED3 -> B1

## RF配置

- 地址 3字节 

- 自动重发 ON，时间 500us
- 空中速率 250K
- 发射功率 0dBm
- CRC ON，一字节
- 通道 0x6E

## 主要功能

可作为发射端，与闪光灯接收器通信。

按下key1时，led1亮，设置发射地址与接收通道1地址为功率设置地址`0x25,0x08,0xA1` ，发送功率设置码 `0x76, 0xAA, 0xAA, 0xAA, 0x0A` ，串口返回发送状态。

按下key2时，led2亮，设置发射地址与接收通道1地址为定义模式地址`0x45,0x2A,0x6D` ，发送接收器地址定义码 `0xDF, 0x00, 0x25, 0x08, 0xA1`，串口返回发送状态。