#ifndef __DRV_RF24L01_H__
#define __DRV_RF24L01_H__

#include "stm32f10x.h"
#include <stdio.h>

/** 配置和选项定义 */
#define DYNAMIC_PACKET      0				//1:动态数据包, 0:固定
#define FIXED_PACKET_LEN    16				//包长度
#define REPEAT_CNT          8				//自动重发次数

/** RF24L01硬件IO定义 */
#define NRF_CE_CLK              RCC_APB2Periph_GPIOG
#define NRF_CE_PORT				GPIOG
#define NRF_CE_PIN				GPIO_Pin_8

#define NRF_IRQ_CLK             RCC_APB2Periph_GPIOC
#define NRF_IRQ_PORT			GPIOC
#define NRF_IRQ_PIN				GPIO_Pin_4

/** 口线操作函数定义 */
#define NRF_SET_CE_HIGH( )			GPIO_SetBits( NRF_CE_PORT, NRF_CE_PIN )
#define NRF_SET_CE_LOW( )			GPIO_ResetBits( NRF_CE_PORT, NRF_CE_PIN )

#define NRF_GET_IRQ_STATUS( )		GPIO_ReadInputDataBit(NRF_IRQ_PORT, NRF_IRQ_PIN)	//IRQ状态


/*SPI接口定义-开头***************************************************************************/
#define     SPIx_NRF                         SPI1
#define     SPI_NRF_APBxClock_FUN            RCC_APB2PeriphClockCmd
#define     SPI_NRF_CLK                      RCC_APB2Periph_SPI1

//CS(NSS)引脚 片选选普通GPIO即可
#define     SPI_NRF_CS_APBxClock_FUN         RCC_APB2PeriphClockCmd
#define     SPI_NRF_CS_CLK                   RCC_APB2Periph_GPIOG
#define     SPI_NRF_CS_PORT                  GPIOG
#define     SPI_NRF_CS_PIN                   GPIO_Pin_15

//SCK引脚
#define     SPI_NRF_SCK_APBxClock_FUN        RCC_APB2PeriphClockCmd
#define     SPI_NRF_SCK_CLK                  RCC_APB2Periph_GPIOA   
#define     SPI_NRF_SCK_PORT                 GPIOA   
#define     SPI_NRF_SCK_PIN                  GPIO_Pin_5
//MISO引脚
#define     SPI_NRF_MISO_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define     SPI_NRF_MISO_CLK                 RCC_APB2Periph_GPIOA    
#define     SPI_NRF_MISO_PORT                GPIOA 
#define     SPI_NRF_MISO_PIN                 GPIO_Pin_6
//MOSI引脚
#define     SPI_NRF_MOSI_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define     SPI_NRF_MOSI_CLK                 RCC_APB2Periph_GPIOA    
#define     SPI_NRF_MOSI_PORT                GPIOA 
#define     SPI_NRF_MOSI_PIN                 GPIO_Pin_7

#define  	SPI_NRF_CS_LOW()				GPIO_ResetBits( SPI_NRF_CS_PORT, SPI_NRF_CS_PIN )
#define  	SPI_NRF_CS_HIGH()				GPIO_SetBits( SPI_NRF_CS_PORT, SPI_NRF_CS_PIN )

/*SPI接口定义-结尾****************************************************************************/

typedef enum ModeType
{
	MODE_TX = 0,
	MODE_RX
}NRF_ModeType;

typedef enum SpeedType
{
	SPEED_250K = 0,
	SPEED_1M,
	SPEED_2M
}NRF_SpeedType;

typedef enum PowerType
{
	POWER_F18DBM = 0,
	POWER_F12DBM,
	POWER_F6DBM,
	POWER_0DBM
}NRF_PowerType;


/** NRF24L01定义 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//寄存器操作命令
#define NRF_READ_REG    0x00	//读配置寄存器，低5位为寄存器地址
#define NRF_WRITE_REG   0x20	//写配置寄存器，低5位为寄存器地址
#define RD_RX_PLOAD     0x61	//读RX有效数据，1~32字节
#define WR_TX_PLOAD     0xA0	//写TX有效数据，1~32字节
#define FLUSH_TX        0xE1	//清除TX FIFO寄存器，发射模式下使用
#define FLUSH_RX        0xE2	//清除RX FIFO寄存器，接收模式下使用
#define REUSE_TX_PL     0xE3	//重新使用上一包数据，CE为高，数据包被不断发送
#define R_RX_PL_WID     0x60
#define NOP             0xFF	//空操作，可以用来读状态寄存器
#define W_ACK_PLOAD		0xA8
#define WR_TX_PLOAD_NACK 0xB0
//SPI(NNRF)寄存器地址
#define CONFIG          0x00	//配置寄存器地址，bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
							    //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能	
#define EN_AA           0x01	//使能自动应答功能 bit0~5 对应通道0~5
#define EN_RXADDR       0x02	//接收地址允许 bit0~5 对应通道0~5
#define SETUP_AW        0x03	//设置地址宽度(所有数据通道) bit0~1: 00,3字节 01,4字节, 02,5字节
#define SETUP_RETR      0x04	//建立自动重发;bit0~3:自动重发计数器;bit4~7:自动重发延时 250*x+86us
#define RF_CH           0x05	//RF通道,bit0~6工作通道频率
#define RF_SETUP        0x06	//RF寄存器，bit3:传输速率( 0:1M 1:2M);bit1~2:发射功率;bit0:噪声放大器增益
#define STATUS          0x07	//状态寄存器;bit0:TX FIFO满标志;bit1~3:接收数据通道号(最大:6);bit4:达到最多次重发次数
								//bit5:数据发送完成中断;bit6:接收数据中断
#define MAX_TX  		0x10	//达到最大发送次数中断
#define TX_OK   		0x20	//TX发送完成中断
#define RX_OK   		0x40	//接收到数据中断

#define OBSERVE_TX      0x08	//发送检测寄存器,bit7~4:数据包丢失计数器;bit3~0:重发计数器
#define CD              0x09	//载波检测寄存器,bit0:载波检测
#define RX_ADDR_P0      0x0A	//数据通道0接收地址，最大长度5个字节，低字节在前
#define RX_ADDR_P1      0x0B	//数据通道1接收地址，最大长度5个字节，低字节在前
#define RX_ADDR_P2      0x0C	//数据通道2接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P3      0x0D	//数据通道3接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P4      0x0E	//数据通道4接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P5      0x0F	//数据通道5接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define TX_ADDR         0x10	//发送地址(低字节在前),ShockBurstTM模式下，RX_ADDR_P0与地址相等
#define RX_PW_P0        0x11	//接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12	//接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13	//接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14	//接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15	//接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16	//接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17	//FIFO状态寄存器;bit0:RX FIFO寄存器空标志;bit1:RX FIFO满标志;bit2~3保留
								//bit4:TX FIFO 空标志;bit5:TX FIFO满标志;bit6:1,循环发送上一数据包.0,不循环								
#define DYNPD			0x1C
#define FEATRUE			0x1D
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/* NRF24L01 寄存器 位定义 */

/******************  Bit definition for CONFIG register  *****************/
#define MASK_RX_DR   	6 		/*!< 接收中断屏蔽位 */
#define MASK_TX_DS   	5 		/*!< 发送中断屏蔽位 */
#define MASK_MAX_RT  	4 		/*!< 最大重发次数中断屏蔽位 */
#define EN_CRC       	3 		/*!< CRC使能 */
#define CRCO         	2 		/*!< CRC位数, 0:1字节, 1:2字节 */
#define PWR_UP       	1 		/*!< 1:Power Up, 2:Power Down*/
#define PRIM_RX      	0 		/*!< 1:RX, 0:TX */
/******************  Bit definition for EN_AA register  *****************/
#define ENAA_P5      	5 		/*!< 数据管道5自动确认使能 */
#define ENAA_P4      	4 		/*!< 数据管道4自动确认使能 */
#define ENAA_P3      	3 		/*!< 数据管道3自动确认使能 */
#define ENAA_P2      	2 		/*!< 数据管道2自动确认使能 */
#define ENAA_P1      	1 		/*!< 数据管道1自动确认使能 */
#define ENAA_P0      	0 		/*!< 数据管道0自动确认使能 */
/******************  Bit definition for EN_RXADDR register  *****************/
#define ERX_P5       	5 		/*!< 数据管道5使能 */
#define ERX_P4       	4 		/*!< 数据管道4使能 */
#define ERX_P3       	3 		/*!< 数据管道3使能 */
#define ERX_P2      	2 		/*!< 数据管道2使能 */
#define ERX_P1       	1 		/*!< 数据管道1使能 */
#define ERX_P0       	0 		/*!< 数据管道0使能 */
/******************  Bit definition for SETUP_AW register  *****************/
#define AW_RERSERVED 	0x0 
#define AW_3BYTES    	0x1 		/*!< 地址宽度配置 */
#define AW_4BYTES    	0x2
#define AW_5BYTES    	0x3
/******************  Bit definition for SETUP_RETR register  *****************/
#define ARD_250US    	(0x00<<4)	/*!< 自动重发间隔 */
#define ARD_500US    	(0x01<<4)
#define ARD_750US    	(0x02<<4)
#define ARD_1000US   	(0x03<<4)
#define ARD_2000US   	(0x07<<4)
#define ARD_4000US   	(0x0F<<4)
#define ARC_DISABLE   	0x00
#define ARC_15        	0x0F
/******************  Bit definition for RF_SETUP register  *****************/
#define CONT_WAVE     	7 
#define RF_DR_LOW     	5 
#define PLL_LOCK      	4 
#define RF_DR_HIGH    	3 
//bit2-bit1:
#define PWR_18DB  		(0x00<<1)
#define PWR_12DB  		(0x01<<1)
#define PWR_6DB   		(0x02<<1)
#define PWR_0DB   		(0x03<<1)
/******************  Bit definition for STATUS register  *****************/
#define RX_DR         	6 
#define TX_DS         	5 
#define MAX_RT        	4 
//for bit3-bit1, 
#define TX_FULL_0     	0 
/******************  Bit definition for RPD register  *****************/
#define RPD           	0 
/******************  Bit definition for FIFO_STATUS register  *****************/
#define TX_REUSE      	6 
#define TX_FULL_1     	5 
#define TX_EMPTY      	4 
//bit3-bit2, reserved, only '00'
#define RX_FULL       	1 
#define RX_EMPTY      	0 
/******************  Bit definition for DYNPD register  *****************/
#define DPL_P5        	5 
#define DPL_P4        	4 
#define DPL_P3        	3 
#define DPL_P2        	2 
#define DPL_P1        	1 
#define DPL_P0        	0 
/******************  Bit definition for FEATURE register  *****************/
#define EN_DPL        	2 
#define EN_ACK_PAY    	1 
#define EN_DYN_ACK    	0 

#define IRQ_ALL  ( (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT) )

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void NRF_Gpio_Init( void );
void SPI_NRF_Init( void );
uint8_t SPI_ReadWriteByte(uint8_t byte);
uint8_t NRF_Write_Reg( uint8_t RegAddr, uint8_t Value );
uint8_t NRF_Read_Reg( uint8_t RegAddr );
void NRF_Read_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len );
void NRF_Write_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len );
void NRF_Flush_Tx_FIFO( void );
void NRF_Flush_Rx_FIFO( void );
void NRF_Reuse_Tx_Payload( void );
void NRF_Nop( void );
uint8_t NRF_Read_Status_Register( void );
uint8_t NRF_Clear_IRQ_Flag( uint8_t IRQ_Source );
uint8_t NRF_Read_IRQ_Status( void );
uint8_t NRF_Read_Top_Fifo_Width( void );
uint8_t NRF_Read_Rx_Payload( uint8_t *pRxBuf );
void NRF_Write_Tx_Payload_Ack( uint8_t *pTxBuf, uint8_t len );
void NRF_Write_Tx_Payload_NoAck( uint8_t *pTxBuf, uint8_t len );
void NRF_Write_Tx_Payload_InAck( uint8_t *pData, uint8_t len );
void NRF_Set_TxAddr( uint8_t *pAddr, uint8_t len );
void NRF_Set_RxAddr( uint8_t PipeNum, uint8_t *pAddr, uint8_t Len );
void NRF_Set_Speed( NRF_SpeedType Speed );
void NRF_Set_Power( NRF_PowerType Power );
void NRF_Set_Freq( uint8_t FreqPoint );
void NRF_Set_Mode( NRF_ModeType Mode );
uint8_t NRF_check( void );
uint8_t NRF_TxPacket( uint8_t *txbuf, uint8_t Length );
uint8_t NRF_RxPacket( uint8_t *rxbuf );
void NRF_Init( void );

#endif

