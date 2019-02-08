/**
  ******************************************************************************
  * @file    main.c
  * @author  Clat
  * @version V1.0
  * @date    2018-10-01
  * @brief   NRF24L01模块驱动库示例程序
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * MCU型号：STM32F103ZET6
  *
  ******************************************************************************
  */ 
	
#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_clkconfig.h"
#include "bsp_mcooutput.h"
#include "bsp_exti.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "drv_nrf24l01.h"
#include <stdbool.h>

uint8_t KeyPressed = 0;
uint8_t txbuf[5]={0x76, 0xAA, 0xAA, 0xAA, 0x0A};
uint8_t txbuf1[5]={0xDF, 0x00, 0x25, 0x08, 0xA1};

#define DEF_ADDR 0x45,0x2A,0x6D
#define SET_ADDR 0x25,0x08,0xA1

uint8_t addr[3] = {SET_ADDR};
uint8_t addr1[3] = {DEF_ADDR};

bool serialIRQ = false;
uint32_t buff_length = 0;

uint8_t g_UsartRxBuffer[100] = {0};
/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{	
    uint8_t status = 0;
	uint8_t observe = 0;
	
	// 使用HSE时，SYSCLK = 8M * RCC_PLLMul_x, x:[2,3,...16],最高是128M
	HSE_SetSysClock(RCC_PLLMul_9);  
	
	/* NVIC分组选择 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	// LED 端口初始化
	LED_GPIO_Config();
    
    // 按键中断初始化
	EXTI_Key_Config();
    
	/* USART GPIO配置 */
	USARTx_GPIO_Config();
	
	/* 初始化USART 配置模式为 115200 8-N-1，中断接收 */
	USART_Config(115200);
		
	/* 配置使用DMA模式 */
	USARTx_DMA_Config();
	
	/* 发送一个字符串 */
    printf("NRF24L01 Test Program\r\n");
    
    // NRF引脚初始化
    NRF_Gpio_Init();
    
    // SPI初始化
    SPI_NRF_Init();
    
    status = NRF_check();
    
    /* 判断连接状态 */  
    if(status == SUCCESS)	   
        printf("NRF24L01 Connect Success!\r\n");  
    else	  
        printf("NRF24L01 Connect Fail!\r\n");
    
    NRF_Init();
    NRF_Set_Mode( MODE_TX );
	
	while (1)
	{
		if (serialIRQ)
		{
			serialIRQ = false;
			printf("serial IRQ! data length:%d\n", buff_length);
			DMA_Cmd(USART_RX_DMA_CHANNEL, ENABLE); 
		}
        if (KeyPressed == 1)
        {
			LED1_ON;
			NRF_Set_TxAddr( addr, 3 );
			NRF_Set_RxAddr( 0, addr, 3 );
            status = NRF_TxPacket(txbuf, 5);
            
            switch(status)
            {
                case TX_OK:
                    printf("TX OK!\r\n");
					observe = NRF_Read_Reg(OBSERVE_TX);
					printf("reTransmit Count: %X\r\n", (observe & 0x0F));
					printf("lost packets Count: %X\r\n", (observe & 0xF0 >> 4));
                    break;
                case MAX_TX:
                    printf("MAX_TX\r\n");
                    break;
                default:
					printf("Status:%X\r\n", status);
                    break;
            }
            KeyPressed = 0;
			LED1_OFF;
        }
        else if (KeyPressed == 2)
        {
            LED2_ON;
			NRF_Set_TxAddr( addr1, 3 );
			NRF_Set_RxAddr( 0, addr1, 3 );
            status = NRF_TxPacket(txbuf1, 5);
            
            switch(status)
            {
                case TX_OK:
                    printf("TX OK!\r\n");
					observe = NRF_Read_Reg(OBSERVE_TX);
					printf("reTransmit Count: %X\r\n", (observe & 0x0F));
					printf("lost packets Count: %X\r\n", (observe & 0xF0 >> 4));
                    break;
                case MAX_TX:
                    printf("MAX_TX\r\n");
                    break;
                case 0xFF:
                    printf("ERROR\r\n");
                    break;
                default:
					printf("Status:%X\r\n", status);
                    break;
            }
            KeyPressed = 0;
			LED2_OFF;
        }

        
	}
}

/*********************************************END OF FILE**********************/
