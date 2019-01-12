/**
  ******************************************************************************
  * @file    drv_nrf24l01.c
  * @author  Clat
  * @version V1.0
  * @date    2018-10-01
  * @brief   NRF24L01驱动库
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "drv_nrf24l01.h"
#include "bsp_SysTick.h"

 /**
  * @brief :NRF24L01引脚初始化
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Gpio_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能SPI引脚相关的时钟 */
    SPI_NRF_CS_APBxClock_FUN ( SPI_NRF_CS_CLK
                                |SPI_NRF_SCK_CLK
                                |SPI_NRF_MISO_CLK
                                |SPI_NRF_MOSI_CLK
                                |NRF_IRQ_CLK
                                |NRF_CE_CLK, ENABLE );

    /* 配置SPI的 CS引脚，普通IO即可 */
    GPIO_InitStructure.GPIO_Pin = SPI_NRF_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_NRF_CS_PORT, &GPIO_InitStructure);

    /* 配置SPI的 SCK引脚*/
    GPIO_InitStructure.GPIO_Pin = SPI_NRF_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI_NRF_SCK_PORT, &GPIO_InitStructure);

    /* 配置SPI的 MISO引脚*/
    GPIO_InitStructure.GPIO_Pin = SPI_NRF_MISO_PIN;
    GPIO_Init(SPI_NRF_MISO_PORT, &GPIO_InitStructure);

    /* 配置SPI的 MOSI引脚*/
    GPIO_InitStructure.GPIO_Pin = SPI_NRF_MOSI_PIN;
    GPIO_Init(SPI_NRF_MOSI_PORT, &GPIO_InitStructure);
    
    /* 配置NRF24L01的 IRQ引脚 */
    GPIO_InitStructure.GPIO_Pin = NRF_IRQ_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //上拉输入
    GPIO_Init(NRF_IRQ_PORT, &GPIO_InitStructure); 
    
    /* 配置NRF24L01的 CE引脚 */
    GPIO_InitStructure.GPIO_Pin = NRF_CE_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
    GPIO_Init(NRF_CE_PORT, &GPIO_InitStructure); 
	
	NRF_SET_CE_LOW( );		//使能设备
	SPI_NRF_CS_HIGH( );		//取消SPI片选
}

/**
  * @brief :SPI初始化（硬件）
  * @param :无
  * @note  :无
  * @retval:无
  */
void SPI_NRF_Init( void )
{
    SPI_InitTypeDef  SPI_InitStructure;

    /* 使能SPI时钟 */
    SPI_NRF_APBxClock_FUN ( SPI_NRF_CLK, ENABLE );

    /* SPI 模式配置 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx_NRF , &SPI_InitStructure);

    /* 使能 SPI  */
    SPI_Cmd(SPIx_NRF , ENABLE);
}

/**
  * @brief :SPI收发一个字节
  * @param :
  *	    @Byte: 发送的数据字节
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:接收到的字节
  */
uint8_t SPI_ReadWriteByte( uint8_t TxByte )
{
    /* 等待发送缓冲区为空，TXE事件 */
    while (SPI_I2S_GetFlagStatus(SPIx_NRF , SPI_I2S_FLAG_TXE) == RESET);

    /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
    SPI_I2S_SendData(SPIx_NRF , TxByte);

    /* 等待接收缓冲区非空，RXNE事件 */
    while (SPI_I2S_GetFlagStatus(SPIx_NRF , SPI_I2S_FLAG_RXNE) == RESET);

    /* 读取数据寄存器，获取接收缓冲区数据 */
    return SPI_I2S_ReceiveData(SPIx_NRF);
}

/**
  * @brief :NRF写寄存器
  * @param :
  *			@reg:寄存器地址
  *			@value:数据
  * @note  :地址在设备中有效
  * @retval:读写状态
  */
uint8_t NRF_Write_Reg( uint8_t RegAddr, uint8_t Value )
{
    uint8_t status;
    
    NRF_SET_CE_LOW( );      //CE置低
    
    SPI_NRF_CS_LOW( );		//片选
	
    status = SPI_ReadWriteByte( NRF_WRITE_REG | RegAddr );	//写命令 地址
    SPI_ReadWriteByte( Value );			//写数据
	
    SPI_NRF_CS_HIGH( );		//取消片选
    
    return status;
}

/**
  * @brief :NRF读寄存器
  * @param :
           @Addr:寄存器地址
  * @note  :地址在设备中有效
  * @retval:读取的数据
  */
uint8_t NRF_Read_Reg( uint8_t RegAddr )
{
    uint8_t btmp;
    
    NRF_SET_CE_LOW( );      //CE置低
    
    SPI_NRF_CS_LOW( );			//片选

    SPI_ReadWriteByte( NRF_READ_REG | RegAddr );	//读命令 地址
    btmp = SPI_ReadWriteByte( 0xFF );				//读数据

    SPI_NRF_CS_HIGH( );			//取消片选

    return btmp;
}

/**
  * @brief :NRF读指定长度的数据
  * @param :
  *			@reg:地址
  *			@pBuf:数据存放地址
  *			@len:数据长度
  * @note  :数据长度不超过255，地址在设备中有效
  * @retval:读取状态
  */
void NRF_Read_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len )
{
    uint8_t i;
    
    NRF_SET_CE_LOW( );      //CE置低
	
    SPI_NRF_CS_LOW( );	    //片选
	
    SPI_ReadWriteByte( NRF_READ_REG | RegAddr );	//读命令 地址
    for( i = 0; i < len; i ++ )
    {
        *( pBuf + i ) = SPI_ReadWriteByte( 0xFF );	//读数据
    }
    
    SPI_NRF_CS_HIGH( );		//取消片选
}

/**
  * @brief :NRF写指定长度的数据
  * @param :
  *			@reg:地址
  *			@pBuf:写入的数据地址
  *			@len:数据长度
  * @note  :数据长度不超过255，地址在设备中有效
  * @retval:写状态
  */
void NRF_Write_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len )
{
    uint8_t i;
	    
    NRF_SET_CE_LOW( );      //CE置低
    
    SPI_NRF_CS_LOW( );		//片选
	
    SPI_ReadWriteByte( NRF_WRITE_REG | RegAddr );	//写命令 地址
    for( i = 0; i < len; i ++ )
    {
        SPI_ReadWriteByte( *( pBuf + i ) );		//写数据
    }
	
    SPI_NRF_CS_HIGH( );		//取消片选
}

/**
  * @brief :清空TX缓冲区
  * @param :无
  * @note  :used in TX mode
  * @retval:无
  */
void NRF_Flush_Tx_FIFO ( void )
{
    SPI_NRF_CS_LOW( );		//片选
	
    SPI_ReadWriteByte( FLUSH_TX );	//清TX FIFO命令
	
    SPI_NRF_CS_HIGH( );		//取消片选
}

/**
  * @brief :清空RX缓冲区
  * @param :无
  * @note  :used in RX mode
  * @retval:无
  */
void NRF_Flush_Rx_FIFO( void )
{
    SPI_NRF_CS_LOW( );		//片选
	
    SPI_ReadWriteByte( FLUSH_RX );	//清RX FIFO命令
	
    SPI_NRF_CS_HIGH( );		//取消片选
}

/**
  * @brief :重新使用上一包数据
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Reuse_Tx_Payload( void )
{
    SPI_NRF_CS_LOW( );		//片选
	
    SPI_ReadWriteByte( REUSE_TX_PL );		//重新使用上一包命令
	
    SPI_NRF_CS_HIGH( );		//取消片选
}

/**
  * @brief :NRF空操作
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Nop( void )
{
    SPI_NRF_CS_LOW( );		//片选
	
    SPI_ReadWriteByte( NOP );		//空操作命令
	
    SPI_NRF_CS_HIGH( );		//取消片选
}

/**
  * @brief :NRF读状态寄存器
  * @param :无
  * @note  :无
  * @retval:RF24L01状态
  */
uint8_t NRF_Read_Status_Register( void )
{
    uint8_t Status;
	
    SPI_NRF_CS_LOW( );		//片选
	
    Status = SPI_ReadWriteByte( NRF_READ_REG | STATUS );	//读状态寄存器
	
    SPI_NRF_CS_HIGH( );		//取消片选
	
    return Status;
}

/**
  * @brief :NRF清中断
  * @param :
           @IRQ_Source:中断源
  * @note  :无
  * @retval:清除后状态寄存器的值
  */
uint8_t NRF_Clear_IRQ_Flag( uint8_t IRQ_Source )
{
    uint8_t btmp = 0;

    IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//中断标志处理
    btmp = NRF_Read_Status_Register( );			//读状态寄存器
			
    SPI_NRF_CS_LOW( );			//片选
    SPI_ReadWriteByte( NRF_WRITE_REG | STATUS );	//写状态寄存器命令
    SPI_ReadWriteByte( IRQ_Source | btmp );		//清相应中断标志
    SPI_NRF_CS_HIGH( );			//取消片选
	
    return ( NRF_Read_Status_Register( ));			//返回状态寄存器状态
}

/**
  * @brief :读RF24L01中断状态
  * @param :无
  * @note  :无
  * @retval:中断状态
  */
uint8_t NRF_Read_IRQ_Status( void )
{
    return ( NRF_Read_Status_Register( ) & (( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT )));	//返回中断状态
}
 
 /**
  * @brief :读FIFO中数据宽度
  * @param :无
  * @note  :无
  * @retval:数据宽度
  */
uint8_t NRF_Read_Top_Fifo_Width( void )
{
    uint8_t btmp;
	
    SPI_NRF_CS_LOW( );		//片选
	
    SPI_ReadWriteByte( R_RX_PL_WID );	//读FIFO中数据宽度命令
    btmp = SPI_ReadWriteByte( 0xFF );	//读数据
	
    SPI_NRF_CS_HIGH( );		//取消片选
	
    return btmp;
}

 /**
  * @brief :读接收到的数据
  * @param :
  *     @pRxBuf:数据存放地址首地址
  * @note  :无
  * @retval:数据宽度 
  */
uint8_t NRF_Read_Rx_Payload( uint8_t *pRxBuf )
{
    uint8_t Width, PipeNum;
	
    PipeNum = ( NRF_Read_Reg( STATUS ) >> 1 ) & 0x07;	//读接收状态
    Width = NRF_Read_Top_Fifo_Width( );		//读接收数据个数

    SPI_NRF_CS_LOW( );		//片选
    SPI_ReadWriteByte( RD_RX_PLOAD );			//读有效数据命令
	
    for( PipeNum = 0; PipeNum < Width; PipeNum ++ )
    {
        *( pRxBuf + PipeNum ) = SPI_ReadWriteByte( 0xFF );		//读数据
    }
    SPI_NRF_CS_HIGH( );		//取消片选
    NRF_Flush_Rx_FIFO( );	//清空RX FIFO
	
    return Width;
}

 /**
  * @brief :发送数据（带应答）
  * @param :
  *			@pTxBuf:发送数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF_Write_Tx_Payload_Ack( uint8_t *pTxBuf, uint8_t len )
{
	uint8_t l_Status = 0;
    uint8_t length = ( len > 32 ) ? 32 : len;		//数据长达大约32 则只发送32个

    NRF_Flush_Tx_FIFO( );		//清TX FIFO
	
    SPI_NRF_CS_LOW( );			//片选
    SPI_ReadWriteByte( WR_TX_PLOAD );	//发送命令
	while( length-- )
    {
        SPI_ReadWriteByte( *pTxBuf++ );			//发送数据
    }
    SPI_NRF_CS_HIGH( );			//取消片选
	
	SysTick_Delay_Us(160);
	l_Status = NRF_Read_Reg(STATUS);				//读状态寄存器
	NRF_Write_Reg( STATUS, l_Status );				//清除TX_DS或MAX_RT中断标志
	NRF_Write_Reg( FLUSH_TX, 0xff );	            //清除TX FIFO寄存器
}

 /**
  * @brief :发送数据（不带应答）
  * @param :
  *			@pTxBuf:发送数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF_Write_Tx_Payload_NoAck( uint8_t *pTxBuf, uint8_t len )
{
	uint8_t l_Status = 0;
	
    uint8_t length = ( len > 32 ) ? 32 : len;		//数据长达大约32 则只发送32个
	
    SPI_NRF_CS_LOW( );	//片选
    SPI_ReadWriteByte( WR_TX_PLOAD_NACK );	    //发送命令
    while( length-- )
    {
        SPI_ReadWriteByte( *pTxBuf++ );			//发送数据
    }
    SPI_NRF_CS_HIGH( );		//取消片选
	
	SysTick_Delay_Us(160);
	l_Status = NRF_Read_Reg(STATUS);				//读状态寄存器
	NRF_Write_Reg( STATUS, l_Status );				//清除TX_DS或MAX_RT中断标志
	NRF_Write_Reg( FLUSH_TX, 0xff );	            //清除TX FIFO寄存器
}

 /**
  * @brief :在接收模式下向TX FIFO写数据(带ACK)
  * @param :
  *			@pData:数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF_Write_Tx_Payload_InAck( uint8_t *pData, uint8_t len )
{
    uint8_t btmp;
	
	len = ( len > 32 ) ? 32 : len;		//数据长度大于32个则只写32个字节

    SPI_NRF_CS_LOW( );			//片选
    SPI_ReadWriteByte( W_ACK_PLOAD );		//命令
    for( btmp = 0; btmp < len; btmp ++ )
    {
        SPI_ReadWriteByte( *( pData + btmp ) );	//写数据
    }
    SPI_NRF_CS_HIGH( );			//取消片选
}

 /**
  * @brief :设置发送地址
  * @param :
  *			@pAddr:地址存放地址
  *			@len:长度
  * @note  :无
  * @retval:无
  */
void NRF_Set_TxAddr( uint8_t *pAddr, uint8_t len )
{
	len = ( len > 5 ) ? 5 : len;			//地址不能大于5个字节
    NRF_Write_Buf( TX_ADDR, pAddr, len );	//写地址
}

 /**
  * @brief :设置接收通道地址
  * @param :
  *			@PipeNum:通道
  *			@pAddr:地址存肥着地址
  *			@Len:长度
  * @note  :通道不大于5 地址长度不大于5个字节
  * @retval:无
  */
void NRF_Set_RxAddr( uint8_t PipeNum, uint8_t *pAddr, uint8_t Len )
{
    Len = ( Len > 5 ) ? 5 : Len;
    PipeNum = ( PipeNum > 5 ) ? 5 : PipeNum;		    //通道不大于5 地址长度不大于5个字节

    NRF_Write_Buf( RX_ADDR_P0 + PipeNum, pAddr, Len );	//写入地址
}

 /**
  * @brief :设置通信速度
  * @param :
  *			@Speed:速度
  * @note  :无
  * @retval:无
  */
void NRF_Set_Speed( NRF_SpeedType Speed )
{
	uint8_t btmp = 0;
	
	btmp = NRF_Read_Reg( RF_SETUP );
	btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	
	if( Speed == SPEED_250K )		//250K
	{
		btmp |= ( 1<<5 );
	}
	else if( Speed == SPEED_1M )   //1M
	{
   		btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	}
	else if( Speed == SPEED_2M )   //2M
	{
		btmp |= ( 1<<3 );
	}

	NRF_Write_Reg( RF_SETUP, btmp );
}

 /**
  * @brief :设置功率
  * @param :
  *			@Speed:速度
  * @note  :无
  * @retval:无
  */
void NRF_Set_Power( NRF_PowerType Power )
{
    uint8_t btmp;
	
	btmp = NRF_Read_Reg( RF_SETUP ) & ~0x07;
    switch( Power )
    {
        case POWER_F18DBM:
            btmp |= PWR_18DB;
            break;
        case POWER_F12DBM:
            btmp |= PWR_12DB;
            break;
        case POWER_F6DBM:
            btmp |= PWR_6DB;
            break;
        case POWER_0DBM:
            btmp |= PWR_0DB;
            break;
        default:
            break;
    }
    NRF_Write_Reg( RF_SETUP, btmp );
}

 /**
  * @brief :设置频率
  * @param :
  *			@FreqPoint:频率设置参数
  * @note  :值不大于127
  * @retval:无
  */
void NRF_Set_Freq( uint8_t FreqPoint )
{
    NRF_Write_Reg(  RF_CH, FreqPoint & 0x7F );
}

 /**
  * @brief :设置模式
  * @param :
  *			@Mode:模式发送模式或接收模式
  * @note  :无
  * @retval:无
  */
void NRF_Set_Mode( NRF_ModeType Mode )
{
    uint8_t controlreg = 0;
	controlreg = NRF_Read_Reg( CONFIG );
	
    if( Mode == MODE_TX )       
	{
		controlreg &= ~( 1<< PRIM_RX );
	}
    else if( Mode == MODE_RX )  
    { 
        controlreg |= ( 1<< PRIM_RX ); 
    }

    NRF_Write_Reg( CONFIG, controlreg );
}

/**
  * @brief :NRF检测
  * @param :无
  * @note  :无
  * @retval:无
  */ 
uint8_t NRF_check( void )
{
	uint8_t i;
	uint8_t buf[5]={ 0xC2,0xC2,0xC2,0xC2,0xC2 };
	uint8_t read_buf[ 5 ] = { 0 };
	 
    NRF_Write_Buf( TX_ADDR, buf, 5 );			//写入5个字节的地址
    NRF_Read_Buf( TX_ADDR, read_buf, 5 );		//读出写入的地址  
    
    for( i = 0; i < 5; i++ )
        if( buf[ i ] != read_buf[ i ] )
            return ERROR;
    
    return SUCCESS;     
}

/**
  * @brief :NRF发送一次数据
  * @param :
  *			@txbuf:待发送数据首地址
  *			@Length:发送数据长度
  * @note  :无
  * @retval:
  *			MAX_TX：达到最大重发次数
  *			TX_OK：发送完成
  *			0xFF:其他原因
  */ 
uint8_t NRF_TxPacket( uint8_t *txbuf, uint8_t Length )
{
	uint8_t l_Status = 0;
	uint16_t l_MsTimes = 0;
	
    NRF_Flush_Tx_FIFO();
	
	NRF_SET_CE_LOW( );		
	NRF_Write_Buf( WR_TX_PLOAD, txbuf, Length );		//写数据到TX BUF 32字节
    NRF_SET_CE_HIGH( );
    
	while( 0 != NRF_GET_IRQ_STATUS( ))
	{
		SysTick_Delay_Ms( 1 );
		if( 500 == l_MsTimes++ )						//500ms还没有发送成功，重新初始化设备
		{
			NRF_Gpio_Init( );
			NRF_Init( );
			NRF_Set_Mode( MODE_TX );
			break;
		}
	}
	
	l_Status = NRF_Read_Status_Register();				//读状态寄存器
	NRF_Write_Reg( STATUS, l_Status );					//清除TX_DS或MAX_RT中断标志
	NRF_SET_CE_LOW( );
	if( l_Status & TX_OK )	//发送完成
	{
		return TX_OK;
	}
	
	if( l_Status & MAX_TX )	//达到最大重发次数
	{
		NRF_Flush_Tx_FIFO();
		return MAX_TX; 
	}

	return l_Status;	//其他原因发送失败
}

/**
  * @brief :NRF接收数据
  * @param :
  *			@rxbuf:接收数据存放地址
  * @note  :无
  * @retval:接收到的数据个数
  */ 
uint8_t NRF_RxPacket( uint8_t *rxbuf )
{
	uint8_t l_Status = 0, l_RxLength = 0;
	
	NRF_Flush_Rx_FIFO();
	
	while( 0 != NRF_GET_IRQ_STATUS( ));
	
	l_Status = NRF_Read_Reg( STATUS );		//读状态寄存器
	NRF_Write_Reg( STATUS,l_Status );		//清中断标志
	if( l_Status & RX_OK)	//接收到数据
	{
		l_RxLength = NRF_Read_Reg( R_RX_PL_WID );		//读取接收到的数据个数
		NRF_Read_Buf( RD_RX_PLOAD,rxbuf,l_RxLength );	//接收到数据 
		NRF_Write_Reg( FLUSH_RX,0xff );				//清除RX FIFO
		return l_RxLength; 
	}	
	
	return 0;				//没有收到数据	
}

 /**
  * @brief :RF24L01模块初始化
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Init( void )
{
    NRF_SET_CE_LOW( );
	
	NRF_Write_Reg( SETUP_AW, AW_3BYTES );     		//地址宽度 3个字节
	
    NRF_Write_Reg( SETUP_RETR, ARD_1000US |
                        ( REPEAT_CNT & 0x0F ) );    //自动重发
	
	NRF_Write_Reg( EN_AA,   ( 1 << ENAA_P0 ) );   	//打开自动应答
    //NRF_Write_Reg( FEATRUE, ( 1 << EN_DPL ) );		//使能动态数据长度				
	//NRF_Write_Reg( DYNPD,   ( 1 << DPL_P0 ) ); 		//使能通道0动态数据长度
	
    NRF_Set_Speed( SPEED_250K );                    //空中速率250k
    NRF_Set_Power( POWER_0DBM );                    //发射功率0dBm
    
	NRF_Write_Reg( CONFIG, ( 1 << EN_CRC ) |     	//使能CRC 1个字节
						   ( 1 << PWR_UP ) );    	//开启设备
								  
    NRF_Set_Freq(0x6E);                             //通道（频率）
    
    //NRF_SET_CE_HIGH( );
}
