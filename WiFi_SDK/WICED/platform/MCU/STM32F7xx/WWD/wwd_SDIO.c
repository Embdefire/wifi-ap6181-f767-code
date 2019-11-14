/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 * Defines WWD SDIO functions for STM32H7xx MCU
 */
#include <string.h> /* For memcpy */
#include "wwd_platform_common.h"
#include "wwd_bus_protocol.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_sdio_interface.h"
#include "platform/wwd_bus_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "network/wwd_network_constants.h"

#include "platform_cmsis.h"
#include "platform_peripheral.h"
#include "platform_config.h"
#include "wwd_rtos_isr.h"

/******************************************************
 *             Constants
 ******************************************************/

#define COMMAND_FINISHED_CMD52_TIMEOUT_LOOPS (100000)
#define COMMAND_FINISHED_CMD53_TIMEOUT_LOOPS (100000)
#define SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS    (100000)
#define SDIO_DMA_TIMEOUT_LOOPS               (1000000)
#define MAX_TIMEOUTS                         (30)
#define SDIO_ERROR_MASK                      ( SDMMC_STA_CCRCFAIL | SDMMC_STA_DCRCFAIL | SDMMC_STA_CTIMEOUT | SDMMC_STA_DTIMEOUT | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR )
#define SDIO_IRQ_CHANNEL                     (0x31)
#define BUS_LEVEL_MAX_RETRIES                (5)
#define SDIO_ENUMERATION_TIMEOUT_MS          (500)

/******************************************************
 *             Structures
 ******************************************************/

typedef struct
{
    /*@shared@*//*@null@*/uint8_t* data;
    uint16_t length;
} sdio_dma_segment_t;

/******************************************************
 *             Variables
 ******************************************************/

static const uint32_t bus_direction_mapping[ ] =
{
    [BUS_READ] = SDMMC_TRANSFER_DIR_TO_SDMMC,
    [BUS_WRITE] = SDMMC_TRANSFER_DIR_TO_CARD
};

ALIGNED_PRE(4) static uint8_t temp_dma_buffer[ MAX( 2 * 1024, WICED_LINK_MTU+ 64 ) ] ALIGNED(4);
static uint8_t*               user_data;
static uint32_t               user_data_size;
static uint8_t*               dma_data_source;
static uint32_t               dma_transfer_size;
static host_semaphore_type_t  sdio_transfer_finished_semaphore;
static wiced_bool_t           sdio_transfer_failed;
static uint32_t               current_command;
static wwd_bus_transfer_direction_t current_transfer_direction;

/******************************************************
 *             Static Function Declarations
 ******************************************************/

static uint32_t sdio_get_blocksize_dctrl( sdio_block_size_t block_size );
static sdio_block_size_t find_optimal_block_size( uint32_t data_size );
static void sdio_prepare_data_transfer( wwd_bus_transfer_direction_t direction, sdio_block_size_t block_size, /*@unique@*/uint8_t* data, uint16_t data_size ) /*@modifies dma_data_source, user_data, user_data_size, dma_transfer_size@*/;

/******************************************************
 *             Function definitions
 ******************************************************/
void platform_mcu_powersave_exit_notify( void )
{
}

#ifndef  WICED_DISABLE_MCU_POWERSAVE


static void sdio_oob_irq_handler( void* arg )
{
    UNUSED_PARAMETER( arg );
    WWD_BUS_STATS_INCREMENT_VARIABLE( oob_intrs );
    platform_mcu_powersave_exit_notify( );
    wwd_thread_notify_irq( );
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */

static void sdio_enable_bus_irq( void )
{
    SDMMC1->MASK |= SDMMC_IT_DCRCFAIL | SDMMC_IT_DTIMEOUT | SDMMC_IT_RXOVERR | SDMMC_IT_TXUNDERR | SDMMC_IT_DATAEND;
}

static void sdio_disable_bus_irq( void )
{
    SDMMC1->MASK = 0;
}




#ifndef WICED_DISABLE_MCU_POWERSAVE
wwd_result_t host_enable_oob_interrupt( void )
{
    /* Set GPIO_B[1:0] to input. One of them will be re-purposed as OOB interrupt */
		/*初始化引脚*/
//    platform_gpio_init( &wifi_sdio_pins[ WWD_PIN_SDIO_OOB_IRQ ], INPUT_HIGH_IMPEDANCE );
		/*使能引脚中断*/
    platform_gpio_irq_enable( &wifi_sdio_pins[ WWD_PIN_SDIO_OOB_IRQ ], sdio_oob_irq_handler, 0 );

    return WWD_SUCCESS;
}

uint8_t host_platform_get_oob_interrupt_pin( void )
{
    return WICED_WIFI_OOB_IRQ_GPIO_PIN;
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */
extern void SD_LowLevel_Init(void);


#define SD_CardInfo HAL_SD_CardInfoTypedef
  
#define MSD_OK                        ((uint8_t)0x00)
#define MSD_ERROR                     ((uint8_t)0x01)
static SD_HandleTypeDef uSdHandle;
static SD_CardInfo      uSdCardInfo;

/**
  * @brief  初始化SD外设
  * @param  hsd: SD 句柄
  * @param  Params
  * @retval None
  */
void BSP_SD_MspInit(SD_HandleTypeDef *hsd, void *Params)
{
  static DMA_HandleTypeDef dma_rx_handle;
  static DMA_HandleTypeDef dma_tx_handle;
  GPIO_InitTypeDef gpio_init_structure;

  /* 使能 SDMMC 时钟 */
  __HAL_RCC_SDMMC1_CLK_ENABLE();


  /* 使能 GPIOs 时钟 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* 配置GPIO复用推挽、上拉、高速模式 */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_SDMMC1;
  
  /* GPIOC 配置 */
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  /* GPIOD 配置 */
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

//  /* SDIO 中断配置 */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
    
		
		  
//  /* 使能 DMA2 时钟 */
//  __DMA2_CLK_ENABLE();
		
	///* 配置 DMA 接收参数 */
	//dma_rx_handle.Init.Channel             = DMA_CHANNEL_4;
	//dma_rx_handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	//dma_rx_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
	//dma_rx_handle.Init.MemInc              = DMA_MINC_ENABLE;
	//dma_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	//dma_rx_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
	//dma_rx_handle.Init.Mode                = DMA_PFCTRL;
	//dma_rx_handle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
	//dma_rx_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
	//dma_rx_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	//dma_rx_handle.Init.MemBurst            = DMA_MBURST_INC4;
	//dma_rx_handle.Init.PeriphBurst         = DMA_PBURST_INC4;

	//dma_rx_handle.Instance = DMA2_Stream3;

	///* 关联DMA句柄 */
	//__HAL_LINKDMA(hsd, hdmarx, dma_rx_handle);

	///* 初始化传输数据流为默认值 */
	//HAL_DMA_DeInit(&dma_rx_handle);

	///* 配置 DMA 数据流 */
	//HAL_DMA_Init(&dma_rx_handle);

	///* 配置 DMA 发送参数 */
	//dma_tx_handle.Init.Channel             = DMA_CHANNEL_4;
	//dma_tx_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	//dma_tx_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
	//dma_tx_handle.Init.MemInc              = DMA_MINC_ENABLE;
	//dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	//dma_tx_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
	//dma_tx_handle.Init.Mode                = DMA_PFCTRL;
	//dma_tx_handle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
	//dma_tx_handle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
	//dma_tx_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	//dma_tx_handle.Init.MemBurst            = DMA_MBURST_INC4;
	//dma_tx_handle.Init.PeriphBurst         = DMA_PBURST_INC4;

	//dma_tx_handle.Instance = DMA2_Stream6;

	///* 关联 DMA 句柄 */
	//__HAL_LINKDMA(hsd, hdmatx, dma_tx_handle);

	///* 初始化传输数据流为默认值 */
	//HAL_DMA_DeInit(&dma_tx_handle);

	///* 配置 DMA 数据流 */
	//HAL_DMA_Init(&dma_tx_handle); 

	///* 配置DMA接收传输完成中断 */
	//HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 6, 0);
	//HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

	///* 配置DMA发送传输完成中断 */
	//HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 6, 0);
	//HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}


/**
  * @brief  初始化SD卡设备
  * @retval SD卡状态
  */
extern HAL_SD_ErrorTypedef HAL_SDIO_WIFI_Init(SD_HandleTypeDef *hsd);
extern HAL_SD_ErrorTypedef SD_PowerON(SD_HandleTypeDef *hsd); 
uint8_t BSP_SD_Init(void)
{ 
  uint8_t sd_state = MSD_OK;
  /* 定义SDMMC句柄 */
  uSdHandle.Instance = SDMMC1;
  GPIO_InitTypeDef gpio_init_structure;
	
//  /* 初始化SD底层驱动 */
  /* 使能 SDMMC 时钟 */
  __HAL_RCC_SDMMC1_CLK_ENABLE();
  /* 使能 GPIOs 时钟 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* 配置GPIO复用推挽、上拉、高速模式 */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_SDMMC1;
  
  /* GPIOC 配置 */
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  /* GPIOD 配置 */
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

//  /* SDIO 中断配置 */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  
	/*分配锁资源并对其进行初始化*/
  uSdHandle.Lock = HAL_UNLOCKED;
	HAL_SD_MspInit(&uSdHandle);
	
  SD_InitTypeDef tmpinit;
  tmpinit.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  tmpinit.ClockBypass         = SDMMC_CLOCK_BYPASS_DISABLE;
  tmpinit.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_ENABLE;
  tmpinit.BusWide             = SDMMC_BUS_WIDE_1B;
  tmpinit.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  tmpinit.ClockDiv            = SDMMC_INIT_CLK_DIV;

	SDMMC_Init(uSdHandle.Instance, tmpinit);
	
	SD_PowerON(&uSdHandle); 
	
	
	    //启用WIFI模块
		int i=0;
    GPIO_InitTypeDef GPIO_InitStruct;		
    /*使能引脚时钟*/	
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /*选择要控制的GPIO引脚*/															   
    GPIO_InitStruct.Pin = GPIO_PIN_13;	
    /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      
    /*设置引脚为上拉模式*/
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    /*设置引脚速率为高速 */   
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
    /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	
//    /*禁用WiFi模块*/
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);  
		for(i=0;i<0xffff;i++)
		{
			__nop();
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);  
	
	 //SDMMC1->POWER |= SDMMC_POWER_PWRCTRL;

  /* HAL SD 初始化 */
//  if(HAL_SDIO_WIFI_Init(&uSdHandle) != SD_OK)
//  {
//    sd_state = MSD_ERROR;
//  }
 
  return  sd_state;
}

wwd_result_t host_platform_bus_init( void )
{
//    SDMMC_InitTypeDef sdio_init_structure;
//    wwd_result_t result;
//    uint8_t a;
//    HAL_StatusTypeDef HAL_return;

//    platform_mcu_powersave_disable( );

//    /* Reset SDIO Block */
//    SDMMC_PowerState_OFF( SDMMC1 );
//    __HAL_RCC_SDMMC1_FORCE_RESET( );
//    __HAL_RCC_SDMMC1_RELEASE_RESET( );

//    /* Enable the SDIO Clock */
//    __HAL_RCC_SDMMC1_CLK_ENABLE( );

//    result = host_rtos_init_semaphore( &sdio_transfer_finished_semaphore );
//    if ( result != WWD_SUCCESS )
//    {
//        return result;
//    }

//    /* Clear all SDIO interrupts */
//    SDMMC1->ICR = (uint32_t) 0xffffffff;

//    /* Turn on SDIO IRQ */
//    /* Must be lower priority than the value of configMAX_SYSCALL_INTERRUPT_PRIORITY */
//    /* otherwise FreeRTOS will not be able to mask the interrupt */
//    /* keep in mind that ARMCM7 interrupt priority logic is inverted, the highest value */
//    /* is the lowest priority */
//    HAL_NVIC_EnableIRQ( (IRQn_Type) SDIO_IRQ_CHANNEL );

//#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_0
//    /* Set GPIO_B[1:0] to 00 to put WLAN module into SDIO mode */
//    platform_gpio_init( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0], OUTPUT_PUSH_PULL );
//    platform_gpio_output_low( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0] );
//#endif
//#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1
//    platform_gpio_init( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1], OUTPUT_PUSH_PULL );
//    platform_gpio_output_low( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1] );
//#endif

//    /* Setup GPIO pins for SDIO data & clock */

//		SD_LowLevel_Init();//WIFI SDIO初始化

//    /* SDMMC Initialization Frequency (400KHz max) for IP CLK 200MHz*/

//    sdio_init_structure.ClockDiv            = SDMMC_INIT_CLK_DIV;
//    sdio_init_structure.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
//    sdio_init_structure.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
//    sdio_init_structure.BusWide             = SDMMC_BUS_WIDE_1B;
//    sdio_init_structure.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
//    HAL_return                              = SDMMC_Init( SDMMC1, sdio_init_structure );
//		printf("HAL_return----->>>>%d\r\n",HAL_return);
//    HAL_return                             |= SDMMC_PowerState_ON( SDMMC1 );
//		printf("HAL_return----->>>>%d\r\n",HAL_return);
//    HAL_return                             |= SDMMC_SetSDMMCReadWaitMode( SDMMC1, SDMMC_READ_WAIT_MODE_CLK );
//		printf("HAL_return----->>>>%d\r\n",HAL_return);
//		
//				  /* SDIO 中断配置 */
//  HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);


//    if ( HAL_return |= HAL_OK )
//    {
//        return (~WWD_SUCCESS);
//    }

//    platform_mcu_powersave_enable( );
		BSP_SD_Init();
    return WWD_SUCCESS;
}

wwd_result_t host_platform_sdio_enumerate( void )
{
    wwd_result_t result;

    uint32_t loop_count;
    uint32_t data = 0;

    loop_count = 0;
    do
    {
        /* Send CMD0 to set it to idle state */
        result = host_platform_sdio_transfer( BUS_WRITE, SDIO_CMD_0, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL );

        /* CMD5. */
        result = host_platform_sdio_transfer( BUS_READ, SDIO_CMD_5, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL );

        /* Send CMD3 to get RCA. */
        result = host_platform_sdio_transfer( BUS_READ, SDIO_CMD_3, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, RESPONSE_NEEDED, &data );

        loop_count++;
        if ( loop_count >= (uint32_t) SDIO_ENUMERATION_TIMEOUT_MS )
        {

            return WWD_TIMEOUT;
        }
    } while ( ( result != WWD_SUCCESS ) && ( host_rtos_delay_milliseconds( (uint32_t) 1 ), ( 1 == 1 ) ) );
    /* If you're stuck here, check the platform matches your hardware */

    /* Send CMD7 with the returned RCA to select the card */
    result = host_platform_sdio_transfer( BUS_WRITE, SDIO_CMD_7, SDIO_BYTE_MODE, SDIO_1B_BLOCK, data, 0, 0, RESPONSE_NEEDED, NULL );

    return WWD_SUCCESS;
}

wwd_result_t host_platform_bus_deinit( void )
{

    wwd_result_t result = WWD_SUCCESS;
    uint32_t a;

    result = host_rtos_deinit_semaphore( &sdio_transfer_finished_semaphore );

    platform_mcu_powersave_disable( );

    sdio_disable_bus_irq( );

    SDMMC_PowerState_OFF( SDMMC1 );
    __HAL_RCC_SDMMC1_CLK_DISABLE( );

    for ( a = 0; a < WWD_PIN_SDIO_MAX; a++ )
    {
        platform_gpio_deinit( &wifi_sdio_pins[ a ] );
    }

#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_0
    platform_gpio_deinit( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0] );
#endif
#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1
    platform_gpio_deinit( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1] );
#endif

    /* Turn off SDIO IRQ */
    NVIC_DisableIRQ( (IRQn_Type) SDIO_IRQ_CHANNEL );

    platform_mcu_powersave_enable( );
    return result;
}

wwd_result_t host_platform_sdio_transfer( wwd_bus_transfer_direction_t direction, sdio_command_t command, sdio_transfer_mode_t mode, sdio_block_size_t block_size, uint32_t argument, /*@null@*/uint32_t* data, uint16_t data_size, sdio_response_needed_t response_expected, /*@out@*//*@null@*/uint32_t* response )
{
    uint32_t loop_count = 0;
    wwd_result_t result = WWD_UNSUPPORTED;
    uint16_t attempts = 0;

    wiced_assert( "Bad args", !((command == SDIO_CMD_53) && (data == NULL)) );

    if ( response != NULL )
    {
        *response = 0;
    }

    platform_mcu_powersave_disable( );

    restart: SDMMC1->ICR = (uint32_t) 0xFFFFFFFF;
    sdio_transfer_failed = WICED_FALSE;
    ++attempts;

    /* Check if we've tried too many times */
    if ( attempts >= (uint16_t) BUS_LEVEL_MAX_RETRIES )
    {
        result = WWD_SDIO_RETRIES_EXCEEDED;
        goto exit;
    }

    /* Prepare the data transfer register */
    current_command = command;
    if ( command == SDIO_CMD_53 )
    {
        sdio_enable_bus_irq( );

        /* Dodgy STM32 hack to set the CMD53 byte mode size to be the same as the block size */
        if ( mode == SDIO_BYTE_MODE )
        {
            block_size = find_optimal_block_size( data_size );
            if ( block_size < SDIO_512B_BLOCK )
            {
                argument = ( argument & (uint32_t) ( ~0x1FF ) ) | block_size;
            }
            else
            {
                argument = ( argument & (uint32_t) ( ~0x1FF ) );
            }
        }

        /* Prepare the SDIO for a data transfer */
        current_transfer_direction = direction;
        sdio_prepare_data_transfer( direction, block_size, (uint8_t*) data, data_size );

        /* Send the command */
        SDMMC1->ARG = argument;
        SDMMC1->CMD = (uint32_t) ( command | SDMMC_RESPONSE_SHORT | SDMMC_WAIT_NO | SDMMC_CPSM_ENABLE | SD_CMD_SD_ERASE_GRP_START );
				//修改过
        /* Wait for the whole transfer to complete */
        result = host_rtos_get_semaphore( &sdio_transfer_finished_semaphore, (uint32_t) 50, WICED_TRUE );
        if ( result != WWD_SUCCESS )
        {
            goto exit;
        }

        if ( sdio_transfer_failed == WICED_TRUE )
        {
            goto restart;
        }

        /* Check if there were any SDIO errors */
        if ( ( SDMMC1->STA & ( SDMMC_STA_DTIMEOUT | SDMMC_STA_CTIMEOUT ) ) != 0 )
        {
            goto restart;
        }
        else if ( ( ( SDMMC1->STA & ( SDMMC_STA_CCRCFAIL | SDMMC_STA_DCRCFAIL | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR ) ) != 0 ) )
        {
            wiced_assert( "SDIO communication failure", 0 );
            goto restart;
        }

        /* Wait till complete */
        loop_count = (uint32_t) SDIO_TX_RX_COMPLETE_TIMEOUT_LOOPS;
        do
        {
            loop_count--;
            if ( loop_count == 0 || ( ( SDMMC1->STA & SDIO_ERROR_MASK ) != 0 ) )
            {

                goto restart;
            }
//        } while ( ( SDMMC1->STA & ( SDMMC_STA_RXACT | SDMMC_STA_TXACT ) ) != 0 );//已修改	SDMMC_FLAG_CMDACT

        } while ( ( SDMMC1->STA & ( SDMMC_FLAG_CMDACT ) ) != 0 );//已修改	SDMMC_FLAG_CMDACT				

        if ( direction == BUS_READ )
        {
            memcpy( user_data, dma_data_source, (size_t) user_data_size );
        }

    }
    /* ALl SDIO CMD other than CMD53 */
    else
    {
				printf("命令编号：%d\r\n",command);
        uint32_t temp_sta;

        /* Send the command */
        SDMMC1->ARG = argument;
        SDMMC1->CMD = (uint32_t) ( command | SDMMC_RESPONSE_SHORT | SDMMC_WAIT_NO | SDMMC_CPSM_ENABLE );
				vTaskDelay(1);
        loop_count = (uint32_t) COMMAND_FINISHED_CMD52_TIMEOUT_LOOPS;
        do
        {
            temp_sta = SDMMC1->STA;
            loop_count--;
            if ( loop_count == 0 || ( ( response_expected == RESPONSE_NEEDED ) && ( ( temp_sta & SDIO_ERROR_MASK ) != 0 ) ) )
            {
                goto restart;
            }
        } while ( ( temp_sta & ( SDMMC_STA_RXACT | SDMMC_STA_TXACT ) ) != 0 );//已修改
    }
				printf("                        SDIO->STA->%d\r\n",SDMMC1->STA);
    if ( response != NULL )
    {
        *response = SDMMC1->RESP1;
    }
    result = WWD_SUCCESS;
    SDMMC1->CMD = 0;

    exit: platform_mcu_powersave_enable( );
    return result;
}

static void sdio_prepare_data_transfer( wwd_bus_transfer_direction_t direction, sdio_block_size_t block_size, /*@unique@*/uint8_t* data, uint16_t data_size ) /*@modifies dma_data_source, user_data, user_data_size, dma_transfer_size@*/
{
    /* Setup a single transfer using the temp buffer */
    user_data = data;
    user_data_size = data_size;
    dma_transfer_size = (uint32_t) ( ( ( data_size + (uint16_t) block_size - 1 ) / (uint16_t) block_size ) * (uint16_t) block_size );

    if ( direction == BUS_WRITE )
    {
        dma_data_source = data;
    }
    else
    {
        dma_data_source = temp_dma_buffer;
    }
		#define SDMMC_DATATIMEOUT                  ((uint32_t)0xFFFFFFFFU)
    SDMMC1->DTIMER    = (uint32_t) SDMMC_DATATIMEOUT;
    SDMMC1->DLEN      = dma_transfer_size;
    SDMMC1->DCTRL     = (uint32_t) sdio_get_blocksize_dctrl( block_size ) | bus_direction_mapping[ (int) direction ] | SDMMC_TRANSFER_MODE_BLOCK | SDMMC_DPSM_DISABLE | SDMMC_DCTRL_SDIOEN;

//    SDMMC1->IDMACTRL  = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
//    SDMMC1->IDMABASE0 = (uint32_t) dma_data_source;
		
//		SDMMC1->IDMACTRL  = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
    SDMMC1->FIFO = (uint32_t) dma_data_source;
		
}

wwd_result_t host_platform_enable_high_speed_sdio( void )
{
    SDMMC_InitTypeDef sdio_init_structure;
#define SDMMC_NSpeed_CLK_DIV  			SDMMC_INIT_CLK_DIV
    /* bus_clock = input_clock / ( 2 * Clockdiv) */
#ifdef SLOW_SDIO_CLOCK
    sdio_init_structure.ClockDiv       = (uint8_t) 10; /* 10 = 10 MHz if SDIO clock = 200MHz */
#else
    sdio_init_structure.ClockDiv       = SDMMC_NSpeed_CLK_DIV; /* 4 = 25MHz if SDIO clock = 200MHz */
#endif
    sdio_init_structure.ClockEdge      = SDMMC_CLOCK_EDGE_RISING;
    sdio_init_structure.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
#ifndef SDIO_1_BIT
    sdio_init_structure.BusWide        = SDMMC_BUS_WIDE_4B;
#else
    sdio_init_structure.BusWide        = SDMMC_BUS_WIDE_1B;
#endif
    sdio_init_structure.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;

    SDMMC_Init( SDMMC1, sdio_init_structure );
    return WWD_SUCCESS;
}

static sdio_block_size_t find_optimal_block_size( uint32_t data_size )
{

    if ( data_size > (uint32_t) 256 )
        return SDIO_512B_BLOCK;
    if ( data_size > (uint32_t) 128 )
        return SDIO_256B_BLOCK;
    if ( data_size > (uint32_t) 64 )
        return SDIO_128B_BLOCK;
    if ( data_size > (uint32_t) 32 )
        return SDIO_64B_BLOCK;
    if ( data_size > (uint32_t) 16 )
        return SDIO_32B_BLOCK;
    if ( data_size > (uint32_t) 8 )
        return SDIO_16B_BLOCK;
    if ( data_size > (uint32_t) 4 )
        return SDIO_8B_BLOCK;
    if ( data_size > (uint32_t) 2 )
        return SDIO_4B_BLOCK;

    return SDIO_4B_BLOCK;
}

static uint32_t sdio_get_blocksize_dctrl( sdio_block_size_t block_size )
{
    switch ( block_size )
    {
        case SDIO_1B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_1B;
        case SDIO_2B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_2B;
        case SDIO_4B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_4B;
        case SDIO_8B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_8B;
        case SDIO_16B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_16B;
        case SDIO_32B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_32B;
        case SDIO_64B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_64B;
        case SDIO_128B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_128B;
        case SDIO_256B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_256B;
        case SDIO_512B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_512B;
        case SDIO_1024B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_1024B;
        case SDIO_2048B_BLOCK:
            return SDMMC_DATABLOCK_SIZE_2048B;
        default:
            return 0;
    }
}

wwd_result_t host_platform_bus_enable_interrupt( void )
{
    SDMMC1->MASK = SDMMC_MASK_SDIOITIE;
    return WWD_SUCCESS;
}

wwd_result_t host_platform_bus_disable_interrupt( void )
{
    SDMMC1->MASK = 0;
    return WWD_SUCCESS;
}

wwd_result_t host_platform_unmask_sdio_interrupt( void )
{
    SDMMC1->MASK |= SDMMC_MASK_SDIOITIE;
    return WWD_SUCCESS;
}

void host_platform_bus_buffer_freed( wwd_buffer_dir_t direction )
{
    //UNUSED_PARAMETER( direction );
}

/******************************************************
 *             IRQ Handler Definitions
 ******************************************************/
//WWD_RTOS_DEFINE_ISR( sdio_irq )
void SDMMC1_IRQHandler()
{
    uint32_t intstatus = SDMMC1->STA;//寄存器数值不对

    WWD_BUS_STATS_INCREMENT_VARIABLE( sdio_intrs );

    if ( ( intstatus & ( SDMMC_STA_CCRCFAIL | SDMMC_STA_DCRCFAIL | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR ) ) != 0 )
    {
        WWD_BUS_STATS_INCREMENT_VARIABLE( error_intrs );
        wiced_assert("sdio error flagged",0);
        sdio_transfer_failed = WICED_TRUE;
        SDMMC1->ICR = (uint32_t) 0xffffffff;
        host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
    }
    else
    {
        if ((intstatus & (SDMMC_STA_CMDREND | SDMMC_STA_CMDSENT)) != 0)
        {
            if ( ( SDMMC1->RESP1 & 0x800 ) != 0 )
            {
                sdio_transfer_failed = WICED_TRUE;
                host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
            }

            /* Clear all command/response interrupts */
            SDMMC1->ICR = (SDMMC_STA_CMDREND | SDMMC_STA_CMDSENT);
        }

        if (intstatus & SDMMC_STA_DATAEND)
        {
            wwd_result_t result;
            SDMMC1->ICR      = SDMMC_STA_DATAEND;
            SDMMC1->DLEN     = 0;
            SDMMC1->DCTRL    = SDMMC_DCTRL_SDIOEN;
            //SDMMC1->IDMACTRL = SDMMC_DISABLE_IDMA;
            SDMMC1->CMD      = 0;
            result = host_rtos_set_semaphore( &sdio_transfer_finished_semaphore, WICED_TRUE );
            wiced_assert( "failed to set dma semaphore", result == WWD_SUCCESS );
        }

        /* Check whether the external interrupt was triggered */
        if (intstatus & SDMMC_STA_SDIOIT)
        {
            /* Clear the interrupt */
            SDMMC1->ICR   = SDMMC_STA_SDIOIT;
            /* Mask interrupt, to be unmasked later by WICED WWD thread */
					
            SDMMC1->MASK &= ~(SDMMC_MASK_SDIOITIE);
															
            /* Inform WICED WWD thread */
            wwd_thread_notify_irq( );
        }
    }
}

