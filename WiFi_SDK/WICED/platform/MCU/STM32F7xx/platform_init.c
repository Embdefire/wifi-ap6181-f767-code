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
 * Define default STM32H7xx initialization functions
 */
#include "stdio.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "platform_sleep.h"
#include "platform_config.h"
#include "platform_toolchain.h"
#include "platform/wwd_platform_interface.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void error_handler(void)
{
    /* User may add here some code to deal with this error */
    while(1)
    {
    }
}
static int32_t       stm32f2_clock_needed_counter = 0;
#define WICED_ENABLE_INTERRUPTS()   __asm("CPSIE i")  /**< Enable interrupts to start task switching in MICO RTOS. */
#define WICED_DISABLE_INTERRUPTS()  __asm("CPSID i")  /**< Disable interrupts to stop task switching in MICO RTOS. */

platform_result_t platform_mcu_powersave_disable( void )
{
#ifndef WICED_DISABLE_MCU_POWERSAVE

    return PLATFORM_SUCCESS;
#else
    return PLATFORM_FEATURE_DISABLED;
#endif
}

platform_result_t platform_mcu_powersave_enable( void )
{
#ifndef WICED_DISABLE_MCU_POWERSAVE
    return PLATFORM_SUCCESS;
#else
    return PLATFORM_FEATURE_DISABLED;
#endif
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            TODO : Fill the table below
  *            System Clock source            =
  *            SYSCLK(Hz)                     =
  *            HCLK(Hz)                       =
  *            AHB Prescaler                  =
  *            APB1 Prescaler                 =
  *            APB2 Prescaler                 =
  *            HSE Frequency(Hz)              =
  *            PLL_M                          =
  *            PLL_N                          =
  *            PLL_P                          =
  *            PLL_Q                          =
  *            VDD(V)                         =
  *            Main regulator output voltage  =
  *            Flash Latency(WS)              =
  * @param  None
  * @retval None
  */
static void system_clock_config(void)
{

}
/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/* STM32F2 common clock initialization function
 * This brings up enough clocks to allow the processor to run quickly while initializing memory.
 * Other platform specific clock init can be done in init_platform() or init_architecture()
 */
WEAK void platform_init_system_clocks( void )
{
#ifndef BOOTLOADER
    /* Initialize watchdog */
//    platform_watchdog_init( );
#endif
    system_clock_config();
}


WEAK void platform_init_memory( void )
{

}

void platform_init_connectivity_module( void )
{
    host_platform_init( );
}

WEAK void platform_init_external_devices( void )
{

}

void rcc_apb2_clock_enable(uint32_t rcc_apb2_peripheral, FunctionalState new_state)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONAL_STATE(new_state));

    if (new_state != DISABLE)
    {
        RCC->APB2ENR |= rcc_apb2_peripheral;
    }
    else
    {
        RCC->APB2ENR &= ~rcc_apb2_peripheral;
    }
}

static void MPU_Config(void)
{

}


void SD_LowLevel_Init(void)
{
		int i;
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 使能 SDMMC 时钟 */
    __HAL_RCC_SDMMC1_CLK_ENABLE();
  
    /* 使能 GPIOs 时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
  
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
    /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /*设置引脚为不需要上、下拉模式*/  
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    /*设置引脚速率为高速 */      
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    /*设置为SDIO1复用 */  
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/  
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    /*设置引脚的输出类型为推挽输出*/    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /*设置引脚为不需要上、下拉模式*/  
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    /*设置引脚速率为高速 */ 
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    /*设置为SDIO1复用 */  
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    /*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/ 
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    //启用WIFI模块


		/*使能引脚时钟*/	
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/*选择要控制的GPIO引脚*/															   
		GPIO_InitStruct.Pin = GPIO_PIN_13;	
		/*设置引脚的输出类型为推挽输出*/
		GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      
		/*设置引脚为上拉模式*/
		GPIO_InitStruct.Pull  = GPIO_PULLUP;
		/*设置引脚速率为高速 */   
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST; 
		/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);	

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);  

		for(i=0;i<0xffff;i++)
		{
				__nop();
		}
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);  

}



void platform_init_mcu_infrastructure( void )
{
    uint8_t i;

#ifdef INTERRUPT_VECTORS_IN_RAM
    SCB->VTOR = 0x24000000; /* Change the vector table to point to start of SRAM */
#endif /* ifdef INTERRUPT_VECTORS_IN_RAM */

    __HAL_RCC_SYSCFG_CLK_ENABLE();

    MPU_Config();

    /* Enable I-Cache */
//  SCB_EnableICache();

    /* Enable D-Cache */
//    SCB_EnableDCache();

    /* 初始化中断优先级*/
    for ( i = 0; i < 100; i++ )
    {
        HAL_NVIC_SetPriority( i, 0xf, 0);
    }
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    platform_init_rtos_irq_priorities();
    platform_init_peripheral_irq_priorities();

    /*初始化GPIO IRQ管理器 */
//    platform_gpio_irq_manager_init();


#ifndef WICED_DISABLE_MCU_POWERSAVE
    /* Initialize MCU powersave */
//    platform_mcu_powersave_init( );
//    platform_mcu_powersave_disable( );

    /* Initialize RTC */
//    platform_rtc_init( );
#endif /* ifndef WICED_DISABLE_MCU_POWERSAVE */
}

void platform_mcu_reset( void )
{
    NVIC_SystemReset( );

    /* Loop forever */
    while ( 1 )
    {
    }
}
