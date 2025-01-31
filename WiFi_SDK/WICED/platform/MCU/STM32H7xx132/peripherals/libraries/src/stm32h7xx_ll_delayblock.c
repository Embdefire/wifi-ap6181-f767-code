/**
  ******************************************************************************
  * @file    stm32h7xx_ll_delayblock.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-August-2017
  * @brief   DelayBlock Low Layer HAL module driver.
  *    
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Delay Block peripheral:
  *           + input clock frequency range 25MHz to 208MHz
  *           + up to 12 oversampling phases
  *         
  @verbatim
  ==============================================================================
                       ##### DelayBlock peripheral features #####
  ==============================================================================        
    [..] The Delay block is used to generate an Output clock which is de-phased from the Input
          clock. The phase of the Output clock is programmed by FW. The Output clock is then used
          to clock the receive data in i.e. a SDMMC or QSPI interface.
         The delay is Voltage and Temperature dependent, which may require FW to do re-tuning
          and recenter the Output clock phase to the receive data.
    
    [..] The Delay Block features include the following:
         (+) Input clock frequency range 25MHz to 208MHz.
         (+) Up to 12 oversampling phases.
         
                           ##### How to use this driver #####
  ==============================================================================
    [..]
      This driver is a considered as a driver of service for external devices drivers
      that interfaces with the DELAY peripheral.
      The DelayBlock_Enable() function, enables the DelayBlock instance, configure the delay line length
      and configure the Output clock phase.
      The DelayBlock_Disable() function, disables the DelayBlock instance by setting DEN flag to 0.
      
  
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "../../../../STM32H7xx/peripherals/libraries/inc/stm32h7xx_hal.h"

/** @addtogroup STM32H7xx_HAL_Driver
  * @{
  */

/** @defgroup DELAYBLOCK_LL DELAYBLOCK_LL
  * @brief Low layer module for Delay Block
  * @{
  */

#if defined(HAL_SD_MODULE_ENABLED) || defined(HAL_QSPI_MODULE_ENABLED)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup DelayBlock_LL_Exported_Functions Delay Block Low Layer Exported Functions
  * @{
  */

/** @defgroup HAL_DELAY_LL_Group1 Initialization de-initialization functions 
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
 
@endverbatim
  * @{
  */


/**
  * @brief  Enable the Delay Block instance.
  * @param  DLYBx: Pointer to DLYB instance.
  * @retval HAL status
  */
HAL_StatusTypeDef DelayBlock_Enable(DLYB_TypeDef *DLYBx)
{
  uint32_t i=0,N=0, lng=0, tuningOn = 1;

  assert_param(IS_DLYB_ALL_INSTANCE(DLYBx));

  DLYBx->CR = DLYB_CR_DEN | DLYB_CR_SEN;
    
  while((tuningOn != 0) && (i < DLYB_MAX_UNIT))
  {
    
    DLYBx->CFGR = 12 | (i << 8);
    HAL_Delay(1);
    if(((DLYBx->CFGR & DLYB_CFGR_LNGF) != 0) 
       && ((DLYBx->CFGR & DLYB_CFGR_LNG) != 0)
       && ((DLYBx->CFGR & DLYB_CFGR_LNG) != (DLYB_CFGR_LNG_11 | DLYB_CFGR_LNG_10)))
    {
      tuningOn = 0;
    }
    i++;

  }
  
  if(DLYB_MAX_UNIT != i)
  {

    lng = (DLYBx->CFGR & DLYB_CFGR_LNG) >> 16;
    N = 10;
    while((N>0) && ((lng >> N) == 0))
    {
      N--;
    }
    if(0 != N)
    {
      MODIFY_REG(DLYBx->CFGR, DLYB_CFGR_SEL, ((N/2)+1));
    
      /* Disable Selection phase */
      DLYBx->CR = DLYB_CR_DEN;
      return HAL_OK;
    }
  }

  /* Disable DLYB */
  DelayBlock_Disable(DLYBx);
  return HAL_ERROR;
  
}

/**
  * @brief  Disable the Delay Block instance.
  * @param  DLYBx: Pointer to DLYB instance.
  * @retval HAL status
  */
HAL_StatusTypeDef DelayBlock_Disable(DLYB_TypeDef *DLYBx)
{
  /* Disable DLYB */
  DLYBx->CR = 0;
  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* (HAL_SD_MODULE_ENABLED) & (HAL_QSPI_MODULE_ENABLED)*/
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
