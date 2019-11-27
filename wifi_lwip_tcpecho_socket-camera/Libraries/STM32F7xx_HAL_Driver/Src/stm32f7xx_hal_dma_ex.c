/**
  ******************************************************************************
  * @file    stm32f7xx_hal_dma_ex.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    23-September-2016 
  * @brief   DMA Extension HAL module driver
  *         This file provides firmware functions to manage the following 
  *         functionalities of the DMA Extension peripheral:
  *           + Extended features functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
  The DMA Extension HAL driver can be used as follows:
   (+) Start a multi buffer transfer using the HAL_DMA_MultiBufferStart() function
       for polling mode or HAL_DMA_MultiBufferStart_IT() for interrupt mode.

     -@-  In Memory-to-Memory transfer mode, Multi (Double) Buffer mode is not allowed.
     -@-  When Multi (Double) Buffer mode is enabled, the transfer is circular by default.
     -@-  In Multi (Double) buffer mode, it is possible to update the base address for 
          the AHB memory port on the fly (DMA_SxM0AR or DMA_SxM1AR) when the stream is enabled.
  
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "stm32f7xx_hal.h"

/** @addtogroup STM32F7xx_HAL_Driver
  * @{
  */

/** @defgroup DMAEx DMAEx
  * @brief DMA Extended HAL module driver
  * @{
  */

#ifdef HAL_DMA_MODULE_ENABLED

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private Constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup DMAEx_Private_Functions
  * @{
  */

static void DMA_MultiBufferSetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup DMAEx_Exported_Functions
  * @{
  */


/** @addtogroup DMAEx_Exported_Functions_Group1
  *
@verbatim
 ===============================================================================
                #####  Extended features functions  #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and 
          Start MultiBuffer DMA transfer
      (+) Configure the source, destination address and data length and 
          Start MultiBuffer DMA transfer with interrupt
      (+) Change on the fly the memory0 or memory1 address.
      
@endverbatim
  * @{
  */


/**
  * @brief 启动multi_buffer DMA传输。
  * @param  hdma      : pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: 			源内存缓冲区地址
  * @param  DstAddress: 			目标内存缓冲区地址
  * @param  SecondMemAddress: 多缓冲区传输时的第二个内存缓冲区地址
  * @param  DataLength: 			从源传输到目标的数据长度
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  /*检查参数 */
  assert_param(IS_DMA_BUFFER_SIZE(DataLength));
  
  /*在双缓冲模式下不支持内存到内存的传输 */
  if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
    status = HAL_ERROR;
  }
  else
  {
    /* 进程已锁定*/
    __HAL_LOCK(hdma);
    
    if(HAL_DMA_STATE_READY == hdma->State)
    {
      /*更改DMA外设状态 */
      hdma->State = HAL_DMA_STATE_BUSY; 
      
      /*启用双缓冲模式 */
      hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;
      
      /* 配置DMA流目标地址*/
      hdma->Instance->M1AR = SecondMemAddress;
      
      /* 配置源，目标地址和数据长度 */
      DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress, DataLength);
      
      /* 启用外围设备 */
      __HAL_DMA_ENABLE(hdma);
    }
    else
    {
      /* 返回错误状态 */
      status = HAL_BUSY;
    }
  }
  return status;
}

/**
  * @brief 	在启用中断的情况下启动多缓冲区DMA传输。
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: 源内存缓冲区地址
  * @param  DstAddress: 目标内存缓冲区地址
  * @param  SecondMemAddress: 多缓冲区传输时的第二个内存缓冲区地址
  * @param  DataLength: 从源传输到目标的数据长度
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength)
{
	
  HAL_StatusTypeDef status = HAL_OK;
  
  /* 校检参数*/
  assert_param(IS_DMA_BUFFER_SIZE(DataLength));
  
  /* 在双缓冲模式下不支持内存到内存的传输 */
  if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
    return HAL_ERROR;
  }
  
  /*进程已锁定 */
  __HAL_LOCK(hdma);
  
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /*更改DMA外设状态 */
    hdma->State = HAL_DMA_STATE_BUSY;
    
    /* 初始化错误代码 */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    
    /* 启用双缓冲模式 */
    hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;
    
    /* 配置DMA流目标地址 */
    hdma->Instance->M1AR = SecondMemAddress;
    
    /* 配置源，目标地址和数据长度 */
    DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress, DataLength); 
    
    /*清除所有标志 */
    __HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
    
    /*启用通用中断*/
    hdma->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    hdma->Instance->FCR |= DMA_IT_FE;
    
    if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
    {
      hdma->Instance->CR  |= DMA_IT_HT;
    }
    
    /* 启用外围设备 */
    __HAL_DMA_ENABLE(hdma); 
  }
  else
  {     
    /*流程已解锁*/
    __HAL_UNLOCK(hdma);	  
    
    /* 返回错误状态*/
    status = HAL_BUSY;
  }  
  return status; 
}

/**
  * @brief  Change the memory0 or memory1 address on the fly.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  Address:    The new address
  * @param  memory:     the memory to be changed, This parameter can be one of 
  *                     the following values:
  *                      MEMORY0 /
  *                      MEMORY1
  * @note   The MEMORY0 address can be changed only when the current transfer use
  *         MEMORY1 and the MEMORY1 address can be changed only when the current 
  *         transfer use MEMORY0.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory)
{
  if(memory == MEMORY0)
  {
    /* change the memory0 address */
    hdma->Instance->M0AR = Address;
  }
  else
  {
    /* change the memory1 address */
    hdma->Instance->M1AR = Address;
  }
  
  return HAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DMAEx_Private_Functions
  * @{
  */

/**
  * @brief  Set the DMA Transfer parameter.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
static void DMA_MultiBufferSetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;
  
  /* Peripheral to Memory */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;
    
    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Memory to Peripheral */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;
    
    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }
}

/**
  * @}
  */

#endif /* HAL_DMA_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
