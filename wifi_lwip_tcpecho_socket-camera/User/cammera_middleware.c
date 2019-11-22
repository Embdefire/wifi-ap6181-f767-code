#include "cammera_middleware.h"

#include "camera_data_queue.h"
#include "wifi_base_config.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"



uint32_t frame_counter = 0;
uint32_t line_counter = 0;
uint32_t vs_counter = 0;
uint32_t err_counter = 0;

uint32_t dcmi_start_counter = 0;
uint32_t dma_start_counter = 0;
uint32_t dma_complete_counter = 0;

//uint32_t img_real_len=0;

uint32_t  XferTransferNumber=0;
uint32_t 	XferCount = 0;
uint32_t 	XferSize = 0;
uint32_t 	pBuffPtr = 0;
uint8_t 	DCMI_State;
uint32_t	DMA2_Stream1_State;


/**
  * @brief  Enables or disables the specified DMAy Streamx.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *         to 7 to select the DMA Stream.
  * @param  NewState: new state of the DMAy Streamx. 
  *          This parameter can be: ENABLE or DISABLE.
  *
  * @note  This function may be used to perform Pause-Resume operation. When a
  *        transfer is ongoing, calling this function to disable the Stream will
  *        cause the transfer to be paused. All configuration registers and the
  *        number of remaining data will be preserved. When calling again this
  *        function to re-enable the Stream, the transfer will be resumed from
  *        the point where it was paused.          
  *    
  * @note  After configuring the DMA Stream (DMA_Init() function) and enabling the
  *        stream, it is recommended to check (or wait until) the DMA Stream is
  *        effectively enabled. A Stream may remain disabled if a configuration 
  *        parameter is wrong.
  *        After disabling a DMA Stream, it is also recommended to check (or wait
  *        until) the DMA Stream is effectively disabled. If a Stream is disabled 
  *        while a data transfer is ongoing, the current data will be transferred
  *        and the Stream will be effectively disabled only after the transfer of
  *        this single data is finished.            
  *    
  * @retval None
  */
void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected DMAy Streamx by setting EN bit */
    DMAy_Streamx->CR |= (uint32_t)DMA_SxCR_EN;
  }
  else
  {
    /* Disable the selected DMAy Streamx by clearing EN bit */
    DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
  }
}


/**
  * @brief  Enables or disables the DCMI JPEG format.
  * @note   The Crop and Embedded Synchronization features cannot be used in this mode.  
  * @param  NewState: new state of the DCMI JPEG format. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DCMI_JPEGCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
 
  if (NewState != DISABLE)
  {
    /* Enable the DCMI JPEG format */
    DCMI->CR |= (uint32_t)DCMI_CR_JPEG;
  }
  else
  {
    /* Disable the DCMI JPEG format */
    DCMI->CR &= ~(uint32_t)DCMI_CR_JPEG;
  }
}




/**
  * @brief  Returns the status of EN bit for the specified DMAy Streamx.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  *   
  * @note    After configuring the DMA Stream (DMA_Init() function) and enabling
  *          the stream, it is recommended to check (or wait until) the DMA Stream
  *          is effectively enabled. A Stream may remain disabled if a configuration
  *          parameter is wrong.
  *          After disabling a DMA Stream, it is also recommended to check (or wait 
  *          until) the DMA Stream is effectively disabled. If a Stream is disabled
  *          while a data transfer is ongoing, the current data will be transferred
  *          and the Stream will be effectively disabled only after the transfer
  *          of this single data is finished.  
  *      
  * @retval Current state of the DMAy Streamx (ENABLE or DISABLE).
  */
FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx)
{
  FunctionalState state = DISABLE;

  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));

  if ((DMAy_Streamx->CR & (uint32_t)DMA_SxCR_EN) != 0)
  {
    /* The selected DMAy Streamx EN bit is set (DMA is still transferring) */
    state = ENABLE;
  }
  else
  {
    /* The selected DMAy Streamx EN bit is cleared (DMA is disabled and 
        all transfers are complete) */
    state = DISABLE;
  }
  return state;
}


/**
  * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));

  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(DMAy_Streamx->NDTR));
}

/**
  * @brief  Enables or disables the DCMI Capture.
  * @param  NewState: new state of the DCMI capture. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DCMI_CaptureCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  if (NewState != DISABLE)
  {
    /* Enable the DCMI Capture */
    DCMI->CR |= (uint32_t)DCMI_CR_CAPTURE;
  }
  else
  {
    /* Disable the DCMI Capture */
    DCMI->CR &= ~(uint32_t)DCMI_CR_CAPTURE;
  }
}
/**
  * @brief  Enables or disables the DCMI interface.
  * @param  NewState: new state of the DCMI interface. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DCMI_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the DCMI by setting ENABLE bit */
    DCMI->CR |= (uint32_t)DCMI_CR_ENABLE;
  }
  else
  {
    /* Disable the DCMI by clearing ENABLE bit */
    DCMI->CR &= ~(uint32_t)DCMI_CR_ENABLE;
  }
}



/**
* @brief  关闭摄像头
* @param
* @param
*/
int close_camera()
{
//	DCMI_Cmd(DISABLE);

//	DMA_Cmd(DMA2_Stream1, DISABLE);

//	DCMI_DeInit();

//	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, DISABLE);

	return 0;
}

/**
* @brief  开始捕获图像，如果是连续获取模式，图片会源源不断的发往img_send_thread线程，等待发送出去
* @param
* @param
*/
void start_capture_img()
{
	DCMI_CaptureCmd(ENABLE);
}

/**
* @brief  停止捕获图像
* @param
* @param
*/
void stop_capture_img()
{
	DCMI_CaptureCmd(DISABLE);
}




void DCMI_Start(void)
{
	camera_data *data_p;
	
	/*获取写缓冲区指针，准备写入新数据*/
	data_p = cbWrite(&cam_circular_buff);
	
	if (data_p != NULL)	//若缓冲队列未满，开始传输
	{		
		dma_start_counter++;
		
		/*配置DMA传输*/
		//HAL_DCMI_Start_DMA((uint32_t )data_p->head, CAMERA_QUEUE_DATA_LEN);
		HAL_DCMI_Start_DMA(&DCMI_Handle, DCMI_MODE_CONTINUOUS, (uint32_t )data_p->head, CAMERA_QUEUE_DATA_LEN);

		DMA_Cmd(DMA2_Stream1, ENABLE);			//重新传输	
	}
	
	 dcmi_start_counter ++;
		
	DCMI_CaptureCmd(ENABLE);
}

void DCMI_Stop(void)
{	
	camera_data *data_p;
	
	/*关闭dma*/
    DMA_Cmd(DMA2_Stream1,DISABLE);
	while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}

//    DCMI_CaptureCmd(DISABLE);	
		

	/*获取正在操作的写指针*/	
	data_p = cbWriteUsing(&cam_circular_buff);
		
	/*计算dma传输的数据个数，用于减少jpeg搜索文件尾的时间*/	
	if (data_p != NULL)	
	{
		data_p->img_dma_len =0; //复位	
		
		if(CAMERA_QUEUE_DATA_LEN>65536*4)	//双dma buff
		{
			data_p->img_dma_len = (XferSize - DMA_GetCurrDataCounter(DMA2_Stream1))*4; //最后一个包
			
			if(dma_complete_counter>=2)
				data_p->img_dma_len += ((dma_complete_counter-1)*XferSize)*4 ;		//	dma_complete_counter个大小为XferSize的包
		}
		else	//单dma buf
			data_p->img_dma_len = (CAMERA_QUEUE_DATA_LEN/4 - DMA_GetCurrDataCounter(DMA2_Stream1))*4;
	}
	
	/*写入缓冲区完毕*/
	cbWriteFinish(&cam_circular_buff);
}

/**
  * @brief  Checks whether the DCMI interrupt has occurred or not.
  * @param  DCMI_IT: specifies the DCMI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg DCMI_IT_FRAME: Frame capture complete interrupt mask
  *            @arg DCMI_IT_OVF: Overflow interrupt mask
  *            @arg DCMI_IT_ERR: Synchronization error interrupt mask
  *            @arg DCMI_IT_VSYNC: VSYNC interrupt mask
  *            @arg DCMI_IT_LINE: Line interrupt mask
  * @retval The new state of DCMI_IT (SET or RESET).
  */
ITStatus DCMI_GetITStatus(uint16_t DCMI_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t itstatus = 0;
  
  /* Check the parameters */
  assert_param(IS_DCMI_GET_IT(DCMI_IT));
  
  itstatus = DCMI->MISR & DCMI_IT; /* Only masked interrupts are checked */
  
  if ((itstatus != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}


#define HIGH_ISR_MASK           (uint32_t)0x20000000
#define RESERVED_MASK           (uint32_t)0x0F7D0F7D  

/**
  * @brief  Checks whether the specified DMAy Streamx flag is set or not.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @param  DMA_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg DMA_FLAG_TCIFx:  Streamx transfer complete flag
  *            @arg DMA_FLAG_HTIFx:  Streamx half transfer complete flag
  *            @arg DMA_FLAG_TEIFx:  Streamx transfer error flag
  *            @arg DMA_FLAG_DMEIFx: Streamx direct mode error flag
  *            @arg DMA_FLAG_FEIFx:  Streamx FIFO error flag
  *         Where x can be 0 to 7 to select the DMA Stream.
  * @retval The new state of DMA_FLAG (SET or RESET).
  */
FlagStatus DMA_GetFlagStatus(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG)
{
  FlagStatus bitstatus = RESET;
  DMA_TypeDef* DMAy;
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));
  assert_param(IS_DMA_GET_FLAG(DMA_FLAG));

  /* Determine the DMA to which belongs the stream */
  if (DMAy_Streamx < DMA2_Stream0)
  {
    /* DMAy_Streamx belongs to DMA1 */
    DMAy = DMA1; 
  } 
  else 
  {
    /* DMAy_Streamx belongs to DMA2 */
    DMAy = DMA2; 
  }

  /* Check if the flag is in HISR or LISR */
  if ((DMA_FLAG & HIGH_ISR_MASK) != (uint32_t)RESET)
  {
    /* Get DMAy HISR register value */
    tmpreg = DMAy->HISR;
  }
  else
  {
    /* Get DMAy LISR register value */
    tmpreg = DMAy->LISR;
  }   
 
  /* Mask the reserved bits */
  tmpreg &= (uint32_t)RESERVED_MASK;

  /* Check the status of the specified DMA flag */
  if ((tmpreg & DMA_FLAG) != (uint32_t)RESET)
  {
    /* DMA_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* DMA_FLAG is reset */
    bitstatus = RESET;
  }

  /* Return the DMA_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Clears the DMAy Streamx's pending flags.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @param  DMA_FLAG: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCIFx:  Streamx transfer complete flag
  *            @arg DMA_FLAG_HTIFx:  Streamx half transfer complete flag
  *            @arg DMA_FLAG_TEIFx:  Streamx transfer error flag
  *            @arg DMA_FLAG_DMEIFx: Streamx direct mode error flag
  *            @arg DMA_FLAG_FEIFx:  Streamx FIFO error flag
  *         Where x can be 0 to 7 to select the DMA Stream.   
  * @retval None
  */
void DMA_ClearFlag(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_FLAG)
{
  DMA_TypeDef* DMAy;

  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));
  assert_param(IS_DMA_CLEAR_FLAG(DMA_FLAG));

  /* Determine the DMA to which belongs the stream */
  if (DMAy_Streamx < DMA2_Stream0)
  {
    /* DMAy_Streamx belongs to DMA1 */
    DMAy = DMA1; 
  } 
  else 
  {
    /* DMAy_Streamx belongs to DMA2 */
    DMAy = DMA2; 
  }

  /* Check if LIFCR or HIFCR register is targeted */
  if ((DMA_FLAG & HIGH_ISR_MASK) != (uint32_t)RESET)
  {
    /* Set DMAy HIFCR register clear flag bits */
    DMAy->HIFCR = (uint32_t)(DMA_FLAG & RESERVED_MASK);
  }
  else 
  {
    /* Set DMAy LIFCR register clear flag bits */
    DMAy->LIFCR = (uint32_t)(DMA_FLAG & RESERVED_MASK);
  }    
}

/**
  * @brief  Checks whether the  DCMI interface flag is set or not.
  * @param  DCMI_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg DCMI_FLAG_FRAMERI: Frame capture complete Raw flag mask
  *            @arg DCMI_FLAG_OVFRI: Overflow Raw flag mask
  *            @arg DCMI_FLAG_ERRRI: Synchronization error Raw flag mask
  *            @arg DCMI_FLAG_VSYNCRI: VSYNC Raw flag mask
  *            @arg DCMI_FLAG_LINERI: Line Raw flag mask
  *            @arg DCMI_FLAG_FRAMEMI: Frame capture complete Masked flag mask
  *            @arg DCMI_FLAG_OVFMI: Overflow Masked flag mask
  *            @arg DCMI_FLAG_ERRMI: Synchronization error Masked flag mask
  *            @arg DCMI_FLAG_VSYNCMI: VSYNC Masked flag mask
  *            @arg DCMI_FLAG_LINEMI: Line Masked flag mask
  *            @arg DCMI_FLAG_HSYNC: HSYNC flag mask
  *            @arg DCMI_FLAG_VSYNC: VSYNC flag mask
  *            @arg DCMI_FLAG_FNE: Fifo not empty flag mask
  * @retval The new state of DCMI_FLAG (SET or RESET).
  */
FlagStatus DCMI_GetFlagStatus(uint16_t DCMI_FLAG)
{
  FlagStatus bitstatus = RESET;
  uint32_t dcmireg, tempreg = 0;

  /* Check the parameters */
  assert_param(IS_DCMI_GET_FLAG(DCMI_FLAG));
  
  /* Get the DCMI register index */
  dcmireg = (((uint16_t)DCMI_FLAG) >> 12);
  
  if (dcmireg == 0x00) /* The FLAG is in RISR register */
  {
    tempreg= DCMI->RISR;
  }
  else if (dcmireg == 0x02) /* The FLAG is in SR register */
  {
    tempreg = DCMI->SR;
  }
  else /* The FLAG is in MISR register */
  {
    tempreg = DCMI->MISR;
  }
  
  if ((tempreg & DCMI_FLAG) != (uint16_t)RESET )
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the DCMI_FLAG status */
  return  bitstatus;
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
static void DMA_ChangeMemory(uint32_t Address, HAL_DMA_MemoryTypeDef memory)
{
  if(memory == MEMORY0)
  {
    /* change the memory0 address */
    DMA2_Stream1->M0AR = Address;
  }
  else
  {

    /* change the memory1 address */
    DMA2_Stream1->M1AR = Address;
  }

}

/* Private functions ---------------------------------------------------------*/
/** @defgroup DCMI_Private_Functions DCMI Private Functions
  * @{
  */
  /**
  * @brief  DMA conversion complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
void DCMI_DMAConvCplt(void)
{
  uint32_t tmp = 0;
 
  DCMI_State= HAL_DCMI_STATE_READY; 
	

  if(XferCount != 0)
  {
    /* Update memory 0 address location */
    tmp = ((DMA2_Stream1->CR) & DMA_SxCR_CT);
    if(((XferCount % 2) == 0) && (tmp != 0))
    {
      tmp = DMA2_Stream1->M0AR;
      DMA_ChangeMemory((tmp + (8*XferSize)), MEMORY0);
      XferCount--;
			dma_complete_counter++;

    }
    /* Update memory 1 address location */
    else if((DMA2_Stream1->CR & DMA_SxCR_CT) == 0)
    {
      tmp = DMA2_Stream1->M1AR;
      DMA_ChangeMemory((tmp + (8*XferSize)), MEMORY1);
      XferCount--;
			dma_complete_counter++;

    }
  }
  /* Update memory 0 address location */
  else if((DMA2_Stream1->CR & DMA_SxCR_CT) != 0)
  {
    DMA2_Stream1->M0AR = pBuffPtr;
  }
  /* Update memory 1 address location */
  else if((DMA2_Stream1->CR & DMA_SxCR_CT) == 0)
  {
    tmp = pBuffPtr;
    DMA2_Stream1->M1AR = (tmp + (4*XferSize));
    XferCount = XferTransferNumber;
  }

  if(DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) != RESET)
  {
    /* FRAME Callback */
//    HAL_DCMI_FrameEventCallback(hdcmi);
  }
}


#define TRANSFER_IT_ENABLE_MASK (uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_HTIE | \
                                           DMA_SxCR_TEIE | DMA_SxCR_DMEIE)

/**
  * @brief  Enables or disables the specified DMAy Streamx interrupts.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @param DMA_IT: specifies the DMA interrupt sources to be enabled or disabled. 
  *          This parameter can be any combination of the following values:
  *            @arg DMA_IT_TC:  Transfer complete interrupt mask
  *            @arg DMA_IT_HT:  Half transfer complete interrupt mask
  *            @arg DMA_IT_TE:  Transfer error interrupt mask
  *            @arg DMA_IT_FE:  FIFO error interrupt mask
  * @param  NewState: new state of the specified DMA interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DMA_ITConfig(DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));
  assert_param(IS_DMA_CONFIG_IT(DMA_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  /* Check if the DMA_IT parameter contains a FIFO interrupt */
  if ((DMA_IT & DMA_IT_FE) != 0)
  {
    if (NewState != DISABLE)
    {
      /* Enable the selected DMA FIFO interrupts */
      DMAy_Streamx->FCR |= (uint32_t)DMA_IT_FE;
    }    
    else 
    {
      /* Disable the selected DMA FIFO interrupts */
      DMAy_Streamx->FCR &= ~(uint32_t)DMA_IT_FE;  
    }
  }

  /* Check if the DMA_IT parameter contains a Transfer interrupt */
  if (DMA_IT != DMA_IT_FE)
  {
    if (NewState != DISABLE)
    {
      /* Enable the selected DMA transfer interrupts */
      DMAy_Streamx->CR |= (uint32_t)(DMA_IT  & TRANSFER_IT_ENABLE_MASK);
    }
    else
    {
      /* Disable the selected DMA transfer interrupts */
      DMAy_Streamx->CR &= ~(uint32_t)(DMA_IT & TRANSFER_IT_ENABLE_MASK);
    }    
  }
}

#define DMA_FLAG_TCIF1                    ((uint32_t)0x10000800)
#define HAL_DMA_STATE_READY_MEM0         0x11
void DMA2_Stream1_IRQHandler(void)
{
      /* Transfer Complete Interrupt management ***********************************/
  if (DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1)==SET)
  {
//    if(DMA_GetFlagStatus(DMA2_Stream1, DMA_IT_TC) != RESET)
    {
      if(((DMA2_Stream1->CR) & (uint32_t)(DMA_SxCR_DBM)) != 0)
      {

        /* Clear the transfer complete flag */
        DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);

        /* Current memory buffer used is Memory 1 */
        if((DMA2_Stream1->CR & DMA_SxCR_CT) == 0)
        {
          /* Transfer complete Callback for memory0 */
            DCMI_DMAConvCplt();
        }
        /* Current memory buffer used is Memory 0 */
        else if((DMA2_Stream1->CR & DMA_SxCR_CT) != 0) 
        {		

          /* Transfer complete Callback for memory0 */
            DCMI_DMAConvCplt();
        }
      }
      /* Disable the transfer complete interrupt if the DMA mode is not CIRCULAR */
      else
      {
        if(((DMA2_Stream1->CR) & (uint32_t)(DMA_SxCR_DBM)) == 0)
        {
          /* Disable the transfer complete interrupt */
          DMA_ITConfig(DMA2_Stream1, DMA_IT_TC,DISABLE);
        }
        /* Clear the transfer complete flag */
        DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1);
        /* Update error code */

        /* Change the DMA state */
        DMA2_Stream1_State = HAL_DMA_STATE_READY_MEM0;
 
        /* Transfer complete callback */
        DCMI_DMAConvCplt();
      }
    }
  }
}
/**
  * @brief  Clears the DCMI's interrupt pending bits.
  * @param  DCMI_IT: specifies the DCMI interrupt pending bit to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DCMI_IT_FRAME: Frame capture complete interrupt mask
  *            @arg DCMI_IT_OVF: Overflow interrupt mask
  *            @arg DCMI_IT_ERR: Synchronization error interrupt mask
  *            @arg DCMI_IT_VSYNC: VSYNC interrupt mask
  *            @arg DCMI_IT_LINE: Line interrupt mask
  * @retval None
  */
void DCMI_ClearITPendingBit(uint16_t DCMI_IT)
{
  /* Clear the interrupt pending Bit by writing in the ICR register 1 in the 
  corresponding pending Bit position*/
  
  DCMI->ICR = DCMI_IT;
}

void DCMI_IRQHandler(void)
{
	if(DCMI_GetITStatus(DCMI_IT_FRAME) == SET )
	{
			DCMI_ClearITPendingBit(DCMI_IT_FRAME);
			frame_counter ++;
			//1.停止DCMI传输
			DCMI_Stop();
			//2.根据缓冲区使用情况决定是否开启dma
			DCMI_Start();
			dma_complete_counter=0;
	}

    if(DCMI_GetITStatus(DCMI_IT_LINE) == SET )
	{
        DCMI_ClearITPendingBit(DCMI_IT_LINE);
        line_counter ++;
	}

    if(DCMI_GetITStatus(DCMI_IT_VSYNC) == SET )
	{
        DCMI_ClearITPendingBit(DCMI_IT_VSYNC);
        vs_counter ++;
	}

    if(DCMI_GetITStatus(DCMI_IT_ERR) == SET )
	{
        err_counter ++;
	}

}

