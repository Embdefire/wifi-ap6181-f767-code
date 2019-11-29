#include "cammera_middleware.h"

#include "camera_data_queue.h"
#include "wifi_base_config.h"
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"



uint32_t frame_counter = 0;

uint32_t dcmi_start_counter = 0;
uint32_t dma_start_counter = 0;
uint32_t dma_complete_counter = 0;



uint32_t  XferTransferNumber=0;
uint32_t 	XferCount = 0;
uint32_t 	XferSize = 0;
uint32_t 	pBuffPtr = 0;
uint8_t 	DCMI_State;
uint32_t	DMA2_Stream1_State;


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


extern DCMI_HandleTypeDef DCMI_Handle;
extern DMA_HandleTypeDef DMA_Handle_dcmi;

#if 0
#else 

/*
    // Enable the DCMI Capture 
    DCMI->CR |= (uint32_t)DCMI_CR_CAPTURE;
    // Disable the DCMI Capture
    DCMI->CR &= ~(uint32_t)DCMI_CR_CAPTURE;
*/


void DCMI_Start(void)
{

	camera_data *data_p;
	
	/*获取写缓冲区指针，准备写入新数据*/
	data_p = cbWrite(&cam_circular_buff);
	
	if (data_p != NULL)	//若缓冲队列未满，开始传输
	{		
		
		/*配置DMA传输*/
		HAL_DCMI_Start_DMA(&DCMI_Handle, DCMI_MODE_CONTINUOUS, (uint32_t )data_p->head, CAMERA_QUEUE_DATA_LEN);

		__HAL_DMA_ENABLE(&DMA_Handle_dcmi);		//使能DMA
		
		
		
	}

	DCMI->CR |= (uint32_t)DCMI_CR_CAPTURE;//使能 DCMI
}

void DCMI_Stop(void)
{	
	camera_data *data_p;
	
	/*关闭dma*/
 	__HAL_DMA_DISABLE(&DMA_Handle_dcmi);		//关掉DMA
 	while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}
	DCMI_CaptureCmd(DISABLE);

	/*获取正在操作的写指针*/	
	data_p = cbWriteUsing(&cam_circular_buff);
		
	/*计算dma传输的数据个数，用于减少jpeg搜索文件尾的时间*/	
	if (data_p != NULL)	
	{
		data_p->img_dma_len = (DCMI_Handle.XferSize - DMA_GetCurrDataCounter(DMA2_Stream1))*4; //最后一个包 
		frame_counter ++;
	}
	else
	{
		printf("获取操作指针错误\r\n");
		while(1);
	}
	
	/*写入缓冲区完毕*/
	cbWriteFinish(&cam_circular_buff);
}



/*帧中断实现*/
void DCMI_IRQHandler_Funtion(void)
{

	//frame_counter ++;
	//1.停止DCMI传输
	DCMI_Stop();
	//2.根据缓冲区使用情况决定是否开启dma
	DCMI_Start();
	
}


#endif
