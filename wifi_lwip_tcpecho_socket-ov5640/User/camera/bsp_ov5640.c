/**
  ******************************************************************************
  * @file    bsp_sdram.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   OV5640摄像头驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32F767 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "./camera/bsp_ov5640.h"
#include "./camera/ov5640_reg.h"
#include "./i2c/bsp_i2c.h"

#include "camera_data_queue.h"
#include "wifi_base_config.h"
#include "cammera_middleware.h"
#include "./delay/core_delay.h" 


DCMI_HandleTypeDef DCMI_Handle;
DMA_HandleTypeDef DMA_Handle_dcmi;
/** @addtogroup DCMI_Camera
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  TIMEOUT  2

ImageFormat_TypeDef ImageFormat;

void OV5640_JPEGConfig(ImageFormat_TypeDef ImageFormat)
{
    uint32_t i;

    OV5640_Reset();
    HAL_Delay(200);
    /* Initialize OV5640 */
    for(i=0; i<(sizeof(OV5640_JPEG_INIT)/4); i++)
    {
        OV5640_WriteReg(OV5640_JPEG_INIT[i][0], OV5640_JPEG_INIT[i][1]);
    }

    HAL_Delay(10);

    switch(ImageFormat)
    {
     case JPEG_160x120:
        {
            for(i=0; i<(sizeof(OV5640_160x120_JPEG)/4); i++)
            {
                OV5640_WriteReg(OV5640_160x120_JPEG[i][0], OV5640_160x120_JPEG[i][1]);
            }
            break;
        }
     case JPEG_320x240:
        {
            for(i=0; i<(sizeof(OV5640_320x240_JPEG)/4); i++)
            {
                OV5640_WriteReg(OV5640_320x240_JPEG[i][0], OV5640_320x240_JPEG[i][1]);
            }
            break;
        }
     case JPEG_640x480:
        {
            for(i=0; i<(sizeof(OV5640_640x480_JPEG)/4); i++)
            {
                OV5640_WriteReg(OV5640_640x480_JPEG[i][0], OV5640_640x480_JPEG[i][1]);
            }
            break;            
        }

     case JPEG_800x600:
        {
            for(i=0; i<(sizeof(OV5640_800x600_JPEG)/4); i++)
            {
                OV5640_WriteReg(OV5640_800x600_JPEG[i][0], OV5640_800x600_JPEG[i][1]);
            }
            break;
        }
     case JPEG_1024x768:
        {
            for(i=0; i<(sizeof(OV5640_1024x768_JPEG)/4); i++)
            {
                OV5640_WriteReg(OV5640_1024x768_JPEG[i][0], OV5640_1024x768_JPEG[i][1]);
            }
            break;
        }
		case JPEG_1280x960:
        {
            for(i=0; i<(sizeof(OV5640_1280x960_JPEG)/4); i++)
            {
                    OV5640_WriteReg(OV5640_1280x960_JPEG[i][0], OV5640_1280x960_JPEG[i][1]);
            }
            break;
        }
		case JPEG_1600x1200:
        {
            for(i=0; i<(sizeof(OV5640_1600x1200_JPEG)/4); i++)
            {
                    OV5640_WriteReg(OV5640_1600x1200_JPEG[i][0], OV5640_1600x1200_JPEG[i][1]);
            }
            break;
        }
        case JPEG_2048x1536:
        {
            for(i=0; i<(sizeof(OV5640_2048x1536_JPEG)/4); i++)
            {
                    OV5640_WriteReg(OV5640_2048x1536_JPEG[i][0], OV5640_2048x1536_JPEG[i][1]);
            }
            break;
        }
        case JPEG_2320x1740:
        {
            for(i=0; i<(sizeof(OV5640_2320x1740_JPEG)/4); i++)
            {
                    OV5640_WriteReg(OV5640_2320x1740_JPEG[i][0], OV5640_2320x1740_JPEG[i][1]);
            }
            break;
        }
        case JPEG_2592x1944:
        {
            for(i=0; i<(sizeof(OV5640_2592x1944_JPEG)/4); i++)
            {
                    OV5640_WriteReg(OV5640_2592x1944_JPEG[i][0], OV5640_2592x1944_JPEG[i][1]);
            }
            break;
        }
     default:
        {
            for(i=0; i<(sizeof(OV5640_320x240_JPEG)/4); i++)
            {
                OV5640_WriteReg(OV5640_320x240_JPEG[i][0], OV5640_320x240_JPEG[i][1]);
            }
            break;
        }
    }
}


/**
  * @brief  初始化控制摄像头使用的GPIO(I2C/DCMI)
  * @param  None
  * @retval None
  */
void OV5640_HW_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /***DCMI引脚配置***/
    /* 使能DCMI时钟 */
    DCMI_PWDN_GPIO_CLK_ENABLE();
    DCMI_RST_GPIO_CLK_ENABLE();
    DCMI_VSYNC_GPIO_CLK_ENABLE();
    DCMI_HSYNC_GPIO_CLK_ENABLE();
    DCMI_PIXCLK_GPIO_CLK_ENABLE();    
    DCMI_D0_GPIO_CLK_ENABLE();
    DCMI_D1_GPIO_CLK_ENABLE();
    DCMI_D2_GPIO_CLK_ENABLE();
    DCMI_D3_GPIO_CLK_ENABLE();    
    DCMI_D4_GPIO_CLK_ENABLE();
    DCMI_D5_GPIO_CLK_ENABLE();
    DCMI_D6_GPIO_CLK_ENABLE();
    DCMI_D7_GPIO_CLK_ENABLE();

    /*控制/同步信号线*/
    GPIO_InitStructure.Pin = DCMI_VSYNC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_PULLUP ;
    GPIO_InitStructure.Alternate = DCMI_VSYNC_AF;
    HAL_GPIO_Init(DCMI_VSYNC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_HSYNC_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_VSYNC_AF;
    HAL_GPIO_Init(DCMI_HSYNC_GPIO_PORT, &GPIO_InitStructure);


    GPIO_InitStructure.Pin = DCMI_PIXCLK_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_PIXCLK_AF;
    HAL_GPIO_Init(DCMI_PIXCLK_GPIO_PORT, &GPIO_InitStructure);

    /*数据信号*/
    GPIO_InitStructure.Pin = DCMI_D0_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D0_AF;
    HAL_GPIO_Init(DCMI_D0_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D1_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D1_AF;
    HAL_GPIO_Init(DCMI_D1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D2_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D2_AF;
    HAL_GPIO_Init(DCMI_D2_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D3_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D3_AF;
    HAL_GPIO_Init(DCMI_D3_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D4_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D4_AF;
    HAL_GPIO_Init(DCMI_D4_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D5_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D5_AF;
    HAL_GPIO_Init(DCMI_D5_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D6_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D6_AF;
    HAL_GPIO_Init(DCMI_D6_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_D7_GPIO_PIN;
    GPIO_InitStructure.Alternate = DCMI_D7_AF;
    HAL_GPIO_Init(DCMI_D7_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_PWDN_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;    
    HAL_GPIO_Init(DCMI_PWDN_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_RST_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;    
    HAL_GPIO_Init(DCMI_RST_GPIO_PORT, &GPIO_InitStructure);

    HAL_GPIO_WritePin(DCMI_RST_GPIO_PORT,DCMI_RST_GPIO_PIN,GPIO_PIN_RESET);
    /*PWDN引脚，高电平关闭电源，低电平供电*/
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_PORT,DCMI_PWDN_GPIO_PIN,GPIO_PIN_SET);
   
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_PORT,DCMI_PWDN_GPIO_PIN,GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DCMI_RST_GPIO_PORT,DCMI_RST_GPIO_PIN,GPIO_PIN_SET);
		//必须延时50ms,模块才会正常工作
		HAL_Delay(50);
}
/**
  * @brief  Resets the OV5640 camera.
  * @param  None
  * @retval None
  */
void OV5640_Reset(void)
{
	/*OV5640软件复位*/
  OV5640_WriteReg(0x3008, 0x80);
}

/**
  * @brief  读取摄像头的ID.
  * @param  OV5640ID: 存储ID的结构体
  * @retval None
  */
void OV5640_ReadID(OV5640_IDTypeDef *OV5640ID)
{

	/*读取寄存芯片ID*/
  OV5640ID->PIDH = OV5640_ReadReg(OV5640_SENSOR_PIDH);
  OV5640ID->PIDL = OV5640_ReadReg(OV5640_SENSOR_PIDL);
}

/**
  * @brief  配置 DCMI/DMA 以捕获摄像头数据
  * @param  None
  * @retval None
  */
void OV5640_DCMI_Init(void) 
{
  /*** 配置DCMI接口 ***/
  /* 使能DCMI时钟 */
  __HAL_RCC_DCMI_CLK_ENABLE();

  /* DCMI 配置*/
  DCMI_Handle.Instance              = DCMI;    
  DCMI_Handle.Init.SynchroMode      = DCMI_MODE_CONTINUOUS;
  DCMI_Handle.Init.SynchroMode      = DCMI_SYNCHRO_HARDWARE;
  DCMI_Handle.Init.PCKPolarity      = DCMI_PCKPOLARITY_RISING;
  DCMI_Handle.Init.VSPolarity       = DCMI_VSPOLARITY_LOW;
  DCMI_Handle.Init.HSPolarity       = DCMI_HSPOLARITY_LOW;
  DCMI_Handle.Init.CaptureRate      = DCMI_CR_ALL_FRAME;
  DCMI_Handle.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  HAL_DCMI_Init(&DCMI_Handle); 

	/* 配置中断 */
  HAL_NVIC_SetPriority(DCMI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn); 	

}


/**
  * @brief  配置 DCMI/DMA 以捕获摄像头数据
	* @param  DMA_Memory0BaseAddr:本次传输的目的首地址
  * @param DMA_BufferSize：本次传输的数据量(单位为字,即4字节)
  */
void OV5640_DMA_Config(uint32_t DMA_Memory0BaseAddr,uint32_t DMA_BufferSize)
{

  /* 配置DMA从DCMI中获取数据*/
  /* 使能DMA*/
  __HAL_RCC_DMA2_CLK_ENABLE(); 
  DMA_Handle_dcmi.Instance = DMA2_Stream1;
  DMA_Handle_dcmi.Init.Channel = DMA_CHANNEL_1;  
  DMA_Handle_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
  DMA_Handle_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
  DMA_Handle_dcmi.Init.MemInc = DMA_MINC_ENABLE;    //寄存器地址自增
  DMA_Handle_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  DMA_Handle_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  DMA_Handle_dcmi.Init.Mode = DMA_CIRCULAR;		    //循环模式
  DMA_Handle_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
  DMA_Handle_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  DMA_Handle_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  DMA_Handle_dcmi.Init.MemBurst = DMA_MBURST_INC4;
  DMA_Handle_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;

  /*DMA中断配置 */
  __HAL_LINKDMA(&DCMI_Handle, DMA_Handle, DMA_Handle_dcmi);
  __HAL_DMA_ENABLE_IT(&DMA_Handle_dcmi,DMA_IT_TE); 

	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  
  HAL_DMA_Init(&DMA_Handle_dcmi);
}



/**
  * @brief  帧同步回调函数.
  * @param  None
  * @retval None
  */
extern DMA_HandleTypeDef DMA_Handle_dcmi;

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	DCMI_IRQHandler_Funtion();

   //重新使能帧中断
//  __HAL_DCMI_ENABLE_IT(&DCMI_Handle,DCMI_IT_FRAME);
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
		DCMI_IRQHandler_Funtion();

		//重新使能帧中断
//		__HAL_DCMI_ENABLE_IT(&DCMI_Handle,DCMI_FLAG_VSYNCRI);
}

uint8_t fps=0;
uint8_t dispBuf[100];
/*简单任务管理*/
uint32_t Task_Delay[3]={0};

#define JPEG_MODO

int32_t open_camera(uint32_t *BufferSRC, uint32_t BufferSize)
{
		OV5640_IDTypeDef OV5640_Camera_ID;	

		/*5640 IO 初始化*/
		I2CMaster_Init(); 
		OV5640_HW_Init();
		/*DCMI 初始化*/
		OV5640_DCMI_Init();
		/*DMA 初始化*/
		OV5640_DMA_Config((uint32_t)BufferSRC, BufferSize);
		/*dcmi 开始 DMA传输 */  //使能DCMI采集数据
		HAL_DCMI_Start_DMA(&DCMI_Handle, DCMI_MODE_CONTINUOUS,(uint32_t)BufferSRC, BufferSize);

		__HAL_DCMI_DISABLE_IT(&DCMI_Handle,DCMI_IT_LINE|DCMI_IT_FRAME|DCMI_IT_ERR|DCMI_IT_OVR);
		__HAL_DCMI_ENABLE_IT(&DCMI_Handle,DCMI_IT_VSYNC);      //使能帧中断
		__HAL_DCMI_ENABLE(&DCMI_Handle);                       //使能DCMI
	
		
		/* 读取摄像头芯片ID，确定摄像头正常连接 */
		OV5640_ReadID(&OV5640_Camera_ID);

		if(OV5640_Camera_ID.PIDH  == 0x56)
		{
			printf("%x%x\r\n",OV5640_Camera_ID.PIDH ,OV5640_Camera_ID.PIDL);
		}
		else
		{
			printf("没有检测到OV5640摄像头，请重新检查连接。\r\n");
			while(1);  
		}
			/* 配置摄像头输出像素格式 */
		OV5640_JPEGConfig(JPEG_IMAGE_FORMAT);
		printf(" OV5640_JPEGConfig ok !!! \r\n");

		
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
