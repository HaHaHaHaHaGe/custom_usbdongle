/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Water_Address 0x00
#define Recoilless_Address 0x01
#define HumanSensor_Address 0x02
#define Local_Address 0xfd
#define Multi_Address 0xfc

#define Command_Shutdown 0x0f
#define Command_GetData 0x00
#define Command_SetData 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
// USB repoter
//bat  button1 button2 encode hall
unsigned char Recoilless_Send[64] = {0};

unsigned char USB_Send[64] = {0};
unsigned char USB_Recv[64] = {0};
unsigned char UART_Send[256] = {0};
unsigned char UART_Recv[256] = {0};
unsigned char UART_Recv_u8;
int UART_Recvloc = 0;
unsigned char USB_Event = 0;


unsigned char Water_Flag = 0;
unsigned char Recoilless_Flag = 0;
unsigned char HumanSensor_Flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t USB_GetData(uint8_t *data,uint32_t dataNum)
{
    uint32_t len=0;
    USBD_CUSTOM_HID_HandleTypeDef   *hhid;
  hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;//???????
    
    for(len=0;len<dataNum;len++){
        *data=hhid->Report_buf[len];
        data++;
    }
    return dataNum;
}

void RFNormalMode()
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
}

void RFConfigMode()
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
}

void RF_init()
{
	unsigned char data1[64] = {0xc0 ,0x00 ,0x03 ,0x2f ,0x1a ,0x04};
	//unsigned char data2[64] = {0xc1 ,0xc1 ,0xc1 ,0x2c ,0x18 ,0x84};
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) != 1);
	RFConfigMode();
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) != 1);
	UART_Recvloc = 0;
	HAL_Delay(50);
	HAL_UART_Transmit(&huart2,data1,6,1000);
	while(1);
}

void RF_Send(unsigned char *data,unsigned int len)
{
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) != 1);
	HAL_UART_Transmit(&huart2,data,len,1000);
}
void SendContrlData(unsigned char dstaddr,unsigned char command,unsigned char *data)
{
	//head:0xff
	//dst_addr : [4]
	//src_addt : [5]
	//data.....
	//end: 0xfe
	unsigned char send[13] = {0xff,0,0,command,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],0xfe};

	send[1] = dstaddr;

	send[2] = Local_Address;
	RF_Send(send,13);
}

void RecvTick()
{
	static unsigned char flag = 0;
	static int location;
	unsigned char Recv_data[64];
	int Recv_data_len = 0;
	unsigned char i = 0;
	static unsigned char count = 0;
	while(1)
	{
		if(UART_Recv[count] == 0xff)
		{
			if(flag == 1)
			{
				UART_Recv[location] = 0;
			}
			
			location = count;
			flag = 1;
		}
		if(UART_Recv[count] == 0xfe)
		{
			UART_Recv[count] = 0;
			if(flag != 1)
			{
				continue;
			}
			UART_Recv[location] = 0;
			flag = 0;
			if(count < location)
				Recv_data_len = (count + 256) - location;
			else
				Recv_data_len = count - location;
			if(Recv_data_len >= 64)
				continue;
			for(int j = 0;j<Recv_data_len;j++)
				Recv_data[j] = UART_Recv[(unsigned char)(location + 1 + j)];
			break;
		}
		if(i==255)
			break;
		i++;
		count++;
	}
	if(Recv_data_len == 0)
		return;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	if(Recv_data[0] != Local_Address)
		return;
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	unsigned char check  = 0;

//	if(Recv_data[1] == Water_Address)
//		check = 1;
//	if(Recv_data[1] == Recoilless_Address)
//		check = 2;

	switch(Recv_data[1])
	{
		case Water_Address:
			Water_Flag = 1;
			USB_Send[0] = Recv_data[2];
			USB_Send[1] = Recv_data[3];
			USB_Send[2] = Recv_data[4];
			USB_Send[3] = Recv_data[5];
			USB_Send[4] = Recv_data[6];
			break;
		case Recoilless_Address:
			Recoilless_Flag = 1;
			USB_Send[5] = Recv_data[2];
			USB_Send[6] = Recv_data[3];
			break;
		case HumanSensor_Address:
			HumanSensor_Flag = 1;
			USB_Send[7] = Recv_data[2];
			USB_Send[8] = Recv_data[3];
			break;
	}
}

unsigned char Send_Tick_WaterCount80ms = 0;
unsigned char Send_Tick_RecoillessCount80ms = 0;
unsigned char Send_Tick_HumanSensorCount80ms = 0;

unsigned char Send_Tick_RadioCount80ms = 0;
void SendTick()
{
	///////////////////////////////////////////////////////////////////////////
	if(Water_Flag == 0)
	{
		Send_Tick_WaterCount80ms++;
		if(Send_Tick_WaterCount80ms >= 12)
		{
			USB_Send[61] = 0;
		}
	}
	else
	{
		Send_Tick_WaterCount80ms = 0;
		USB_Send[61] = 1;
	}
	if(Recoilless_Flag == 0)
	{
		Send_Tick_RecoillessCount80ms++;
		if(Send_Tick_RecoillessCount80ms >= 12)
		{
			USB_Send[62] = 0;
		}
	}	
	else
	{
		Send_Tick_RecoillessCount80ms = 0;
		USB_Send[62] = 1;
	}
	
	if(HumanSensor_Flag == 0)
	{
		Send_Tick_HumanSensorCount80ms++;
		if(Send_Tick_HumanSensorCount80ms >= 12)
		{
			USB_Send[63] = 0;
		}
	}	
	else
	{
		Send_Tick_HumanSensorCount80ms = 0;
		USB_Send[63] = 1;
	}
	
	
	if(USB_Send[61] == 0 || USB_Send[62] == 0)// || USB_Send[63] == 0)
	{
		Send_Tick_RadioCount80ms++;
		if(Send_Tick_RadioCount80ms >= 12)
		{
			SendContrlData(Water_Address,Command_GetData,"80 20   ");
			SendContrlData(Recoilless_Address,Command_GetData,"80 40   ");
			SendContrlData(HumanSensor_Address,Command_GetData,"80 60   ");
		}
	}
	else
	{
		Send_Tick_RadioCount80ms = 0;
	}
	if(USB_Send[61] != 0 && USB_Send[62] != 0)
	{
		SendContrlData(Recoilless_Address,Command_SetData,Recoilless_Send);
	}
	
	
	Water_Flag = 0;
	Recoilless_Flag = 0;
	HumanSensor_Flag = 0;
	///////////////////////////////////////////////////////////////////////////
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	HAL_Delay(200);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2, UART_Recv, 256);
	
	//HAL_Delay(200);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim16);
	
	
	
	//RF_init();
	RFNormalMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//USART2->CR1 = 0x0000010d; 
	//	DMA1_Channel5->CCR = 0xaf;
  while (1)
  {
		if(USB_Event == 1)
		{
			unsigned char len = USB_GetData(USB_Recv,64);
			switch(USB_Recv[0])
			{
				case Recoilless_Address:
					Recoilless_Send[0] = USB_Recv[1];
					break;
				case HumanSensor_Address:
					SendContrlData(HumanSensor_Address,Command_Shutdown,"        ");
					break;
				case Water_Address:
					SendContrlData(Water_Address,Command_Shutdown,"        ");
					break;
			}

			//HAL_UART_Transmit(&huart2,USB_Recv,len,1000);
			//memcpy(USB_Send,USB_Recv,64);
			USB_Event = 0;
		}
		
//		*(unsigned int*)&USB_Send[16] = USART1->CR1;
//		*(unsigned int*)&USB_Send[20] = USART1->CR3;
//		*(unsigned int*)&USB_Send[24] = USART1->BRR;
//		*(unsigned int*)&USB_Send[28] = USART1->ISR;
//		
//		
//		*(unsigned int*)&USB_Send[0] = DMA1_Channel5->CCR;
//		*(unsigned int*)&USB_Send[4] = DMA1_Channel5->CNDTR;
//		*(unsigned int*)&USB_Send[8] = DMA1_Channel5->CPAR;
//		*(unsigned int*)&USB_Send[12] = DMA1_Channel5->CMAR;
		
		
		RecvTick();
		//RF_Send(0x0001,0x18,"test1",5);
		//HAL_Delay(50);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 9999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 4799;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 799;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
