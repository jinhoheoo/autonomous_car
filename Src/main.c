/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 4024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "delay.h"
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t Left_INC_Value1 = 0;
uint32_t Left_INC_Value2 = 0;
uint32_t Left_echoTime = 0;
uint8_t Left_captureFlag = 0;
extern uint8_t Left_distance  = 0;
uint32_t Forward_INC_Value1 = 0;
uint32_t Forward_INC_Value2 = 0;
uint32_t Forward_echoTime = 0;
uint8_t Forward_captureFlag = 0;
extern uint8_t Forward_distance  = 0;
uint32_t Right_INC_Value1 = 0;
uint32_t Right_INC_Value2 = 0;
uint32_t Right_echoTime = 0;
uint8_t Right_captureFlag = 0;
extern uint8_t Right_distance  = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//
//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
    {
        if (Left_captureFlag == 0) // if the first value is not captured
        {
          Left_INC_Value1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1); // read the first value
          Left_captureFlag = 1;  // set the first captured as true

            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }

        else if (Left_captureFlag == 1)   // if the first is already captured
        {
          Left_INC_Value2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);  // read second value
            __HAL_TIM_SET_COUNTER(&htim3, 0);  // reset the counter

            if (Left_INC_Value2 > Left_INC_Value1)
            {
              Left_echoTime = Left_INC_Value2-Left_INC_Value1;
            }

            else if (Left_INC_Value1 > Left_INC_Value2)
            {
              Left_echoTime = (0xffff - Left_INC_Value1) + Left_INC_Value2;
            }

            //distance = echoTime * .034/2;
            Left_distance = Left_echoTime/58;
            Left_captureFlag = 0; // set it back to false

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
            printf ("Left %d cm\r\n", Left_distance);


        }
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
    {
        if (Forward_captureFlag == 0) // if the first value is not captured
        {
          Forward_INC_Value1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2); // read the first value
          Forward_captureFlag = 1;  // set the first captured as true

            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
        }

        else if (Forward_captureFlag == 1)   // if the first is already captured
        {
          Forward_INC_Value2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);  // read second value
            __HAL_TIM_SET_COUNTER(&htim3, 0);  // reset the counter

            if (Forward_INC_Value2 > Forward_INC_Value1)
            {
              Forward_echoTime = Forward_INC_Value2-Forward_INC_Value1;
            }

            else if (Forward_INC_Value1 > Forward_INC_Value2)
            {
              Forward_echoTime = (0xffff - Forward_INC_Value1) + Forward_INC_Value2;
            }

            //distance = echoTime * .034/2;
            Forward_distance = Forward_echoTime/58;
            Forward_captureFlag = 0; // set it back to false

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC2);
            printf ("Forward %d cm\r\n", Forward_distance);
        }
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel1
    {
        if (Right_captureFlag == 0) // if the first value is not captured
        {
          Right_INC_Value1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3); // read the first value
          Right_captureFlag = 1;  // set the first captured as true

            // Now change the polarity to falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        }

        else if (Right_captureFlag == 1)   // if the first is already captured
        {
          Right_INC_Value2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);  // read second value
            __HAL_TIM_SET_COUNTER(&htim3, 0);  // reset the counter

            if (Right_INC_Value2 > Right_INC_Value1)
            {
              Right_echoTime = Right_INC_Value2-Right_INC_Value1;
            }

            else if (Right_INC_Value1 > Right_INC_Value2)
            {
              Right_echoTime = (0xffff - Right_INC_Value1) + Right_INC_Value2;
            }

            //distance = echoTime * .034/2;
            Right_distance = Right_echoTime/58;
            Right_captureFlag = 0; // set it back to false

            // set polarity to rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC3);
            printf ("Right %d cm\r\n", Right_distance);
        }
    }
}
void HCSR04_Left_Read (void)
{
    HAL_GPIO_WritePin(TRIG_Left_GPIO_Port, TRIG_Left_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    delay_us(10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_Left_GPIO_Port, TRIG_Left_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);

}

void HCSR04_Forward_Read (void)
{
    HAL_GPIO_WritePin(TRIG_Forward_GPIO_Port, TRIG_Forward_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    delay_us(10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_Forward_GPIO_Port, TRIG_Forward_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low


    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);

}

void HCSR04_Right_Read (void)
{
    HAL_GPIO_WritePin(TRIG_Right_GPIO_Port, TRIG_Right_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    delay_us(10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_Right_GPIO_Port, TRIG_Right_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC3);
}
extern char *State;
void MOTOR_Con(void)
{
  HAL_GPIO_WritePin(N1_GPIO_Port, N1_Pin, 1);
  HAL_GPIO_WritePin(N2_GPIO_Port, N2_Pin, 0);
//    TIM4->CCR3 = 700;  //오른쪽 바퀴
//    TIM4->CCR4 = 0; //왼쪽 바퀴
  if (Left_distance >= 40 && Forward_distance >= 40 && Right_distance >= 40)
  {

    TIM4->CCR3 = 800;
    TIM4->CCR4 = 900;
    State = "Forward";
  }

  if (Right_distance <= 35)
  {
    TIM4->CCR3 = 800;
    TIM4->CCR4 = 0;
    State = "Left";
  }
  else if (Left_distance <= 35)
  {
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 800;
    State = "Right";
  }
  else if (Forward_distance < 25)
  {
    HAL_GPIO_WritePin (N1_GPIO_Port, N1_Pin, 0);
    HAL_GPIO_WritePin (N2_GPIO_Port, N2_Pin, 1);
    TIM4->CCR3 = 800;
    TIM4->CCR4 = 800;
    State = "Back";
  }

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


  HAL_GPIO_WritePin(N1_GPIO_Port, N1_Pin, 1);
  HAL_GPIO_WritePin(N2_GPIO_Port, N2_Pin, 0);


  HAL_TIM_IC_Start_IT (&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT (&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT (&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim11);


  //HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
  //HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
  //HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
  //HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
