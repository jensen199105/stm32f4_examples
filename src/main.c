/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c 
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UART_BUFFER_LEN                     100
#define TIMEOUT                             10000000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef gUartHandle;
I2C_HandleTypeDef gI2cHandle;
ACCELERO_DrvTypeDef *accDrv = &Lsm303agrDrv;
GYRO_DrvTypeDef *gyroDrv = &L3gd20Drv;
char gOutBuffer[UART_BUFFER_LEN];
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void EXTILine0_Config(void);
static void UART2_Config(void);
static void I2C1_Config(void);
static void ACC_Config(void);
static void debugPrint(const char *fmt, ...);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* This sample code shows how to use STM32F4xx GPIO HAL API to toggle PD12, PD13,
    PD14, and PD14 IOs (connected to LED4, LED3, LED5 and LED6 on STM32F411E-DISCO board (MB1115B)) 
    in an infinite loop.
    To proceed, 3 steps are required: */

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
 
  /* Configure LED3, LED4, LED5 and LED6 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  
  /* Configure the system clock to 100 MHz */
  SystemClock_Config();
    
  /* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
  EXTILine0_Config();

  /* Configure uart2 */
  UART2_Config();

  /* configure i2c */
  // I2C1_Config();
  
  ACC_Config();
  
  // char txBuffer[UART_BUFFER_LEN];
  uint8_t accId;
  uint8_t ctrl[6];
  int8_t data[3];
  int iLoop;
  int16_t data16bit[3];

  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG1_A, ACC_INIT_LOWBYTE);
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A, ACC_INIT_HIGHBYTE);

  ctrl[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG1_A);
  ctrl[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A);
  ctrl[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG3_A);
  ctrl[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A);
  ctrl[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG5_A);
  ctrl[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG6_A);

  for (iLoop = 0; iLoop < 6; ++iLoop) {
      debugPrint("ctrl reg %d is %02X\r\n", iLoop, ctrl[iLoop]);
      HAL_Delay(100);
  }
  /* Infinite loop */
  while (1)
  {
      // snprintf(txBuffer, UART_BUFFER_LEN, "Tick number is %lu\n", HAL_GetTick());
      
      // HAL_UART_Transmit(&gUartHandle, (uint8_t*)txBuffer, strlen(txBuffer), TIMEOUT);
      debugPrint("Tick number is %lu\r\n", HAL_GetTick());

      HAL_Delay(1000);

      accId = accDrv->ReadID();

      data[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_L_A);
      data[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_L_A);
      data[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_L_A);

      debugPrint("8bit; x: %hd, y: %hd, z: %hd\r\n", data[0], data[1], data[2]);
      accDrv->GetXYZ(data16bit);

      debugPrint("x: %hd, y: %hd, z: %hd\r\n", data16bit[0], data16bit[1], data16bit[2]);

      HAL_Delay(1000);

      uint8_t gyroId = gyroDrv->ReadID();

      debugPrint("gyro id: %#.2X\r\n", gyroId);

      HAL_Delay(1000);

      debugPrint("Accelerometer product id is %hhu\r\n", accId);

      HAL_Delay(1000);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTILine0_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

static void UART2_Config(void)
{   
    gUartHandle.Instance = USART2;

    gUartHandle.Init.BaudRate = 115200;
    gUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    gUartHandle.Init.StopBits = UART_STOPBITS_1;
    gUartHandle.Init.Parity = UART_PARITY_NONE;
    gUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    gUartHandle.Init.Mode = UART_MODE_TX_RX;
    gUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    if(HAL_UART_Init(&gUartHandle) != HAL_OK) {
        Error_Handler();
    }
} 

static void I2C1_Config(void)
{
    gI2cHandle.Instance = I2C1;
    gI2cHandle.Init.OwnAddress1 = 0x43;
    // configure i2c clock to 100k
    gI2cHandle.Init.ClockSpeed = 100000;
    gI2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    gI2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    gI2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    gI2cHandle.Init.OwnAddress2 = 0x00;
    gI2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    gI2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    HAL_StatusTypeDef status;

    if ((status = HAL_I2C_Init(&gI2cHandle)) != HAL_OK) {
        snprintf(gOutBuffer, UART_BUFFER_LEN, "i2c config fault, errorcode: %d\n", (int)status);
        HAL_UART_Transmit(&gUartHandle, (uint8_t *)gOutBuffer, strlen(gOutBuffer), TIMEOUT);
        Error_Handler();
    }
}

static void ACC_Config()
{
    uint16_t initValue = (ACC_INIT_HIGHBYTE << 8) + ACC_INIT_LOWBYTE;

    accDrv->Init(initValue);
}


static void debugPrint(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsprintf(gOutBuffer, fmt, args);

    HAL_UART_Transmit(&gUartHandle, (uint8_t *)gOutBuffer, strlen(gOutBuffer), TIMEOUT);

    va_end(args);
}
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    /* Toggle LED3 */
    BSP_LED_Toggle(LED3);
    /* Toggle LED4 */
    BSP_LED_Toggle(LED4);    
    /* Toggle LED5 */
    BSP_LED_Toggle(LED5);   
    /* Toggle LED6 */
    BSP_LED_Toggle(LED6);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED5 on */
  BSP_LED_On(LED5);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
