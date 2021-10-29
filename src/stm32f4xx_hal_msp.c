#include "main.h"


/**
 *@brief UART Msp Initialization 
 *@param huart
 *@retval none 
 * 
 * 
 */

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpioInit;

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // initialise tx pin 
    gpioInit.Pin = GPIO_PIN_2;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FAST;
    gpioInit.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOA, &gpioInit);

    // initialise rx pin
    gpioInit.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpioInit);

}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpioInit;

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // pin6: scl, pin9: sda
    gpioInit.Pin = GPIO_PIN_6 | GPIO_PIN_9;
    gpioInit.Mode = GPIO_MODE_AF_OD;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FAST;
    gpioInit.Alternate = GPIO_AF4_I2C1;

    HAL_GPIO_Init(GPIOB, &gpioInit);

    // reset i2c1
    __HAL_RCC_I2C1_FORCE_RESET();

    __HAL_RCC_I2C1_RELEASE_RESET();
}