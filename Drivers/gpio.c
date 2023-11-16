/*
 * MIT License
 * Copyright (c) 2019 _VIFEXTech
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "gpio.h"
/*GPIO 基本信息*/
const PinInfo_TypeDef PIN_MAP[] =
{
    /*GPIO_TypeDef* GPIOx;  //  对应GPIOx地址
    TIM_TypeDef* TIMx;      //  对应TIMx地址
    ADC_TypeDef* ADCx;      //  对应ADCx地址

    uint16_t GPIO_Pin_x;    //  对应GPIO_Pin位
    uint8_t TimerChannel;   //  对应定时器通道
    uint32_t ADC_Channel;*/  //  对应ADC通道
    {GPIOA, TIM2, ADC1,  LL_GPIO_PIN_0, 1, ADC_CHANNEL_0_NUMBER}, /* PA0 */
    {GPIOA, TIM2, ADC1,  LL_GPIO_PIN_1, 2, ADC_CHANNEL_1_NUMBER}, /* PA1 */
    {GPIOA, TIM2, ADC1,  LL_GPIO_PIN_2, 3, ADC_CHANNEL_2_NUMBER}, /* PA2 */
    {GPIOA, TIM2, ADC1,  LL_GPIO_PIN_3, 4, ADC_CHANNEL_3_NUMBER}, /* PA3 */
    {GPIOA, TIM3, ADC2,  LL_GPIO_PIN_4, 2, ADC_CHANNEL_4_NUMBER}, /* PA4 */
    {GPIOA, TIM2, ADC2,  LL_GPIO_PIN_5, 1, ADC_CHANNEL_5_NUMBER}, /* PA5 */
    {GPIOA, TIM3, ADC2,  LL_GPIO_PIN_6, 1, ADC_CHANNEL_6_NUMBER}, /* PA6 */
    {GPIOA, TIM3, ADC2,  LL_GPIO_PIN_7, 2, ADC_CHANNEL_7_NUMBER}, /* PA7 */
    {GPIOA, TIM1, NULL,  LL_GPIO_PIN_8, 1, ADC_CHANNEL_8_NUMBER}, /* PA8 */
    {GPIOA, TIM1, NULL,  LL_GPIO_PIN_9, 2, ADC_CHANNEL_9_NUMBER}, /* PA9 */
    {GPIOA, TIM1, NULL, LL_GPIO_PIN_10, 3, ADC_CHANNEL_10_NUMBER}, /* PA10 */
    {GPIOA, TIM1, NULL, LL_GPIO_PIN_11, 4, ADC_CHANNEL_11_NUMBER}, /* PA11 */
    {GPIOA, NULL, NULL, LL_GPIO_PIN_12, 0, ADC_CHANNEL_12_NUMBER}, /* PA12 */
    {GPIOA, NULL, NULL, LL_GPIO_PIN_13, 0, ADC_CHANNEL_13_NUMBER}, /* PA13 */
    {GPIOA, NULL, NULL, LL_GPIO_PIN_14, 0, ADC_CHANNEL_14_NUMBER}, /* PA14 */
    {GPIOA, NULL, NULL, LL_GPIO_PIN_15, 0, ADC_CHANNEL_15_NUMBER}, /* PA15 */

    {GPIOB, TIM3, ADC1,  LL_GPIO_PIN_0, 3, ADC_CHANNEL_0_NUMBER}, /* PB0 */
    {GPIOB, TIM3, ADC1,  LL_GPIO_PIN_1, 4, ADC_CHANNEL_0_NUMBER}, /* PB1 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_2, 0, ADC_Channel_X}, /* PB2 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_3, 0, ADC_Channel_X}, /* PB3 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_4, 0, ADC_Channel_X}, /* PB4 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_5, 0, ADC_Channel_X}, /* PB5 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_6, 1, ADC_Channel_X}, /* PB6 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_7, 2, ADC_Channel_X}, /* PB7 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_8, 3, ADC_Channel_X}, /* PB8 */
    {GPIOB, NULL, NULL,  LL_GPIO_PIN_9, 4, ADC_Channel_X}, /* PB9 */
    {GPIOB, NULL, NULL, LL_GPIO_PIN_10, 0, ADC_Channel_X}, /* PB10 */
    {GPIOB, NULL, NULL, LL_GPIO_PIN_11, 0, ADC_Channel_X}, /* PB11 */
    {GPIOB, NULL, NULL, LL_GPIO_PIN_12, 0, ADC_Channel_X}, /* PB12 */
    {GPIOB, NULL, NULL, LL_GPIO_PIN_13, 0, ADC_Channel_X}, /* PB13 */
    {GPIOB, NULL, NULL, LL_GPIO_PIN_14, 0, ADC_Channel_X}, /* PB14 */
    {GPIOB, NULL, NULL, LL_GPIO_PIN_15, 0, ADC_Channel_X}, /* PB15 */

    {GPIOC, NULL, ADC1,  LL_GPIO_PIN_0, 0, ADC_CHANNEL_0_NUMBER}, /* PC0 */
    {GPIOC, NULL, ADC1,  LL_GPIO_PIN_1, 0, ADC_CHANNEL_0_NUMBER}, /* PC1 */
    {GPIOC, NULL, ADC1,  LL_GPIO_PIN_2, 0, ADC_CHANNEL_0_NUMBER}, /* PC2 */
    {GPIOC, NULL, ADC1,  LL_GPIO_PIN_3, 0, ADC_CHANNEL_0_NUMBER}, /* PC3 */
    {GPIOC, NULL, ADC1,  LL_GPIO_PIN_4, 0, ADC_CHANNEL_0_NUMBER}, /* PC4 */
    {GPIOC, NULL, ADC1,  LL_GPIO_PIN_5, 0, ADC_CHANNEL_0_NUMBER}, /* PC5 */
    {GPIOC, NULL, NULL,  LL_GPIO_PIN_6, 1, ADC_Channel_X}, /* PC6 */
    {GPIOC, NULL, NULL,  LL_GPIO_PIN_7, 2, ADC_Channel_X}, /* PC7 */
    {GPIOC, NULL, NULL,  LL_GPIO_PIN_8, 3, ADC_Channel_X}, /* PC8 */
    {GPIOC, NULL, NULL,  LL_GPIO_PIN_9, 4, ADC_Channel_X}, /* PC9 */
    {GPIOC, NULL, NULL, LL_GPIO_PIN_10, 0, ADC_Channel_X}, /* PC10 UART4_TX/SDIO_D2 */
    {GPIOC, NULL, NULL, LL_GPIO_PIN_11, 0, ADC_Channel_X}, /* PC11 UART4_RX/SDIO_D3 */
    {GPIOC, NULL, NULL, LL_GPIO_PIN_12, 0, ADC_Channel_X}, /* PC12 UART5_TX/SDIO_CK */
    {GPIOC, NULL, NULL, LL_GPIO_PIN_13, 0, ADC_Channel_X}, /* PC13 TAMPER-RTC */
    {GPIOC, NULL, NULL, LL_GPIO_PIN_14, 0, ADC_Channel_X}, /* PC14 OSC32_IN */
    {GPIOC, NULL, NULL, LL_GPIO_PIN_15, 0, ADC_Channel_X}, /* PC15 OSC32_OUT */

    {GPIOD, NULL, NULL,  LL_GPIO_PIN_0, 0, ADC_Channel_X}, /* PD0 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_1, 0, ADC_Channel_X}, /* PD1 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_2, 0, ADC_Channel_X}, /* PD2 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_3, 0, ADC_Channel_X}, /* PD3 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_4, 0, ADC_Channel_X}, /* PD4 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_5, 0, ADC_Channel_X}, /* PD5 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_6, 0, ADC_Channel_X}, /* PD6 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_7, 0, ADC_Channel_X}, /* PD7 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_8, 0, ADC_Channel_X}, /* PD8 */
    {GPIOD, NULL, NULL,  LL_GPIO_PIN_9, 0, ADC_Channel_X}, /* PD9 */
    {GPIOD, NULL, NULL, LL_GPIO_PIN_10, 0, ADC_Channel_X}, /* PD10 */
    {GPIOD, NULL, NULL, LL_GPIO_PIN_11, 0, ADC_Channel_X}, /* PD11 */
    {GPIOD, NULL, NULL, LL_GPIO_PIN_12, 0, ADC_Channel_X}, /* PD12 */
    {GPIOD, NULL, NULL, LL_GPIO_PIN_13, 0, ADC_Channel_X}, /* PD13 */
    {GPIOD, NULL, NULL, LL_GPIO_PIN_14, 0, ADC_Channel_X}, /* PD14 */
    {GPIOD, NULL, NULL, LL_GPIO_PIN_15, 0, ADC_Channel_X}, /* PD15 */

#if defined(GPIOE)
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_0, 0, ADC_Channel_X}, /* PE0 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_1, 0, ADC_Channel_X}, /* PE1 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_2, 0, ADC_Channel_X}, /* PE2 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_3, 0, ADC_Channel_X}, /* PE3 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_4, 0, ADC_Channel_X}, /* PE4 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_5, 0, ADC_Channel_X}, /* PE5 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_6, 0, ADC_Channel_X}, /* PE6 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_7, 0, ADC_Channel_X}, /* PE7 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_8, 0, ADC_Channel_X}, /* PE8 */
    {GPIOE, NULL, NULL,  LL_GPIO_PIN_9, 0, ADC_Channel_X}, /* PE9 */
    {GPIOE, NULL, NULL, LL_GPIO_PIN_10, 0, ADC_Channel_X}, /* PE10 */
    {GPIOE, NULL, NULL, LL_GPIO_PIN_11, 0, ADC_Channel_X}, /* PE11 */
    {GPIOE, NULL, NULL, LL_GPIO_PIN_12, 0, ADC_Channel_X}, /* PE12 */
    {GPIOE, NULL, NULL, LL_GPIO_PIN_13, 0, ADC_Channel_X}, /* PE13 */
    {GPIOE, NULL, NULL, LL_GPIO_PIN_14, 0, ADC_Channel_X}, /* PE14 */
    {GPIOE, NULL, NULL, LL_GPIO_PIN_15, 0, ADC_Channel_X}, /* PE15 */
#endif

    {GPIOF, NULL, NULL,  LL_GPIO_PIN_0, 0, ADC_Channel_X}, /* PF0 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_1, 0, ADC_Channel_X}, /* PF1 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_2, 0, ADC_Channel_X}, /* PF2 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_3, 0, ADC_Channel_X}, /* PF3 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_4, 0, ADC_Channel_X}, /* PF4 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_5, 0, ADC_Channel_X}, /* PF5 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_6, 0, ADC_CHANNEL_0_NUMBER}, /* PF6 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_7, 0, ADC_CHANNEL_0_NUMBER}, /* PF7 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_8, 0, ADC_CHANNEL_0_NUMBER}, /* PF8 */
    {GPIOF, NULL, NULL,  LL_GPIO_PIN_9, 0, ADC_CHANNEL_0_NUMBER}, /* PF9 */
    {GPIOF, NULL, NULL, LL_GPIO_PIN_10, 0, ADC_CHANNEL_0_NUMBER}, /* PF10 */
    {GPIOF, NULL, NULL, LL_GPIO_PIN_11, 0, ADC_Channel_X}, /* PF11 */
    {GPIOF, NULL, NULL, LL_GPIO_PIN_12, 0, ADC_Channel_X}, /* PF12 */
    {GPIOF, NULL, NULL, LL_GPIO_PIN_13, 0, ADC_Channel_X}, /* PF13 */
    {GPIOF, NULL, NULL, LL_GPIO_PIN_14, 0, ADC_Channel_X}, /* PF14 */
    {GPIOF, NULL, NULL, LL_GPIO_PIN_15, 0, ADC_Channel_X}, /* PF15 */
};

/**
  * @brief  GPIO初始化
  * @param  GPIOx: GPIO地址
  * @param  GPIO_Pin_x: GPIO对应位
  * @param  pinMode_x: GPIO模式
  * @param  GPIO_Speed_x: GPIO速度
  * @retval 无
  */
void GPIOx_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x, pinMode_TypeDef pinMode_x, uint16_t GPIO_Speed_x)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t RCC_AHBPeriph_GPIOx;
//    GPIOMode_TypeDef GPIO_Mode_x;
//    GPIOOType_TypeDef GPIO_OType_x;
//    GPIOPuPd_TypeDef GPIO_PuPd_x;

    if(GPIOx == GPIOA)RCC_AHBPeriph_GPIOx = LL_AHB2_GRP1_PERIPH_GPIOA;
    else if(GPIOx == GPIOB)RCC_AHBPeriph_GPIOx = LL_AHB2_GRP1_PERIPH_GPIOB;
    else if(GPIOx == GPIOC)RCC_AHBPeriph_GPIOx = LL_AHB2_GRP1_PERIPH_GPIOC;
    else if(GPIOx == GPIOD)RCC_AHBPeriph_GPIOx = LL_AHB2_GRP1_PERIPH_GPIOD;
#if (defined(GPIOE) && defined(RCC_AHBPeriph_GPIOE))
    else if(GPIOx == GPIOE)RCC_AHBPeriph_GPIOx = LL_AHB2_GRP1_PERIPH_GPIOE;
#endif
    else if(GPIOx == GPIOF)RCC_AHBPeriph_GPIOx = LL_AHB2_GRP1_PERIPH_GPIOF;
    else return;

    if(pinMode_x == INPUT)
    {
			  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    }
    else if(pinMode_x == INPUT_PULLUP)
    {
			  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    }
    else if(pinMode_x == INPUT_PULLDOWN)
    {
			  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    }
    else if(pinMode_x == INPUT_ANALOG)
    {
        GPIO_InitStruct.Mode  = LL_GPIO_MODE_ANALOG;
				GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    }
    else if(pinMode_x == OUTPUT)
    {
				GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
			  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
				GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    }
    else if(pinMode_x == OUTPUT_OPEN_DRAIN)
    {
				GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
			  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
				GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    }
    else if(pinMode_x == OUTPUT_AF)
    {
				GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
			  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
				GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    }

    GPIO_InitStruct.Pin = GPIO_Pin_x;
    GPIO_InitStruct.Speed = GPIO_Speed_x;
			
    LL_AHB2_GRP1_EnableClock(RCC_AHBPeriph_GPIOx);
    LL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief  获取当前引脚对应的GPIOx编号
  * @param  Pin: 引脚编号
  * @retval 无
  */
uint8_t GPIO_GetPortNum(uint8_t Pin)
{
    if(PIN_MAP[Pin].GPIOx == GPIOA)return 0;
    else if(PIN_MAP[Pin].GPIOx == GPIOB)return 1;
    else if(PIN_MAP[Pin].GPIOx == GPIOC)return 2;
    else if(PIN_MAP[Pin].GPIOx == GPIOD)return 3;
    else if(PIN_MAP[Pin].GPIOx == GPIOE)return 4;
    else if(PIN_MAP[Pin].GPIOx == GPIOF)return 5;
    else return 0xFF;
}

/**
  * @brief  获取当前引脚对应的编号
  * @param  Pin: 引脚编号
  * @retval 无
  */
uint8_t GPIO_GetPinNum(uint8_t Pin)
{
    return GPIO_GetPinSource(PIN_MAP[Pin].GPIO_Pin_x);
}

/**
  * @brief  获取当前引脚对应的 PinSource
  * @param  GPIO_Pin_x: GPIO对应位
  * @retval PinSource
  */
uint8_t GPIO_GetPinSource(uint16_t GPIO_Pin_x)
{
    uint16_t PinSource = 0;
    while(GPIO_Pin_x > 1)
    {
        GPIO_Pin_x >>= 1;
        PinSource++;
    }
    return PinSource;
}
