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
#ifndef __EXTI_H
#define __EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_type.h"

#define EXTI_ChannelPriority_Default 2

#define EXTI_GetLine(Pin)            (1 << Pinx)
#define EXTI_GetPortSourceGPIOx(Pin) (GPIO_GetPortNum(Pin))
#define EXTI_GetPinSourcex(Pin)      (GPIO_GetPinNum(Pin))

#define CHANGE  LL_EXTI_TRIGGER_RISING_FALLING
#define FALLING LL_EXTI_TRIGGER_FALLING
#define RISING  LL_EXTI_TRIGGER_RISING
    
typedef void(*EXTI_CallbackFunction_t)(void);
    
void EXTIx_Init(uint8_t Pin, EXTI_CallbackFunction_t function, uint8_t Trigger_Mode, uint8_t ChannelPriority);
void attachInterrupt(uint8_t Pin, EXTI_CallbackFunction_t function, uint8_t Trigger_Mode);
void detachInterrupt(uint8_t Pin);

#ifdef __cplusplus
}
#endif

#endif