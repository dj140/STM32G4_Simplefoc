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
#include "timer.h"

/*定时器编号枚举*/
typedef enum
{
    TIMER1, TIMER2, TIMER3, TIMER4, TIMER6, TIMER7, TIMER8,
    TIMER15, TIMER16, TIMER17, TIMER_MAX
} TIMER_Type;

/*定时中断回调函数指针数组*/
Timer_CallbackFunction_t TIM_Function[TIMER_MAX] = {0};

/**
  * @brief  启动或关闭指定定时器的时钟
  * @param  TIMx:定时器地址
  * @param  NewState: ENABLE启动，DISABLE关闭
  * @retval 无
  */
void TimerClockCmd(TIM_TypeDef* TIMx, FunctionalState NewState)
{
    if(TIMx == TIM1)
    {
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, NewState);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    }
    else if(TIMx == TIM2)
    {
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, NewState);
			  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    }
    else if(TIMx == TIM3)
    {
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, NewState);
				LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    }
    else if(TIMx == TIM4)
    {
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, NewState);
				LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    }
    else if(TIMx == TIM6)
    {
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, NewState);
				LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    }
    else if(TIMx == TIM7)
    {
//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, NewState);
				LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
    }
    else if(TIMx == TIM8)
    {
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, NewState);
			  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
    }
    else if(TIMx == TIM15)
    {
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, NewState);
				LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

    }
    else if(TIMx == TIM16)
    {
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, NewState);
				LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

    }
    else if(TIMx == TIM17)
    {
//        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, NewState);
				LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);
    }

}

/**
  * @brief  定时中断配置
  * @param  TIMx:定时器地址
  * @param  InterruptTime_us: 中断时间(微秒)
  * @param  function: 定时中断回调函数
  * @retval 无
  */
void TimerSet(TIM_TypeDef* TIMx, uint32_t InterruptTime_us, Timer_CallbackFunction_t function)
{
    Timer_Init(TIMx, InterruptTime_us, function, Timer_PreemptionPriority_Default, Timer_SubPriority_Default);
}

/**
  * @brief  定时中断配置
  * @param  TIMx:定时器地址
  * @param  InterruptTime_us: 中断时间(微秒)
  * @param  function: 定时中断回调函数
  * @param  PreemptionPriority: 抢占优先级
  * @param  SubPriority: 子优先级
  * @retval 无
  */
void Timer_Init(TIM_TypeDef* TIMx, uint32_t InterruptTime_us, Timer_CallbackFunction_t function, uint8_t PreemptionPriority, uint8_t SubPriority)
{
    LL_TIM_InitTypeDef    TIM_TimeBaseStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

    uint32_t arr, psc;
    IRQn_Type TIMx_IRQn;
    TIMER_Type TIMERx;

//    if(!IS_TIM_ALL_PERIPH(TIMx))
//        return;

    if(TIMx == TIM1)
    {
        TIMERx = TIMER1;
        TIMx_IRQn = TIM1_BRK_TIM15_IRQn;
    }

    else if(TIMx == TIM2)
    {
        TIMERx = TIMER2;
        TIMx_IRQn = TIM2_IRQn;
    }
    else if(TIMx == TIM3)
    {
        TIMERx = TIMER3;
        TIMx_IRQn = TIM3_IRQn;
    }
    else if(TIMx == TIM15)
    {
        TIMERx = TIMER15;
        TIMx_IRQn = TIM1_BRK_TIM15_IRQn;
    }
    else if(TIMx == TIM16)
    {
        TIMERx = TIMER16;
        TIMx_IRQn = TIM1_UP_TIM16_IRQn;
    }
    else if(TIMx == TIM17)
    {
        TIMERx = TIMER17;
        TIMx_IRQn = TIM1_TRG_COM_TIM17_IRQn;
    }

    TIM_Function[TIMERx] = function;//Callback Functions

    //Calculate TIM_Period and TIM_Prescaler
    InterruptTime_us *= CYCLES_PER_MICROSECOND;
    if(InterruptTime_us < CYCLES_PER_MICROSECOND * 30)
    {
        arr = 10;
        psc = InterruptTime_us / arr;
    }
    else if(InterruptTime_us < 65535 * 1000)
    {
        arr = InterruptTime_us / 1000;
        psc = InterruptTime_us / arr;
    }
    else
    {
        arr = InterruptTime_us / 20000;
        psc = InterruptTime_us / arr;
    }

    //Enable PeriphClock
    TimerClockCmd(TIMx, ENABLE);

    LL_TIM_DeInit(TIMx);
    TIM_TimeBaseStructure.RepetitionCounter = 0;
    TIM_TimeBaseStructure.Autoreload = arr - 1;         //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.Prescaler = psc - 1;  //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
    TIM_TimeBaseStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;     //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;  //TIM向上计数模式
    LL_TIM_Init(TIMx, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    /**********************************设置中断优先级************************************/
		NVIC_SetPriority(TIMx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),PreemptionPriority, SubPriority));
		NVIC_EnableIRQ(TIMx_IRQn);  
    LL_TIM_ClearFlag_UPDATE(TIMx);
    LL_TIM_EnableIT_UPDATE(TIMx);
//    NVIC_InitStructure.NVIC_IRQChannel = TIMx_IRQn;  //TIM中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;  //先占优先级
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;  //从优先级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

//    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
//    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);  //使能TIM中断
}

/**
  * @brief  更新定时中断时间
  * @param  TIMx:定时器地址
  * @param  InterruptTime_us: 中断时间(微秒)
  * @retval 无
  */
void TimerSet_InterruptTimeUpdate(TIM_TypeDef* TIMx, uint32_t InterruptTime_us)
{
    uint32_t arr, psc;
    //Calculate TIM_Period and TIM_Prescaler
    InterruptTime_us *= CYCLES_PER_MICROSECOND;
    if(InterruptTime_us < CYCLES_PER_MICROSECOND * 30)
    {
        arr = 10;
        psc = InterruptTime_us / arr;
    }
    else if(InterruptTime_us < 65535 * 1000)
    {
        arr = InterruptTime_us / 1000;
        psc = InterruptTime_us / arr;
    }
    else
    {
        arr = InterruptTime_us / 20000;
        psc = InterruptTime_us / arr;
    }

    TIMx->ARR = arr - 1;
    TIMx->PSC = psc - 1;
    TIMx->EGR = TIM_EGR_UG;
}

/*****************************定时中断函数*********************************************/
void TIM1_BRK_TIM15_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) != RESET)
    {
        TIM_Function[TIMER1]();
        LL_TIM_ClearFlag_UPDATE(TIM1);
    }
}

void TIM2_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2) != RESET)
    {
        TIM_Function[TIMER2]();
        LL_TIM_ClearFlag_UPDATE(TIM2);
    }
}

void TIM3_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM3) != RESET)
    {
        TIM_Function[TIMER3]();
        LL_TIM_ClearFlag_UPDATE(TIM3);
    }
}

void TIM15_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM15) != RESET)
    {
        TIM_Function[TIMER15]();
        LL_TIM_ClearFlag_UPDATE(TIM15);
    }
}

void TIM16_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM16) != RESET)
    {
        TIM_Function[TIMER16]();
        LL_TIM_ClearFlag_UPDATE(TIM16);
    }
}

void TIM17_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM17) != RESET)
    {
        TIM_Function[TIMER17]();
        LL_TIM_ClearFlag_UPDATE(TIM17);
    }
}


