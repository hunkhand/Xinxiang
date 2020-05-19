#ifndef __FILTER_H__
#define __FILTER_H__


#define TIM3_COUNTER_CLOCK        24000000                 //计数时钟(24M次/秒)
                                                           //预分频值
#define TIM3_PRESCALER_VALUE      (SystemCoreClock/TIM3_COUNTER_CLOCK - 1)

void TIM3_PWM_INIT(void);
void TIM3_CH2_PWM(uint32_t arr);
//void TIM3_CH2_PWM(uint32_t Freq, uint16_t Dutycycle);  //移植于：F:\我的资料\F0第一阶段软件工程\STM32F0xx_TIM输出PWM配置详细过程

void App_Filter_Task (void *p_arg);

#endif
