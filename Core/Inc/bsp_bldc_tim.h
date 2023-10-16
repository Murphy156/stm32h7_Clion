#ifndef CPROJECT_BSP_BLDC_TIM_H
#define CPROJECT_BSP_BLDC_TIM_H

#include "stm32h7xx.h"
#include "bsp_bldc_control.h"

/** 电机控制定时器 */
#define BLDC_TIM							TIM1
#define BLDC_TIM_CLK_ENABLE()				__TIM1_CLK_ENABLE()
extern TIM_HandleTypeDef	                htimx_bldcm;

/** 累计 TIM_Period个后产生一个更新或者中断
	当定时器从0计数到5599，即为5600次，为一个定时周期 */
#define PWM_PERIOD_COUNT     (5600)

#define PWM_MAX_PERIOD_COUNT    (PWM_PERIOD_COUNT - 100)

/** 定时器时钟源TIMxCLK = 2 * PCLK1
					PCLK1 = HCLK / 2
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)/PWM_PERIOD_COUNT = 14.28KHz*/
#define PWM_PRESCALER_COUNT     (3)

/** TIM1通道1输出引脚*/
#define BLDC_OCPWM1_PIN						GPIO_PIN_8
#define BLDC_OCPWM1_GPIO_PORT				GPIOA
#define BLDC_OCPWM1_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#define BLDC_OCPWM1_AF						GPIO_AF1_TIM1

/** TIM1通道2输出引脚*/
#define BLDC_OCPWM2_PIN						GPIO_PIN_9
#define BLDC_OCPWM2_GPIO_PORT				GPIOA
#define BLDC_OCPWM2_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#define BLDC_OCPWM2_AF						GPIO_AF1_TIM1

/** TIM1通道3输出引脚*/
#define BLDC_OCPWM3_PIN						GPIO_PIN_10
#define BLDC_OCPWM3_GPIO_PORT				GPIOA
#define BLDC_OCPWM3_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#define BLDC_OCPWM3_AF						GPIO_AF1_TIM1

/** TIM1通道1互补输出引脚*/
#define BLDC_OCNPWM1_PIN					GPIO_PIN_13
#define BLDC_OCNPWM1_GPIO_PORT				GPIOB
#define BLDC_OCNPWM1_GPIO_CLK_ENABLE()	    __GPIOB_CLK_ENABLE()
#define BLDC_OCNPWM1_AF						GPIO_AF1_TIM1

/** TIM1通道1互补输出引脚*/
#define BLDC_OCNPWM2_PIN					GPIO_PIN_14
#define BLDC_OCNPWM2_GPIO_PORT				GPIOB
#define BLDC_OCNPWM2_GPIO_CLK_ENABLE()	    __GPIOB_CLK_ENABLE()
#define BLDC_OCNPWM2_AF						GPIO_AF1_TIM1

/** TIM1通道1互补输出引脚*/
#define BLDC_OCNPWM3_PIN					GPIO_PIN_15
#define BLDC_OCNPWM3_GPIO_PORT				GPIOB
#define BLDC_OCNPWM3_GPIO_CLK_ENABLE()	    __GPIOB_CLK_ENABLE()
#define BLDC_OCNPWM3_AF						GPIO_AF1_TIM1

#define TIM_COM_TS_ITRx                     TIM_TS_ITR2    // 内部触发配置(TIM1->ITR2->TIM3)

/** 霍尔传感器定时器 */
#define HALL_TIM							TIM3
#define HALL_TIM_CLK_ENABLE()				__TIM3_CLK_ENABLE()

extern TIM_HandleTypeDef 					htimx_hall;

/** 累计 TIM_Period个后产生一个更新或者中断
	当定时器从0计数到9999，即为10000次，为一个定时周期 */
#define HALL_PERIOD_COUNT     (10000)

/** 定时器时钟源TIMxCLK = 2 * PCLK1
					PCLK1 = HCLK / 2
					=> TIMxCLK=HCLK/2=SystemCoreClock/2=240MHz
	 设定定时器频率为 = TIMxCLK / (PWM_PRESCALER_COUNT + 1) / PWM_PERIOD_COUNT = 10Hz
   周期 T = 100ms */
#define HALL_PRESCALER_COUNT     (2400)

/** TIM3 通道 1 引脚 */
#define HALL_INPUTU_PIN						GPIO_PIN_6
#define HALL_INPUTU_GPIO_PORT				GPIOC
#define HALL_INPUTU_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#define HALL_INPUTU_AF						GPIO_AF2_TIM3

/** TIM3 通道 2 引脚 */
#define HALL_INPUTV_PIN						GPIO_PIN_7
#define HALL_INPUTV_GPIO_PORT				GPIOC
#define HALL_INPUTV_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#define HALL_INPUTV_AF						GPIO_AF2_TIM3

/** TIM3 通道 3 引脚 */
#define HALL_INPUTW_PIN						GPIO_PIN_8
#define HALL_INPUTW_GPIO_PORT				GPIOC
#define HALL_INPUTW_GPIO_CLK_ENABLE()		__GPIOC_CLK_ENABLE()
#define HALL_INPUTW_AF						GPIO_AF2_TIM3

#define HALL_TIM_IRQn                       TIM3_IRQn
#define HALL_TIM_IRQHandler                 TIM3_IRQHandler

extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void TIMx_Configuration(void);
void stop_pwm_output(void);
void start_pwm_output(void);
void set_pwm_pulse(uint16_t pulse);

void hall_enable(void);
void hall_disable(void);
void hall_tim_config(void);

#endif //CPROJECT_BSP_BLDC_TIM_H
