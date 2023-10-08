/**
  ******************************************************************************
  * @file    bsp_bldc_control.c
  * @author  Leo
  * @version V1.0
  * @date    2023-10-08
  * @brief
  ******************************************************************************
  */

#include "bsp_bldc_control.h"
#include "bsp_led.h"

/** 私有变量 */
static bldcm_data_t bldcm_data;

/** 局部函数 */
static void sd_gpio_config(void);

/**
  * @brief  电机初始化
  * @param  无
  * @retval 无
  */
void bldcm_init(void)
{
    TIMx_Configuration();    /** 电机控制定时器，引脚初始化 */
    hall_tim_config();       /** 霍尔传感器初始化 */
    sd_gpio_config();
}

static void sd_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /** 定时器通道功能引脚端口时钟使能 */
    BLDC_SHUTDOWN_GPIO_CLK_ENABLE();

    /** 引脚IO初始化 */
    /** 设置输出类型*/
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    /** 设置引脚速率 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /** 选择要控制的GPIO引脚*/
    GPIO_InitStruct.Pin = BLDC_SHUTDOWN_PIN;

    /** 调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
    HAL_GPIO_Init(BLDC_SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  设置电机速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_BLDC_speed(uint16_t v)
{
    bldcm_data.dutyfactor = v;

    set_pwm_pulse(v);     /** 设置速度 */
}

/**
  * @brief  设置电机方向
  * @param  无
  * @retval 无
  */
void set_BLDC_direction(motor_dir_t dir)
{
    bldcm_data.direction = dir;
}

/**
  * @brief  获取电机当前方向
  * @param  无
  * @retval 无
  */
motor_dir_t get_BLDC_direction(void)
{
    return bldcm_data.direction;
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_BLDC_enable(void)
{
    BLDC_ENABLE_SD();
    /** 延迟50ms，H743开Cache后速度会过快，未满足SD引脚时序。加入操作系统时需格外注意这个延时函数是否生效或阻塞这里 */
    HAL_Delay(50);
    hall_enable();
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void set_BLDC_disable(void)
{
    /** 禁用霍尔传感器接口 */
    hall_disable();

    /** 停止 PWM 输出 */
    stop_pwm_output();

    /** 关闭 MOS 管 */
    BLDC_DISABLE_SD();
}
