#ifndef CPROJECT_BSP_BLDC_CONTROL_H
#define CPROJECT_BSP_BLDC_CONTROL_H

#include "stm32h7xx.h"
#include "main.h"
#include "bsp_bldc_tim.h"

/** 引脚定义 */
/*******************************************************/
/**  连接驱动板的 SD 脚 */
#define BLDC_SHUTDOWN_PIN                  GPIO_PIN_12
#define BLDC_SHUTDOWN_GPIO_PORT            GPIOG
#define BLDC_SHUTDOWN_GPIO_CLK_ENABLE()    __GPIOG_CLK_ENABLE()

/** 电机 SD or EN 使能脚 */
#define BLDC_ENABLE_SD()                     HAL_GPIO_WritePin(BLDC_SHUTDOWN_GPIO_PORT, BLDC_SHUTDOWN_PIN, GPIO_PIN_SET)      /** 高电平打开-高电平使能 */
#define BLDC_DISABLE_SD()                    HAL_GPIO_WritePin(BLDC_SHUTDOWN_GPIO_PORT, BLDC_SHUTDOWN_PIN, GPIO_PIN_RESET)    /** 低电平关断-低电平禁用 */

/** 电机方向控制枚举 */
typedef enum
{
    MOTOR_FWD = 0,
    MOTOR_REV,
}motor_dir_t;

typedef struct
{
    motor_dir_t direction;    /** 电机方向 */
    uint16_t dutyfactor;      /** PWM 输出占空比 */
    uint8_t is_enable;        /** 使能电机 */
    uint32_t lock_timeout;    /** 电机堵转计时 */
}bldcm_data_t;

void bldcm_init(void);

void set_BLDC_speed(uint16_t v);
void set_BLDC_direction(motor_dir_t dir);
motor_dir_t get_BLDC_direction(void);
void set_BLDC_enable(void);
void set_BLDC_disable(void);

#endif //CPROJECT_BSP_BLDC_CONTROL_H
