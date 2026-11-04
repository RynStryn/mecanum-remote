/**
 * bsp_motor.c
 * 增加了方向引脚控制逻辑
 */

#include "../inc/bsp_motor.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h> // for abs()

/* === 配置参数 === */
#define ENCODER_PPR     11.0f    // 基础脉冲数
#define GEAR_RATIO      46.8f    // (假设) 减速比，如果是直驱则为1.0
// 实际单圈总脉冲 = 11 * 4(倍频) * 减速比。
// 如果您的 ENCODER_PPR 已经包含了减速比和4倍频，请保持原样。
// 这里的 11.0f 看起来像是电机裸机的线数，通常都需要乘减速比。
// 下面计算公式中使用： diff / (ENCODER_PPR * 4 * GEAR_RATIO)
// 为了保持与您原代码逻辑一致，这里暂时沿用您原本的逻辑，但建议确认 PPR 含义
#define TOTAL_PPR   (ENCODER_PPR * 2.0f * GEAR_RATIO) // 这里假设减速比为1，请按需修改

/* === 硬件引脚配置 === */

#define MOT_FL_DIR_GPIO_PORT    GPIOB
#define MOT_FL_DIR_PIN          GPIO_PIN_12
#define MOT_FL_DIR_GPIO_PORT2    GPIOB
#define MOT_FL_DIR_PIN2          GPIO_PIN_13

#define MOT_FR_DIR_GPIO_PORT    GPIOB
#define MOT_FR_DIR_PIN          GPIO_PIN_14
#define MOT_FR_DIR_GPIO_PORT2    GPIOB
#define MOT_FR_DIR_PIN2          GPIO_PIN_15


#define MOT_RL_DIR_GPIO_PORT    GPIOB
#define MOT_RL_DIR_PIN          GPIO_PIN_3
#define MOT_RL_DIR_GPIO_PORT2    GPIOB
#define MOT_RL_DIR_PIN2          GPIO_PIN_4

#define MOT_RR_DIR_GPIO_PORT    GPIOA
#define MOT_RR_DIR_PIN          GPIO_PIN_10
#define MOT_RR_DIR_GPIO_PORT2    GPIOA
#define MOT_RR_DIR_PIN2          GPIO_PIN_11
/* === 外部句柄 === */
extern TIM_HandleTypeDef htim1; // LF Encoder
extern TIM_HandleTypeDef htim2; // RL Encoder
extern TIM_HandleTypeDef htim3; // FR Encoder
extern TIM_HandleTypeDef htim4; // RR Encoder
extern TIM_HandleTypeDef htim8; // PWM Output

/* === 内部变量 === */
static TIM_HandleTypeDef *enc_tim[MOTOR_CH_COUNT];
static int16_t prev_cnt[MOTOR_CH_COUNT];
static float rpm[MOTOR_CH_COUNT];

/* --------------------------------------------------------------------------
 * 初始化
 * -------------------------------------------------------------------------- */
void bsp_motor_init(void)
{
    // 1. 映射定时器
    enc_tim[MOTOR_CH_FL] = &htim1;
    enc_tim[MOTOR_CH_FR] = &htim3;
    enc_tim[MOTOR_CH_RL] = &htim2;
    enc_tim[MOTOR_CH_RR] = &htim4;

    // 2. 初始化状态
    for (int i = 0; i < MOTOR_CH_COUNT; i++) {
        prev_cnt[i] = (int16_t)__HAL_TIM_GET_COUNTER(enc_tim[i]);
        rpm[i] = 0.0f;
    }

    // 3. 启动编码器
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // 4. 启动 PWM
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    // 5. 初始化方向引脚 (防止电机上电乱转)
    // 确保这些引脚在 MX_GPIO_Init 中已经初始化为 OUTPUT
}

/* --------------------------------------------------------------------------
 * 内部辅助：设置方向 GPIO
 * -------------------------------------------------------------------------- */
static void set_motor_dir(e_motor_ch ch, int8_t dir)
{
    // 1. 确定第一个引脚的状态
    // dir: 1 = 正转, -1 = 反转 (若方向反了，交换这里的 SET/RESET)
    GPIO_PinState pin_state = (dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // 2. 确定第二个引脚的状态 (取反)
    // 显式取反，比 !pin_state 更严谨
    GPIO_PinState inv_pin_state = (pin_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    switch (ch) {
        case MOTOR_CH_FL:
            HAL_GPIO_WritePin(MOT_FL_DIR_GPIO_PORT,  MOT_FL_DIR_PIN,  pin_state);
            HAL_GPIO_WritePin(MOT_FL_DIR_GPIO_PORT2, MOT_FL_DIR_PIN2, inv_pin_state);
            break;

        case MOTOR_CH_FR:
            HAL_GPIO_WritePin(MOT_FR_DIR_GPIO_PORT,  MOT_FR_DIR_PIN,  pin_state);
            HAL_GPIO_WritePin(MOT_FR_DIR_GPIO_PORT2, MOT_FR_DIR_PIN2, inv_pin_state);
            break;

        case MOTOR_CH_RL:
            HAL_GPIO_WritePin(MOT_RL_DIR_GPIO_PORT,  MOT_RL_DIR_PIN,  pin_state);
            HAL_GPIO_WritePin(MOT_RL_DIR_GPIO_PORT2, MOT_RL_DIR_PIN2, inv_pin_state);
            break;

        case MOTOR_CH_RR:
            HAL_GPIO_WritePin(MOT_RR_DIR_GPIO_PORT,  MOT_RR_DIR_PIN,  pin_state);
            HAL_GPIO_WritePin(MOT_RR_DIR_GPIO_PORT2, MOT_RR_DIR_PIN2, inv_pin_state);
            break;

        default:
            break;
    }
}
/* --------------------------------------------------------------------------
 * 执行：设置 PWM 和 方向
 * -------------------------------------------------------------------------- */
void bsp_motor_set_pwm(e_motor_ch ch, int32_t pwm)
{
    // 1. 限幅
    if (pwm >  MAX_PWM) pwm =  MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;

    // 2. 判断方向并设置 GPIO
    if (pwm >= 0) {
        set_motor_dir(ch, 1);
    } else {
        set_motor_dir(ch, -1);
    }

    // 3. 取绝对值设置占空比
    uint32_t duty = (uint32_t)abs(pwm);

    // 4. 写入硬件
    switch (ch) {
        case MOTOR_CH_FL: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty); break;
        case MOTOR_CH_RL: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty); break;
        case MOTOR_CH_FR: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty); break;
        case MOTOR_CH_RR: __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, duty); break;
        default: break;
    }
}

/* --------------------------------------------------------------------------
 * 感知：读取 RPM
 * -------------------------------------------------------------------------- */
void bsp_motor_periodic_update(float dt_s)
{


    for (int ch = 0; ch < MOTOR_CH_COUNT; ch++) {
        TIM_HandleTypeDef *htim = enc_tim[ch];

        // 读取当前计数器
        int16_t cur  = (int16_t)__HAL_TIM_GET_COUNTER(htim);

        // 计算差值 (利用 int16_t 溢出特性自动处理 0-65535 回绕)
     int16_t diff = cur - prev_cnt[ch];
//!db
        diff=-diff;
        prev_cnt[ch] = cur;

        // 计算转速 RPM
        // diff 是 dt 时间内的脉冲数
        // TOTAL_PPR 是转一圈的总脉冲数 (线数 * 倍频 * 减速比)
        // 您的原代码是 ENCODER_PPR (11.0)，这可能偏小，请确认是否需要乘减速比
        float rev = (float)diff / TOTAL_PPR; // 如果 ENCODER_PPR 没乘4和减速比，这里计算的 RPM 会非常大

        rpm[ch] = (rev / dt_s) * 60.0f;
    }
}

float bsp_motor_get_rpm(e_motor_ch ch)
{
    if (ch >= MOTOR_CH_COUNT) return 0.0f;
    return rpm[ch];
}