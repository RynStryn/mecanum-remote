/* --- START OF FILE App_Remote.c (SPL Version) --- */

#include "stm32f10x.h"
#include "protocol.h"
#include <string.h>

/* === Configuration === */
#define SEND_PERIOD_CHASSIS_MS   80U
#define SEND_PERIOD_ARM_MS       100U

#define ADC_SAMPLE_COUNT         200
#define ADC_DEADZONE             20

#define SERVO_MIN_CCR            50
#define SERVO_MAX_CCR            250

#define GRIPPER_CHAMPS_TIGHTLY   50
#define GRIPPER_RELEASED         180

/* === GPIO Definitions === */
#define BTN_W_PORT     GPIOA
#define BTN_W_PIN      GPIO_Pin_7

#define BTN_A_PORT     GPIOA
#define BTN_A_PIN      GPIO_Pin_6

#define BTN_S_PORT     GPIOB
#define BTN_S_PIN      GPIO_Pin_0

#define BTN_D_PORT     GPIOB
#define BTN_D_PIN      GPIO_Pin_1

#define BTN_SHIFT_PORT GPIOB
#define BTN_SHIFT_PIN  GPIO_Pin_10

#define BTN_CTRL_PORT  GPIOB
#define BTN_CTRL_PIN   GPIO_Pin_11
/* === 【新增】GPIO Definitions (请根据实际电路修改引脚!) === */
// 假设 Q键 接在 PB12, E键 接在 PB13, 急停 接在 PB14
#define BTN_Q_PORT     GPIOB
#define BTN_Q_PIN      GPIO_Pin_8

#define BTN_E_PORT     GPIOB
#define BTN_E_PIN      GPIO_Pin_9

#define BTN_STOP_PORT  GPIOB
#define BTN_STOP_PIN   GPIO_Pin_7
/* === External === */
extern uint32_t GetTick(void);
extern uint16_t AD_GetValue(uint8_t ch);

/* === Internal State === */
static uint32_t last_chassis_tick = 0;
static uint32_t last_arm_tick     = 0;

uint8_t arm_mode_flag = 0;

/* === Arm Data === */
static uint8_t s_arm_joints[6] = {150,150,150,150,150,150};

/* === DMA Packet Buffer === */
 Packet_t tx_chassis_buf;
static Packet_t tx_arm_buf;
/* ========================================================= */
/*                    Button Read                            */
/* ========================================================= */

static uint8_t Remote_ReadButtonMask(void)
{
    Remote_State_t st;
    st.all = 0;

    st.bits.W     = (GPIO_ReadInputDataBit(BTN_W_PORT, BTN_W_PIN) == Bit_SET) ? 0 : 1;
    st.bits.A     = (GPIO_ReadInputDataBit(BTN_A_PORT, BTN_A_PIN) == Bit_SET) ? 0 : 1;
    st.bits.S     = (GPIO_ReadInputDataBit(BTN_S_PORT, BTN_S_PIN) == Bit_SET) ? 0 : 1;
    st.bits.D     = (GPIO_ReadInputDataBit(BTN_D_PORT, BTN_D_PIN) == Bit_SET) ? 0 : 1;
    st.bits.Shift = (GPIO_ReadInputDataBit(BTN_SHIFT_PORT, BTN_SHIFT_PIN) == Bit_SET) ? 0 : 1;
    // --- 【新增】读取 Q, E, STOP ---
    st.bits.Q     = (GPIO_ReadInputDataBit(BTN_Q_PORT, BTN_Q_PIN) == Bit_SET) ? 0 : 1;
    st.bits.E     = (GPIO_ReadInputDataBit(BTN_E_PORT, BTN_E_PIN) == Bit_SET) ? 0 : 1;
    st.bits.STOP  = (GPIO_ReadInputDataBit(BTN_STOP_PORT, BTN_STOP_PIN) == Bit_SET) ? 0 : 1;
    return st.all;
}

/* ========================================================= */
/*                    DMA Send                               */
/* ========================================================= */

static void Remote_DMA_Transmit(uint8_t *data, uint16_t len)
{
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA1_Channel4->CMAR  = (uint32_t)data;
    DMA1_Channel4->CNDTR = len;
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/* ========================================================= */
/*                    Arm Interface                          */
/* ========================================================= */

void Remote_UpdateArmData(uint8_t joint, uint8_t value)
{
    if (joint < 6)
    {
        if (value < SERVO_MIN_CCR) value = SERVO_MIN_CCR;
        if (value > SERVO_MAX_CCR) value = SERVO_MAX_CCR;
        s_arm_joints[joint] = value;
    }
}
uint8_t map_and_clamp(float adc_val)
{
    // 1. 先用 float 计算，防止溢出
    float result = (adc_val / 2050.0f) * 200.0f + 50.0f; // 还是用你原来的参数也行
    
    // 2. 判断是否越界
    if (result > 250.0f) result = 250.0f; // 就算算出 449，也强行变成 250
    if (result < 50.0f)  result = 50.0f;
    
    // 3. 最后再转 uint8
    return (uint8_t)result;
}
/* ========================================================= */
/*                    ADC Filter                             */
/* ========================================================= */

static float ADC_Filter(uint8_t ch)
{
    float sum = 0;
    float max = 0;
    float min = 4096;
    float val;

    for (uint16_t i = 0; i < ADC_SAMPLE_COUNT; i++)
    {
        val = AD_GetValue(ch);
        sum += val;
        if (val > max) max = val;
        if (val < min) min = val;
    }

    sum -= max;
    sum -= min;

    return sum / (ADC_SAMPLE_COUNT - 2);
}

static float Apply_Deadzone(float now, float last)
{
    if ((now > last - ADC_DEADZONE) &&
        (now < last + ADC_DEADZONE))
    {
        return last;
    }
    return now;
}

/* ========================================================= */
/*                    Arm Task                               */
/* ========================================================= */

void Arm_Task(void)
{

    static float last_adc[5] = {0};



    /* --- Mode Switch --- */
    arm_mode_flag = (GPIO_ReadInputDataBit(BTN_CTRL_PORT, BTN_CTRL_PIN) == Bit_SET);

    /* --- ADC Read + Filter --- */
    float adc[5];
    adc[0] = ADC_Filter(ADC_Channel_0);
    adc[1] = ADC_Filter(ADC_Channel_1);
    adc[2] = ADC_Filter(ADC_Channel_3);
    adc[3] = ADC_Filter(ADC_Channel_4);
    adc[4] = ADC_Filter(ADC_Channel_5);

    /* --- Deadzone --- */
    for (int i = 0; i < 5; i++)
    {
        adc[i] = Apply_Deadzone(adc[i], last_adc[i]);
        last_adc[i] = adc[i];
    }

    /* --- ADC → Servo (示例映射，你可自行修改) --- */
    s_arm_joints[0] = (uint8_t)(adc[0] / 2050.0f * 200 + 50);
    s_arm_joints[1] = (uint8_t)(adc[1] / 2050.0f * 200 + 50);
    s_arm_joints[2] = (uint8_t)(adc[2] / 2050.0f * 200 + 50);
    s_arm_joints[3] = (uint8_t)(adc[3] / 2050.0f * 200 + 50);
    s_arm_joints[4] = (uint8_t)(adc[4] / 2050.0f * 200 + 50);

    s_arm_joints[5] = arm_mode_flag ?
                      GRIPPER_CHAMPS_TIGHTLY :
                      GRIPPER_RELEASED;
}

/* ========================================================= */
/*                    Remote Task                            */
/* ========================================================= */
void Remote_Init(void)
{
    last_chassis_tick = GetTick();
    last_arm_tick = GetTick();
    
    // 注意：需要在 main.c 中完成 GPIO, USART1 和 DMA1_Channel4 的硬件初始化
}
void Remote_Task(void)
{
    uint32_t now = GetTick();

    /* === 必须调用：更新机械臂数据 === */
    
    /* === Chassis Send === */
    if (now - last_chassis_tick >= SEND_PERIOD_CHASSIS_MS)
    {
       
        if (DMA_GetCurrDataCounter(DMA1_Channel4) == 0)
        {
             last_chassis_tick = now;
            uint8_t mask = Remote_ReadButtonMask();
            pack_chassis(&tx_chassis_buf, mask);
            Remote_DMA_Transmit((uint8_t*)&tx_chassis_buf, PKT_LENGTH);
        }
    }

    /* === Arm Send === */
    if (now - last_arm_tick >= SEND_PERIOD_ARM_MS)
    {
        
        if (DMA_GetCurrDataCounter(DMA1_Channel4) == 0)
        {
            Arm_Task();

            last_arm_tick = now;
            pack_arm(&tx_arm_buf, s_arm_joints);
            Remote_DMA_Transmit((uint8_t*)&tx_arm_buf, PKT_LENGTH);
            
        }
    }
}

/* --- END OF FILE --- */
