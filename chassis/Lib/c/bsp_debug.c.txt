#include "../inc/bsp_debug.h"
#include "../inc/App_Robot.h" // 获取机器人状态
#include "../inc/bsp_motor.h" // 获取电机状态

extern UART_HandleTypeDef DEBUG_UART_HANDLE;
extern Robot_TypeDef Robot; // 引用 Robot 全局实例

// --- 1. Printf 重定向 (支持 GCC/STM32CubeIDE 和 Keil) ---

#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}
#else
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
#endif

// --- 2. 调试逻辑 ---

void Debug_Init(void)
{
    printf("\r\n[DEBUG] System Start. Waiting for data...\r\n");
    printf("Format: Time, Target_LF, Actual_LF, PWM_LF\r\n"); // 打印表头方便绘图软件识别
}

void Debug_Task(void)
{
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print < DEBUG_RATE_MS) return;
    last_print = HAL_GetTick();

    // --- 打印遥测数据 (CSV格式) ---
    // 适合 SerialPlot / Vofa+ :
    // 通道1: 目标转速
    // 通道2: 实际转速
    // 通道3: PWM输出 (缩放一下以便在同一张图显示)

    // 以左前轮 (LF) 为例进行 PID 调试
    // 注意: App_Robot.c 没保存 TargetRPM 到结构体，这里我们计算一下或者修改 App_Robot
    // 简单起见，我们打印 PID 的 error (误差) 和 out (输出)

    float actual_rpm = bsp_motor_get_rpm(MOTOR_CH_FL);
    float pid_out    = Robot.Pid_LF.out;
    float rx_cmd     = Robot.Remote.KeyMask; // 查看收到的指令掩码

    // 格式: [Tick], [Actual_RPM], [PID_Out], [CMD_Mask]
    // 使用 %.2f 打印浮点
    printf("%.2f,%.2f,%.2f,%d\r\n", 
           (float)HAL_GetTick()/1000.0f, // X轴时间
           actual_rpm,                   // 曲线1：实际速度
           pid_out / 10.0f,              // 曲线2：PWM (缩小10倍方便同屏查看)
           rx_cmd                        // 曲线3：收到的指令
    );
}