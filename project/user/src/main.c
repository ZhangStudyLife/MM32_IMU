/*********************************************************************************************************************
* MM32F327X-G8P IMU姿态解算测试程序 - 老王暴躁技术流
*
* 功能说明:
*   1. 使用定时器中断(200Hz)周期性触发IMU数据更新
*   2. 只使用Quaternion算法进行姿态解算
*   3. 通过串口发送VOFA+ JustFloat协议数据：9个float依次为
*      gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, pitch, roll, yaw
*
* 作者: 老王暴躁技术流
* 日期: 2025-10-28
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "../code/IMU/IMU.h"
#include <string.h>

/********************************************************************************************************************
 * 配置参数
 ********************************************************************************************************************/

#define IMU_SAMPLE_RATE         500     // IMU采样率：500Hz
#define VOFA_SEND_RATE          100      // 数据发送率：500Hz

/********************************************************************************************************************
 * 全局变量
 ********************************************************************************************************************/

volatile uint8_t g_imu_update_flag = 0;  // IMU更新标志

/********************************************************************************************************************
 * VOFA+ JustFloat协议发送函数
 ********************************************************************************************************************/

/**
 * @brief 发送VOFA+ JustFloat协议数据
 * @param data float数组指针
 * @param count 数据个数
 */
void vofa_send_data(float *data, uint8_t count)
{
    if (data == NULL || count == 0) return;

    // 准备发送缓冲区（float数据 + 帧尾）
    uint32_t data_bytes = count * sizeof(float);
    uint8_t send_buffer[64];  // 最大支持15个float（15*4=60字节 + 4字节帧尾）

    if (data_bytes + 4 > sizeof(send_buffer)) return;  // 防止缓冲区溢出

    // 复制float数据（小端格式）
    memcpy(send_buffer, data, data_bytes);

    // 添加帧尾：0x00 0x00 0x80 0x7F
    send_buffer[data_bytes + 0] = 0x00;
    send_buffer[data_bytes + 1] = 0x00;
    send_buffer[data_bytes + 2] = 0x80;
    send_buffer[data_bytes + 3] = 0x7F;

    // 一次性发送整个缓冲区（艹！快多了！）
    uart_write_buffer(UART_1, send_buffer, data_bytes + 4);
}

/**
 * @brief 发送IMU数据
 * 格式：gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, pitch, roll, yaw (9个float)
 */
void vofa_send_imu_data(void)
{
    float vofa_data[9];  // 9个float数据

    // 获取原始传感器数据
    const imu_raw_data_t *raw = imu_get_raw_data();
    if (raw == NULL) return;

    // 前6个：角速度（rad/s）和加速度（g）
    vofa_data[0] = raw->gyro_x;
    vofa_data[1] = raw->gyro_y;
    vofa_data[2] = raw->gyro_z;
    vofa_data[3] = raw->acc_x;
    vofa_data[4] = raw->acc_y;
    vofa_data[5] = raw->acc_z;

    // 后3个：姿态角度（度）
    imu_attitude_t attitude;
    if (0 == imu_get_attitude(IMU_ALGO_MAHONY, &attitude))
    {
        vofa_data[6] = attitude.pitch;
        vofa_data[7] = attitude.roll;
        vofa_data[8] = attitude.yaw;
    }
    else
    {
        vofa_data[6] = 0.0f;
        vofa_data[7] = 0.0f;
        vofa_data[8] = 0.0f;
    }

    // 发送数据
    vofa_send_data(vofa_data, 9);
}

/********************************************************************************************************************
 * 主程序
 ********************************************************************************************************************/

int main(void)
{
    // 系统初始化
    clock_init(SYSTEM_CLOCK_120M);
    debug_init();
    system_delay_ms(100);

    // IMU系统初始化
    imu_system_init(IMU_SAMPLE_RATE);

    // ✅ 加速度计静态校准（设备必须水平静止放置！）
    // 艹！这一步超级重要！消除加速度计偏移
    // 你的ay=0.139就是因为没校准！
    //imu_calibrate_acc_start();

    // ✅ 陀螺仪静态校准（设备必须完全静止放置！）
    // 艹！这一步超级重要！消除初始零漂±3-5°/s
    imu_calibrate_gyro_start();

    // 等待IMU数据稳定
    for (uint16_t i = 0; i < 100; i++)
    {
        imu_update();
        system_delay_ms(5);
    }

    // 使能Mahony算法
    imu_algorithm_enable(IMU_ALGO_MAHONY);

    // ✅ 禁用磁力计（六轴模式）
    extern void mahony_set_mag_enable(bool enable);
    mahony_set_mag_enable(false);

    // ✅ 禁用Ki积分（避免六轴模式下的Yaw漂移）
    extern void mahony_set_Ki(float Ki);
    mahony_set_Ki(0.0f);

    // 初始化定时器中断（500Hz）
    uint32_t timer_period_ms = 1000 / IMU_SAMPLE_RATE;
    pit_ms_init(TIM6_PIT, timer_period_ms);

    // 计算VOFA+发送分频系数
    uint32_t vofa_divider = IMU_SAMPLE_RATE / VOFA_SEND_RATE;
    uint32_t vofa_send_counter = 0;

    // 主循环
    while(1)
    {
        // 检查IMU更新标志
        if (g_imu_update_flag)
        {
            g_imu_update_flag = 0;

            vofa_send_counter++;

            // 按照设定的频率发送数据
            if (vofa_send_counter >= vofa_divider)
            {
                vofa_send_counter = 0;
                vofa_send_imu_data();
            }
        }
    }

    return 0;
}
