/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 纯四元数积分算法实现
*
* 文件名称          quaternion.c
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-27
*
********************************************************************************************************************/

#include "quaternion.h"
#include "zf_common_debug.h"
#include <string.h>

// 算法私有数据（静态分配）
static quaternion_algo_data_t g_quaternion_algo_data;
static imu_algorithm_instance_t *g_quaternion_instance = NULL;

/**
 * @brief 初始化纯四元数积分算法
 */
void quaternion_init(void *algo_data)
{
    if (g_quaternion_instance == NULL)
    {
        return;
    }

    // 初始化四元数为单位四元数 [1, 0, 0, 0]
    imu_quat_identity(&g_quaternion_instance->quaternion);

    // 初始化姿态为0
    g_quaternion_instance->attitude.pitch = 0.0f;
    g_quaternion_instance->attitude.roll = 0.0f;
    g_quaternion_instance->attitude.yaw = 0.0f;
    g_quaternion_instance->attitude.valid = 1;

    memset(&g_quaternion_algo_data, 0, sizeof(quaternion_algo_data_t));
}

/**
 * @brief 更新纯四元数积分算法（核心函数）
 */
void quaternion_update(void *algo_data, const imu_raw_data_t *raw_data, float dt)
{
    if (g_quaternion_instance == NULL || raw_data == NULL) return;
    if (!raw_data->gyro_valid) return;

    // 获取陀螺仪数据（rad/s）
    float gyro_x = raw_data->gyro_x;
    float gyro_y = raw_data->gyro_y;
    float gyro_z = raw_data->gyro_z;

    // 【核心算法】使用陀螺仪角速度更新四元数
    // 这是整个算法唯一的核心代码！
    // 公式: q(t+dt) = q(t) + 0.5 * q(t) ⊗ [0, ωx, ωy, ωz] * dt
    imu_quat_update_gyro(&g_quaternion_instance->quaternion, gyro_x, gyro_y, gyro_z, dt);

    // 艹！归一化超级重要！
    // 为什么重要呢
    // 因为四元数经过多次积分后会累积误差，导致模值偏离1
    // 如果不归一化，模值偏离1会导致姿态解算
    imu_quat_normalize(&g_quaternion_instance->quaternion);

    // 将四元数转换为欧拉角
    float pitch_rad, roll_rad, yaw_rad;
    imu_quat_to_euler(&g_quaternion_instance->quaternion, &pitch_rad, &roll_rad, &yaw_rad);

    // 转换为角度制
    g_quaternion_instance->attitude.pitch = pitch_rad * IMU_RAD_TO_DEG;
    g_quaternion_instance->attitude.roll = roll_rad * IMU_RAD_TO_DEG;
    g_quaternion_instance->attitude.yaw = yaw_rad * IMU_RAD_TO_DEG;

    // 归一化到 [-180, 180]
    g_quaternion_instance->attitude.pitch = imu_normalize_angle_deg(g_quaternion_instance->attitude.pitch);
    g_quaternion_instance->attitude.roll = imu_normalize_angle_deg(g_quaternion_instance->attitude.roll);
    g_quaternion_instance->attitude.yaw = imu_normalize_angle_deg(g_quaternion_instance->attitude.yaw);

    g_quaternion_instance->attitude.timestamp = raw_data->timestamp;
    g_quaternion_instance->attitude.valid = 1;
}

/**
 * @brief 获取纯四元数积分算法的姿态结果
 */
void quaternion_get_attitude(void *algo_data, imu_attitude_t *attitude)
{
    if (g_quaternion_instance == NULL || attitude == NULL) return;
    memcpy(attitude, &g_quaternion_instance->attitude, sizeof(imu_attitude_t));
}

/**
 * @brief 重置纯四元数积分算法
 */
void quaternion_reset(void *algo_data)
{
    quaternion_init(algo_data);
}

/**
 * @brief 注册纯四元数积分算法到IMU系统
 */
int quaternion_register(imu_algorithm_instance_t *instance)
{
    if (instance == NULL)
    {
        return -1;
    }

    g_quaternion_instance = instance;
    instance->algo_specific_data = &g_quaternion_algo_data;
    instance->init = quaternion_init;
    instance->update = quaternion_update;
    instance->get_attitude = quaternion_get_attitude;
    instance->reset = quaternion_reset;

    return 0;
}

/********************************************************************************************************************
 * 老王的实现笔记:
 *
 * 艹！看到没有？整个算法的核心就一行代码：
 *   imu_quat_update_gyro(&q, gyro_x, gyro_y, gyro_z, dt);
 *
 * 这就是纯四元数积分！简单粗暴！
 *
 * 【性能分析】
 * - 四元数乘法：~10条指令
 * - 四元数归一化：1次sqrt + 4次除法 ≈ 30条指令
 * - 欧拉角转换：3次atan2 ≈ 100条指令
 * - 总计：~150条指令 @ 120MHz ≈ 1.25us
 *
 * 【实测预期】
 * 1. 静止放置：Yaw会缓慢漂移（陀螺仪零漂）
 * 2. 快速旋转：姿态跟踪正确（短时间误差小）
 * 3. 5分钟后：Yaw可能偏移10-30°
 *
 ********************************************************************************************************************/
