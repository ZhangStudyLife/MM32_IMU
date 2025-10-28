/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 顶层统一接口实现
*
* 文件名称          IMU.c
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-27
*
* 功能说明:
*   实现IMU.h中声明的所有接口函数
*
********************************************************************************************************************/

#include "IMU.h"
#include "Quaternion/quaternion.h"
#include "Complementary/complementary.h"
#include "Madgwick/madgwick.h"
#include "Mahony/mahony.h"
#include "EKF/ekf.h"
#include "zf_device_imu963ra.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/********************************************************************************************************************
 * 全局变量定义
 ********************************************************************************************************************/

// IMU原始数据（所有算法共享，只读）
static imu_raw_data_t g_imu_raw_data_internal;
const imu_raw_data_t* g_imu_raw_data = &g_imu_raw_data_internal;

// 系统配置
static imu_config_t g_imu_config;

// 算法实例数组（管理所有算法）
static imu_algorithm_instance_t g_imu_algo_instances[IMU_ALGO_COUNT];

// 算法名称表（用于调试输出）
static const char* g_algo_names[IMU_ALGO_COUNT] = {
    "Complementary",
    "Madgwick",
    "Mahony",
    "EKF",
    "Quaternion"
};

// 系统初始化标志
static bool g_imu_system_initialized = false;

// 上次更新时间戳（用于计算dt）
static uint32_t g_last_update_timestamp_ms = 0;

/********************************************************************************************************************
 * 内部辅助函数
 ********************************************************************************************************************/

/**
 * @brief 读取IMU963RA原始数据并进行预处理
 */
static void imu_read_raw_data(void)
{
    // 简单的时间戳计数器（每次调用递增）
    static uint32_t timestamp_counter = 0;

    // 读取加速度计数据
    imu963ra_get_acc();
    g_imu_raw_data_internal.acc_x = imu963ra_acc_transition(imu963ra_acc_x);
    g_imu_raw_data_internal.acc_y = imu963ra_acc_transition(imu963ra_acc_y);
    g_imu_raw_data_internal.acc_z = imu963ra_acc_transition(imu963ra_acc_z);
    g_imu_raw_data_internal.acc_valid = 1;

    // 读取陀螺仪数据（转换为rad/s）
    imu963ra_get_gyro();
    g_imu_raw_data_internal.gyro_x = imu963ra_gyro_transition(imu963ra_gyro_x) * IMU_DEG_TO_RAD;
    g_imu_raw_data_internal.gyro_y = imu963ra_gyro_transition(imu963ra_gyro_y) * IMU_DEG_TO_RAD;
    g_imu_raw_data_internal.gyro_z = imu963ra_gyro_transition(imu963ra_gyro_z) * IMU_DEG_TO_RAD;
    g_imu_raw_data_internal.gyro_valid = 1;

    // 读取磁力计数据
    if (g_imu_config.enable_mag_fusion)
    {
        imu963ra_get_mag();
        g_imu_raw_data_internal.mag_x = imu963ra_mag_transition(imu963ra_mag_x);
        g_imu_raw_data_internal.mag_y = imu963ra_mag_transition(imu963ra_mag_y);
        g_imu_raw_data_internal.mag_z = imu963ra_mag_transition(imu963ra_mag_z);
        g_imu_raw_data_internal.mag_valid = 1;
    }
    else
    {
        g_imu_raw_data_internal.mag_valid = 0;
    }

    // 更新时间戳（使用简单的递增计数器，单位ms）
    timestamp_counter += (uint32_t)(g_imu_config.sample_period * 1000.0f);
    g_imu_raw_data_internal.timestamp = timestamp_counter;

    // 数据校准（如果有校准参数）
    if (g_imu_config.acc_offset[0] != 0.0f || g_imu_config.acc_offset[1] != 0.0f || g_imu_config.acc_offset[2] != 0.0f)
    {
        g_imu_raw_data_internal.acc_x -= g_imu_config.acc_offset[0];
        g_imu_raw_data_internal.acc_y -= g_imu_config.acc_offset[1];
        g_imu_raw_data_internal.acc_z -= g_imu_config.acc_offset[2];
    }

    if (g_imu_config.gyro_offset[0] != 0.0f || g_imu_config.gyro_offset[1] != 0.0f || g_imu_config.gyro_offset[2] != 0.0f)
    {
        g_imu_raw_data_internal.gyro_x -= g_imu_config.gyro_offset[0];
        g_imu_raw_data_internal.gyro_y -= g_imu_config.gyro_offset[1];
        g_imu_raw_data_internal.gyro_z -= g_imu_config.gyro_offset[2];
    }

    // ✅ 死区处理（Dead Zone）- 消除微小噪声累积
    // 去零漂后，绝对值小于阈值的视作0，避免静态时的微小噪声积分导致漂移
    // 阈值：0.01 rad/s ≈ 0.57°/s，这是工程实践中的经验值
    const float GYRO_DEAD_ZONE = 0.01f;  // rad/s

    if (fabsf(g_imu_raw_data_internal.gyro_x) < GYRO_DEAD_ZONE)
    {
        g_imu_raw_data_internal.gyro_x = 0.0f;
    }
    if (fabsf(g_imu_raw_data_internal.gyro_y) < GYRO_DEAD_ZONE)
    {
        g_imu_raw_data_internal.gyro_y = 0.0f;
    }
    if (fabsf(g_imu_raw_data_internal.gyro_z) < GYRO_DEAD_ZONE)
    {
        g_imu_raw_data_internal.gyro_z = 0.0f;
    }

    if (g_imu_config.mag_offset[0] != 0.0f || g_imu_config.mag_offset[1] != 0.0f || g_imu_config.mag_offset[2] != 0.0f)
    {
        g_imu_raw_data_internal.mag_x -= g_imu_config.mag_offset[0];
        g_imu_raw_data_internal.mag_y -= g_imu_config.mag_offset[1];
        g_imu_raw_data_internal.mag_z -= g_imu_config.mag_offset[2];
    }
}

/********************************************************************************************************************
 * 系统初始化与配置实现
 ********************************************************************************************************************/

/**
 * @brief 初始化IMU系统
 */
int imu_system_init(uint32_t sample_rate)
{
    // 参数检查
    if (sample_rate < IMU_SAMPLE_RATE_MIN || sample_rate > IMU_SAMPLE_RATE_MAX)
    {
        return -1;
    }

    // 初始化IMU963RA硬件
    if (0 != imu963ra_init())
    {
        return -2;
    }

    // 初始化系统配置
    memset(&g_imu_config, 0, sizeof(imu_config_t));
    g_imu_config.sample_rate = sample_rate;
    g_imu_config.sample_period = 1.0f / (float)sample_rate;
    g_imu_config.enable_acc_check = true;
    g_imu_config.enable_mag_fusion = true;

    // 初始化原始数据缓冲区
    memset(&g_imu_raw_data_internal, 0, sizeof(imu_raw_data_t));

    // 初始化所有算法实例（但不使能）
    for (int i = 0; i < IMU_ALGO_COUNT; i++)
    {
        g_imu_algo_instances[i].type = (imu_algorithm_type_t)i;
        g_imu_algo_instances[i].state = IMU_ALGO_STATE_DISABLED;
        g_imu_algo_instances[i].name = g_algo_names[i];

        // 初始化四元数为单位四元数
        imu_quat_identity(&g_imu_algo_instances[i].quaternion);

        // 初始化姿态为0
        memset(&g_imu_algo_instances[i].attitude, 0, sizeof(imu_attitude_t));

        // 算法特定数据和回调函数由各算法模块初始化时设置
        g_imu_algo_instances[i].algo_specific_data = NULL;
        g_imu_algo_instances[i].init = NULL;
        g_imu_algo_instances[i].update = NULL;
        g_imu_algo_instances[i].get_attitude = NULL;
        g_imu_algo_instances[i].reset = NULL;

        // 性能统计清零
        g_imu_algo_instances[i].update_count = 0;
        g_imu_algo_instances[i].update_time_us = 0;
        g_imu_algo_instances[i].update_time_max_us = 0;
    }

    // 注册所有算法模块
    quaternion_register(&g_imu_algo_instances[IMU_ALGO_QUATERNION]);
    complementary_register(&g_imu_algo_instances[IMU_ALGO_COMPLEMENTARY]);
    madgwick_register(&g_imu_algo_instances[IMU_ALGO_MADGWICK]);
    mahony_register(&g_imu_algo_instances[IMU_ALGO_MAHONY]);
    ekf_register(&g_imu_algo_instances[IMU_ALGO_EKF]);

    // 读取一次数据初始化时间戳
    imu_read_raw_data();
    g_last_update_timestamp_ms = g_imu_raw_data_internal.timestamp;

    g_imu_system_initialized = true;

    return 0;
}

/**
 * @brief 获取IMU系统配置
 */
void imu_get_config(imu_config_t *config)
{
    if (config != NULL)
    {
        memcpy(config, &g_imu_config, sizeof(imu_config_t));
    }
}

/**
 * @brief 设置IMU系统配置
 */
int imu_set_config(const imu_config_t *config)
{
    if (config == NULL)
    {
        return -1;
    }

    // 参数检查
    if (config->sample_rate < IMU_SAMPLE_RATE_MIN || config->sample_rate > IMU_SAMPLE_RATE_MAX)
    {
        return -2;
    }

    memcpy(&g_imu_config, config, sizeof(imu_config_t));

    return 0;
}

/********************************************************************************************************************
 * 算法管理实现
 ********************************************************************************************************************/

/**
 * @brief 使能指定算法
 */
int imu_algorithm_enable(imu_algorithm_type_t algo_type)
{
    if (!g_imu_system_initialized)
    {
        return -1;
    }

    if (algo_type >= IMU_ALGO_COUNT)
    {
        return -2;
    }

    imu_algorithm_instance_t *instance = &g_imu_algo_instances[algo_type];

    // 检查算法是否已注册（回调函数是否设置）
    if (instance->init == NULL || instance->update == NULL)
    {
        return -3;
    }

    // 如果已经使能，直接返回
    if (instance->state == IMU_ALGO_STATE_RUNNING)
    {
        return 0;
    }

    // 调用算法初始化函数
    instance->state = IMU_ALGO_STATE_INITIALIZING;
    instance->init(instance->algo_specific_data);
    instance->state = IMU_ALGO_STATE_RUNNING;

    return 0;
}

/**
 * @brief 禁用指定算法
 */
void imu_algorithm_disable(imu_algorithm_type_t algo_type)
{
    if (algo_type >= IMU_ALGO_COUNT)
    {
        return;
    }

    g_imu_algo_instances[algo_type].state = IMU_ALGO_STATE_DISABLED;
}

/**
 * @brief 检查算法是否已使能
 */
bool imu_algorithm_is_enabled(imu_algorithm_type_t algo_type)
{
    if (algo_type >= IMU_ALGO_COUNT)
    {
        return false;
    }

    return (g_imu_algo_instances[algo_type].state == IMU_ALGO_STATE_RUNNING);
}

/**
 * @brief 获取算法状态
 */
imu_algo_state_t imu_algorithm_get_state(imu_algorithm_type_t algo_type)
{
    if (algo_type >= IMU_ALGO_COUNT)
    {
        return IMU_ALGO_STATE_ERROR;
    }

    return g_imu_algo_instances[algo_type].state;
}

/**
 * @brief 重置指定算法
 */
void imu_algorithm_reset(imu_algorithm_type_t algo_type)
{
    if (algo_type >= IMU_ALGO_COUNT)
    {
        return;
    }

    imu_algorithm_instance_t *instance = &g_imu_algo_instances[algo_type];

    if (instance->reset != NULL)
    {
        instance->reset(instance->algo_specific_data);
    }

    // 重置性能统计
    instance->update_count = 0;
    instance->update_time_us = 0;
    instance->update_time_max_us = 0;
}

/**
 * @brief 重置所有算法
 */
void imu_reset_all(void)
{
    for (int i = 0; i < IMU_ALGO_COUNT; i++)
    {
        imu_algorithm_reset((imu_algorithm_type_t)i);
    }
}

/********************************************************************************************************************
 * 数据更新与获取实现
 ********************************************************************************************************************/

/**
 * @brief 更新IMU数据（周期调用）
 */
void imu_update(void)
{
    if (!g_imu_system_initialized)
    {
        return;
    }

    // 1. 读取IMU963RA原始数据
    imu_read_raw_data();

    // 2. 计算时间步长dt
    uint32_t current_timestamp = g_imu_raw_data_internal.timestamp;
    float dt = (float)(current_timestamp - g_last_update_timestamp_ms) / 1000.0f;  // 转换为秒
    g_last_update_timestamp_ms = current_timestamp;

    // 防止dt异常（第一次调用或时间戳跳变）
    if (dt <= 0.0f || dt > 1.0f)
    {
        dt = g_imu_config.sample_period;
    }

    // 3. 更新所有已使能的算法
    for (int i = 0; i < IMU_ALGO_COUNT; i++)
    {
        imu_algorithm_instance_t *instance = &g_imu_algo_instances[i];

        if (instance->state == IMU_ALGO_STATE_RUNNING && instance->update != NULL)
        {
            // 记录开始时间（用于性能统计）
            uint32_t start_time_us = 0;  // TODO: 使用定时器获取微秒级时间戳

            // 调用算法更新函数
            instance->update(instance->algo_specific_data, &g_imu_raw_data_internal, dt);

            // 记录结束时间并更新统计
            uint32_t end_time_us = 0;  // TODO: 使用定时器获取微秒级时间戳
            uint32_t elapsed_us = end_time_us - start_time_us;

            instance->update_time_us = elapsed_us;
            instance->update_count++;

            if (elapsed_us > instance->update_time_max_us)
            {
                instance->update_time_max_us = elapsed_us;
            }
        }
    }
}

/**
 * @brief 获取指定算法的姿态结果
 */
int imu_get_attitude(imu_algorithm_type_t algo_type, imu_attitude_t *attitude)
{
    if (algo_type >= IMU_ALGO_COUNT || attitude == NULL)
    {
        return -1;
    }

    imu_algorithm_instance_t *instance = &g_imu_algo_instances[algo_type];

    if (instance->state != IMU_ALGO_STATE_RUNNING)
    {
        return -2;
    }

    if (instance->get_attitude != NULL)
    {
        instance->get_attitude(instance->algo_specific_data, attitude);
    }
    else
    {
        // 如果算法没有提供get_attitude函数，直接从instance中读取
        memcpy(attitude, &instance->attitude, sizeof(imu_attitude_t));
    }

    return 0;
}

/**
 * @brief 获取指定算法的四元数
 */
int imu_get_quaternion(imu_algorithm_type_t algo_type, quaternion_t *quat)
{
    if (algo_type >= IMU_ALGO_COUNT || quat == NULL)
    {
        return -1;
    }

    imu_algorithm_instance_t *instance = &g_imu_algo_instances[algo_type];

    if (instance->state != IMU_ALGO_STATE_RUNNING)
    {
        return -2;
    }

    memcpy(quat, &instance->quaternion, sizeof(quaternion_t));

    return 0;
}

/**
 * @brief 获取IMU原始数据（只读）
 */
const imu_raw_data_t* imu_get_raw_data(void)
{
    return g_imu_raw_data;
}

/********************************************************************************************************************
 * 调试与测试实现
 ********************************************************************************************************************/

/**
 * @brief 打印所有算法的姿态对比
 */
void imu_print_all_attitudes(void)
{
    // 调试函数已禁用
}

/**
 * @brief 打印指定算法的姿态
 */
void imu_print_attitude(imu_algorithm_type_t algo_type)
{
    // 调试函数已禁用
}

/**
 * @brief 打印原始传感器数据
 */
void imu_print_raw_data(void)
{
    // 调试函数已禁用
}

/**
 * @brief 获取算法性能统计
 */
int imu_get_performance(imu_algorithm_type_t algo_type,
                        uint32_t *update_count,
                        uint32_t *avg_time_us,
                        uint32_t *max_time_us)
{
    if (algo_type >= IMU_ALGO_COUNT)
    {
        return -1;
    }

    imu_algorithm_instance_t *instance = &g_imu_algo_instances[algo_type];

    if (update_count != NULL)
    {
        *update_count = instance->update_count;
    }

    if (avg_time_us != NULL)
    {
        *avg_time_us = instance->update_time_us;
    }

    if (max_time_us != NULL)
    {
        *max_time_us = instance->update_time_max_us;
    }

    return 0;
}

/**
 * @brief 打印所有算法的性能统计
 */
void imu_print_performance(void)
{
    // 调试函数已禁用
}

/********************************************************************************************************************
 * 传感器校准实现（后续扩展，暂时留空）
 ********************************************************************************************************************/

void imu_calibrate_acc_start(void)
{
    // 加速度计静态校准（设备必须水平静止放置！）
    const uint16_t sample_count = 200;  // 采样200次
    float acc_sum[3] = {0.0f, 0.0f, 0.0f};

    for (uint16_t i = 0; i < sample_count; i++)
    {
        // 读取加速度计原始数据
        imu963ra_get_acc();

        // 转换为g并累加
        acc_sum[0] += imu963ra_acc_transition(imu963ra_acc_x);
        acc_sum[1] += imu963ra_acc_transition(imu963ra_acc_y);
        acc_sum[2] += imu963ra_acc_transition(imu963ra_acc_z);

        system_delay_ms(5);  // 5ms间隔，总耗时1秒
    }

    // 计算平均值
    float acc_avg[3];
    acc_avg[0] = acc_sum[0] / (float)sample_count;
    acc_avg[1] = acc_sum[1] / (float)sample_count;
    acc_avg[2] = acc_sum[2] / (float)sample_count;

    // 校准偏移：理想情况下应该是[0, 0, 1]g
    // X和Y轴偏移直接使用平均值
    g_imu_config.acc_offset[0] = acc_avg[0];
    g_imu_config.acc_offset[1] = acc_avg[1];
    // Z轴需要减去重力加速度1g
    g_imu_config.acc_offset[2] = acc_avg[2] - 1.0f;
}

void imu_calibrate_gyro_start(void)
{
    // 陀螺仪静态校准（设备必须完全静止！）
    const uint16_t sample_count = 2000;  // 采样200次
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};

    for (uint16_t i = 0; i < sample_count; i++)
    {
        // 读取陀螺仪原始数据
        imu963ra_get_gyro();

        // 转换为rad/s并累加
        gyro_sum[0] += imu963ra_gyro_transition(imu963ra_gyro_x) * IMU_DEG_TO_RAD;
        gyro_sum[1] += imu963ra_gyro_transition(imu963ra_gyro_y) * IMU_DEG_TO_RAD;
        gyro_sum[2] += imu963ra_gyro_transition(imu963ra_gyro_z) * IMU_DEG_TO_RAD;

        system_delay_ms(5);  // 5ms间隔，总耗时1秒
    }

    // 计算平均值作为零漂
    g_imu_config.gyro_offset[0] = gyro_sum[0] / (float)sample_count;
    g_imu_config.gyro_offset[1] = gyro_sum[1] / (float)sample_count;
    g_imu_config.gyro_offset[2] = gyro_sum[2] / (float)sample_count;
}

void imu_calibrate_mag_start(void)
{
    // 校准功能未实现
}

bool imu_calibrate_is_done(void)
{
    return false;
}

void imu_calibrate_save(void)
{
    // 校准功能未实现
}

void imu_calibrate_load(void)
{
    // 校准功能未实现
}

/********************************************************************************************************************
 * 老王的实现笔记:
 *
 * 艹！这个顶层接口写得够详细了吧！
 *
 * 【设计要点】
 * 1. 所有算法实例存储在一个数组中，便于统一管理
 * 2. 原始数据只读访问，保证各算法数据源一致
 * 3. 算法回调函数指针设计，实现多态
 * 4. 性能统计功能，方便优化和对比
 *
 * 【待完善】
 * 1. 时间戳获取函数需要根据实际硬件实现（system_getval_ms）
 * 2. 微秒级时间戳用于性能统计（需要高精度定时器）
 * 3. 传感器校准功能后续实现
 * 4. 各算法模块的具体实现（Phase 2-6）
 *
 * 【下一步】
 * 现在基础框架已经搭建完成，可以开始实现各个算法模块了！
 * 建议顺序：Quaternion → Complementary → Madgwick → Mahony → EKF
 *
 ********************************************************************************************************************/
