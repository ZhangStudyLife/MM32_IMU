/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 公共数据结构定义
*
* 文件名称          imu_common.h
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-27
*
* 功能说明:
*   1. 定义IMU系统所有公共数据结构
*   2. 定义算法类型枚举
*   3. 定义统一的回调接口
*   4. 为所有算法模块提供统一的数据格式
*
* 使用注意:
*   - 所有算法模块必须使用这里定义的数据结构
*   - 原始数据 imu_raw_data_t 是只读的，所有算法共享
*   - 四元数必须归一化，否则数值会发散！
*   - 禁止直接对欧拉角积分，必须用四元数！
*
********************************************************************************************************************/

#ifndef _IMU_COMMON_H
#define _IMU_COMMON_H

#include "zf_common_typedef.h"
#include <stdint.h>
#include <stdbool.h>

/********************************************************************************************************************
 * 宏定义
 ********************************************************************************************************************/

#define IMU_PI                      (3.14159265358979323846f)       // 圆周率
#define IMU_RAD_TO_DEG              (57.295779513082320876798f)     // 弧度转角度 (180/π)
#define IMU_DEG_TO_RAD              (0.017453292519943295769f)      // 角度转弧度 (π/180)

#define IMU_GRAVITY                 (9.80665f)                      // 重力加速度 (m/s^2)

#define IMU_SAMPLE_RATE_MIN         (50)                            // 最小采样率 50Hz
#define IMU_SAMPLE_RATE_MAX         (1000)                          // 最大采样率 1000Hz

#define IMU_ACC_NORM_THRESHOLD      (0.4f)                          // 加速度模值检测阈值 (判断是否有外力)

/********************************************************************************************************************
 * 数据结构定义
 ********************************************************************************************************************/

/**
 * @brief 四元数结构（姿态表示）
 * @note  四元数避免万向锁，是所有算法内部表示姿态的标准方式
 *        q = q0 + q1*i + q2*j + q3*k
 *        q0 = w (实部), q1 = x, q2 = y, q3 = z (虚部)
 */
typedef struct
{
    float q0;   // w 分量（实部）
    float q1;   // x 分量（i）
    float q2;   // y 分量（j）
    float q3;   // z 分量（k）
} quaternion_t;

/**
 * @brief IMU原始数据结构（9轴传感器数据）
 * @note  这个结构体是只读的！所有算法共享同一份原始数据
 *        数据由顶层IMU.c统一读取并更新，各算法只能读取不能修改
 */
typedef struct
{
    // 加速度计原始数据 (单位: g, 重力加速度)
    float acc_x;        // X轴加速度
    float acc_y;        // Y轴加速度
    float acc_z;        // Z轴加速度

    // 陀螺仪原始数据 (单位: rad/s, 弧度每秒)
    float gyro_x;       // X轴角速度
    float gyro_y;       // Y轴角速度
    float gyro_z;       // Z轴角速度

    // 磁力计原始数据 (单位: Gauss, 高斯)
    float mag_x;        // X轴磁场强度
    float mag_y;        // Y轴磁场强度
    float mag_z;        // Z轴磁场强度

    // 时间戳 (单位: ms, 毫秒)
    uint32_t timestamp; // 数据采样时间戳

    // 数据有效标志位
    uint8_t acc_valid   : 1;    // 加速度计数据有效
    uint8_t gyro_valid  : 1;    // 陀螺仪数据有效
    uint8_t mag_valid   : 1;    // 磁力计数据有效
    uint8_t reserved    : 5;    // 保留位

} imu_raw_data_t;

/**
 * @brief 姿态解算结果（欧拉角表示）
 * @note  输出格式统一为角度值（度），方便使用
 *        内部必须用四元数，输出时才转换为欧拉角
 */
typedef struct
{
    float pitch;        // 俯仰角 (单位: 度, 范围: -90° ~ +90°)
                        // 抬头为正，低头为负

    float roll;         // 横滚角 (单位: 度, 范围: -180° ~ +180°)
                        // 右倾为正，左倾为负

    float yaw;          // 偏航角 (单位: 度, 范围: -180° ~ +180°)
                        // 顺时针为正（北为0°）

    uint32_t timestamp; // 时间戳 (ms)
    uint8_t valid;      // 数据有效标志 (0-无效, 1-有效)

} imu_attitude_t;

/**
 * @brief 算法类型枚举
 * @note  定义系统支持的所有姿态解算算法
 */
typedef enum
{
    IMU_ALGO_COMPLEMENTARY = 0,     // 互补滤波算法（简单快速）
    IMU_ALGO_MADGWICK,              // Madgwick算法（推荐，精度高）
    IMU_ALGO_MAHONY,                // Mahony算法（推荐，响应快）
    IMU_ALGO_EKF,                   // 扩展卡尔曼滤波（精度最高，计算量大）
    IMU_ALGO_QUATERNION,            // 纯四元数积分（基线对比用）

    IMU_ALGO_COUNT                  // 算法总数（用于数组大小）
} imu_algorithm_type_t;

/**
 * @brief 算法状态枚举
 */
typedef enum
{
    IMU_ALGO_STATE_DISABLED = 0,    // 禁用状态
    IMU_ALGO_STATE_INITIALIZING,    // 初始化中
    IMU_ALGO_STATE_RUNNING,         // 正常运行
    IMU_ALGO_STATE_ERROR            // 错误状态
} imu_algo_state_t;

/**
 * @brief 算法回调函数指针类型定义
 * @note  所有算法模块必须实现这4个回调函数，接口必须一致！
 */

// 初始化函数指针
typedef void (*imu_algo_init_func_t)(void *algo_data);

// 更新函数指针（核心算法逻辑）
typedef void (*imu_algo_update_func_t)(void *algo_data, const imu_raw_data_t *raw_data, float dt);

// 获取姿态结果函数指针
typedef void (*imu_algo_get_attitude_func_t)(void *algo_data, imu_attitude_t *attitude);

// 重置函数指针
typedef void (*imu_algo_reset_func_t)(void *algo_data);

/**
 * @brief 算法实例结构体
 * @note  每个算法维护自己独立的实例，互不干扰
 *        这是实现模块化和并行运行的核心！
 */
typedef struct
{
    // 算法基本信息
    imu_algorithm_type_t type;      // 算法类型
    imu_algo_state_t state;         // 算法状态
    const char *name;               // 算法名称（用于调试输出）

    // 算法输出
    imu_attitude_t attitude;        // 解算结果（欧拉角）
    quaternion_t quaternion;        // 内部四元数状态（避免万向锁）

    // 算法私有数据
    void *algo_specific_data;       // 算法特定数据指针（如EKF的协方差矩阵）

    // 统一回调接口（所有算法必须实现）
    imu_algo_init_func_t init;              // 初始化函数
    imu_algo_update_func_t update;          // 更新函数
    imu_algo_get_attitude_func_t get_attitude;  // 获取姿态
    imu_algo_reset_func_t reset;            // 重置函数

    // 性能统计（用于测试对比）
    uint32_t update_count;          // 更新次数
    uint32_t update_time_us;        // 最近一次更新耗时（微秒）
    uint32_t update_time_max_us;    // 最大更新耗时（微秒）

} imu_algorithm_instance_t;

/**
 * @brief IMU系统配置结构体
 */
typedef struct
{
    uint32_t sample_rate;           // 采样率 (Hz)
    float sample_period;            // 采样周期 (秒)

    bool enable_acc_check;          // 使能加速度计有效性检测
    bool enable_mag_fusion;         // 使能磁力计融合（九轴）

    // 传感器校准参数（后续扩展）
    float acc_offset[3];            // 加速度计零偏
    float gyro_offset[3];           // 陀螺仪零偏
    float mag_offset[3];            // 磁力计硬铁偏移

} imu_config_t;

/********************************************************************************************************************
 * 常用宏函数
 ********************************************************************************************************************/

/**
 * @brief 角度归一化到 [-180, 180] 范围
 */
#define IMU_NORMALIZE_ANGLE(angle)  \
    do { \
        while ((angle) > 180.0f)  (angle) -= 360.0f; \
        while ((angle) < -180.0f) (angle) += 360.0f; \
    } while(0)

/**
 * @brief 检查加速度计数据是否有效（是否受外力影响）
 * @note  当加速度模值接近1g时，说明只有重力，数据可信
 *        如果偏差过大，说明有外力加速度，此时加速度计不可信！
 */
#define IMU_IS_ACC_VALID(ax, ay, az) \
    (fabsf(sqrtf((ax)*(ax) + (ay)*(ay) + (az)*(az)) - 1.0f) < IMU_ACC_NORM_THRESHOLD)

/********************************************************************************************************************
 * 全局变量声明（由IMU.c定义）
 ********************************************************************************************************************/

// 原始数据（所有算法共享，只读）
extern const imu_raw_data_t* g_imu_raw_data;

/********************************************************************************************************************
 * 函数声明
 ********************************************************************************************************************/

// 注：具体的算法函数由各算法模块实现，这里只定义公共接口

#endif // _IMU_COMMON_H

/********************************************************************************************************************
 * 老王的叮嘱:
 *
 * 1. 艹！四元数必须归一化！每次更新后都要做，否则1小时后姿态就tm乱了！
 *
 * 2. 禁止直接对欧拉角积分！会万向锁的！必须用四元数积分，最后才转欧拉角！
 *
 * 3. 加速度计在运动时不可信！必须检测加速度模值，剧烈运动时降低权重！
 *
 * 4. 原始数据是只读的！各算法不能修改，保证公平对比！
 *
 * 5. 所有算法必须实现那4个回调函数，接口必须一致，不然老王打死你！
 *
 ********************************************************************************************************************/
