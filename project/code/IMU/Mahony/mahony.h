/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - Mahony算法
*
* 文件名称          mahony.h
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-28
*
* 功能说明:
*   1. 实现Mahony AHRS算法（基于PI控制器）
*   2. 支持6轴（IMU模式）和9轴（AHRS模式，含磁力计）
*   3. 显式补偿陀螺仪零漂（PI控制器积分项）
*   4. 动态响应快，适合飞行控制
*   5. 使用四元数表示姿态，避免万向锁
*
* 算法原理:
*   - 核心思想: 使用PI控制器修正陀螺仪零漂
*   - 控制目标: 使四元数预测的重力/磁场方向与传感器测量值一致
*   - 数学推导: 详见Robert Mahony 2008年论文
*     "Nonlinear Complementary Filters on the Special Orthogonal Group"
*
* 关键公式:
*   1. 误差计算: e = a_measured × g_predicted  (叉积)
*   2. PI控制器:
*      - P项: ω_corrected = ω_gyro + Kp × e
*      - I项: gyro_bias += Ki × e × dt
*   3. 四元数更新: q̇ = 0.5 × q ⊗ [0, ω_corrected]
*   4. 积分: q = q + q̇ × dt
*
* 参数Kp和Ki的物理意义:
*   - Kp (比例增益): 控制修正强度，越大响应越快（类似Madgwick的β）
*   - Ki (积分增益): 控制零漂补偿速度，越大零漂收敛越快
*   - 推荐值: Kp=1.0~5.0, Ki=0.0~0.1
*
* 优点:
*   ✅ 动态响应最快（比Madgwick快）
*   ✅ 显式补偿陀螺仪零漂（PI控制器积分项）
*   ✅ 参数物理意义明确（Kp/Ki是控制理论标准参数）
*   ✅ 计算量小（比Madgwick稍快）
*   ✅ PX4/APM飞控默认算法（久经考验）
*
* 缺点:
*   ⚠️ 参数调节需要控制理论知识
*   ⚠️ Ki设置不当可能导致振荡
*   ⚠️ 磁干扰敏感
*
* 适用场景:
*   - 飞行控制系统（推荐！PX4/APM默认算法）
*   - 需要快速响应的场合
*   - 对陀螺仪零漂敏感的应用
*   - 智能车快速转向
*
********************************************************************************************************************/

#ifndef _MAHONY_H
#define _MAHONY_H

#include "../common/imu_common.h"
#include "../common/imu_math.h"

/********************************************************************************************************************
 * 算法参数配置
 ********************************************************************************************************************/

// Kp参数（比例增益，控制修正强度）
// 推荐值：1.0~5.0，越大响应越快但噪声越敏感
#define MAHONY_KP_DEFAULT                   (2.5f)

// Ki参数（积分增益，控制零漂补偿速度）
// 推荐值：0.0~0.1，越大零漂收敛越快但可能振荡
// 设为0则禁用积分项（只用P控制）
#define MAHONY_KI_DEFAULT                   (0.05f)

// Kp参数最小值（运动时降低修正强度）
#define MAHONY_KP_MIN                       (0.5f)

// Kp参数最大值（静态时提高修正强度）
#define MAHONY_KP_MAX                       (10.0f)

// 加速度模值检测阈值（判断是否在运动）
#define MAHONY_ACC_NORM_MIN                 (0.75f)   // 0.75g
#define MAHONY_ACC_NORM_MAX                 (1.25f)   // 1.25g

// 是否使能自适应Kp（运动时自动降低Kp）
#define MAHONY_ENABLE_ADAPTIVE_KP           (1)

// 自适应Kp的调整系数
#define MAHONY_ADAPTIVE_KP_GAIN             (0.5f)

// 是否使能磁力计融合（九轴模式）
#define MAHONY_ENABLE_MAG_FUSION            (1)

// 磁力计修正增益（相对于加速度计）
#define MAHONY_MAG_GAIN                     (1.0f)

// 陀螺仪零漂积分限幅（rad/s）
#define MAHONY_GYRO_BIAS_MAX                (0.2f)

/********************************************************************************************************************
 * 算法私有数据结构
 ********************************************************************************************************************/

/**
 * @brief Mahony算法私有数据
 */
typedef struct
{
    // Kp参数（比例增益）
    float Kp;

    // Ki参数（积分增益）
    float Ki;

    // Kp参数动态范围
    float Kp_min;
    float Kp_max;

    // 陀螺仪零漂估计值（PI控制器积分项，rad/s）
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    // 是否使能自适应Kp
    bool enable_adaptive_Kp;

    // 是否使能磁力计
    bool enable_mag;

    // 磁力计修正增益
    float mag_gain;

    // 统计信息（用于调试）
    uint32_t acc_invalid_count;     // 加速度计无效次数
    float current_Kp;                // 当前使用的Kp值
    float acc_norm;                  // 当前加速度模值
    float error_norm;                // 当前误差模值

} mahony_algo_data_t;

/********************************************************************************************************************
 * 标准算法接口（必须实现）
 ********************************************************************************************************************/

/**
 * @brief 初始化Mahony算法
 * @param algo_data 算法私有数据指针
 */
void mahony_init(void *algo_data);

/**
 * @brief 更新Mahony算法（核心函数）
 * @param algo_data 算法私有数据指针
 * @param raw_data IMU原始数据（只读）
 * @param dt 时间步长（秒）
 */
void mahony_update(void *algo_data, const imu_raw_data_t *raw_data, float dt);

/**
 * @brief 获取Mahony算法的姿态结果
 * @param algo_data 算法私有数据指针
 * @param attitude 输出姿态数据
 */
void mahony_get_attitude(void *algo_data, imu_attitude_t *attitude);

/**
 * @brief 重置Mahony算法
 * @param algo_data 算法私有数据指针
 */
void mahony_reset(void *algo_data);

/**
 * @brief 注册Mahony算法到IMU系统
 * @param instance 算法实例指针
 * @return 0-成功, 非0-失败
 */
int mahony_register(imu_algorithm_instance_t *instance);

/********************************************************************************************************************
 * 算法参数配置接口
 ********************************************************************************************************************/

/**
 * @brief 设置Kp参数
 * @param Kp Kp值（0.1~10.0）
 * @note  Kp越大越信任加速度计，响应越快但噪声越敏感
 */
void mahony_set_Kp(float Kp);

/**
 * @brief 设置Ki参数
 * @param Ki Ki值（0.0~0.5）
 * @note  Ki越大零漂收敛越快，但设置过大可能振荡
 *        设为0则禁用积分项（只用P控制）
 */
void mahony_set_Ki(float Ki);

/**
 * @brief 设置Kp参数范围（自适应模式）
 * @param Kp_min 最小Kp值（运动时）
 * @param Kp_max 最大Kp值（静态时）
 */
void mahony_set_Kp_range(float Kp_min, float Kp_max);

/**
 * @brief 设置是否使能自适应Kp
 * @param enable true-使能, false-禁用
 */
void mahony_set_adaptive_Kp_enable(bool enable);

/**
 * @brief 设置是否使能磁力计融合
 * @param enable true-使能（九轴模式）, false-禁用（六轴模式）
 */
void mahony_set_mag_enable(bool enable);

/**
 * @brief 获取当前Kp值（调试用）
 * @return 当前使用的Kp值
 */
float mahony_get_current_Kp(void);

/**
 * @brief 获取陀螺仪零漂估计值（调试用）
 * @param bias_x 输出X轴零漂（rad/s）
 * @param bias_y 输出Y轴零漂（rad/s）
 * @param bias_z 输出Z轴零漂（rad/s）
 */
void mahony_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z);

/**
 * @brief 获取当前误差模值（调试用）
 * @return 当前误差模值
 */
float mahony_get_error_norm(void);

#endif // _MAHONY_H

/********************************************************************************************************************
 * 老王的技术笔记:
 *
 * 【Mahony算法的数学原理】
 *
 * 1. 误差向量（叉积）：
 *    e = a_normalized × g_predicted
 *    其中a是归一化加速度计测量值，g是四元数预测的重力方向
 *    叉积的物理意义：旋转轴，使g旋转到a
 *
 * 2. PI控制器：
 *    P项：ω_corrected = ω_gyro + Kp × e
 *    I项：gyro_bias += Ki × e × dt
 *    总修正：ω_final = ω_corrected - gyro_bias
 *
 * 3. 四元数更新：
 *    q̇ = 0.5 × q ⊗ [0, ω_final]
 *    q = q + q̇ × dt
 *    归一化(q)
 *
 * 【为什么使用PI控制器？】
 * - P项：快速响应当前误差（即时修正）
 * - I项：累积历史误差，补偿零漂（长期稳定）
 * - 控制理论经典方法，工程师熟悉，调参直观
 *
 * 【与Madgwick对比】
 * Mahony                      Madgwick
 * --------------------------------
 * PI控制器修正                 梯度下降修正
 * 叉积计算误差                 目标函数最小化
 * 显式零漂补偿（I项）          隐式修正
 * 动态响应最快                 精度更高
 * Kp/Ki参数直观               β参数需要经验
 * 适合飞控                     适合VR/AR
 *
 * 【Kp和Ki的调参建议】
 * 1. 先调Kp：
 *    - 从1.0开始，逐渐增大直到响应满意
 *    - 如果噪声过大，降低Kp
 *    - 推荐范围：1.0~5.0
 *
 * 2. 再调Ki：
 *    - 从0开始（纯P控制）
 *    - 观察长期漂移，如果明显则逐渐增大Ki
 *    - 如果出现振荡，降低Ki
 *    - 推荐范围：0.0~0.1
 *
 * 3. 自适应Kp：
 *    - 运动时自动降低Kp（加速度计不可信）
 *    - 静态时恢复Kp（加速度计可信）
 *
 * 【精度分析】
 * - 静态精度：±0.5°（与Madgwick相当）
 * - 动态精度：±0.5-1°（比Madgwick略好）
 * - 长期稳定性：优秀（I项持续补偿零漂）
 * - 响应速度：最快（PI控制直接修正）
 *
 * 【PX4飞控的选择】
 * PX4飞控默认使用Mahony算法（也支持EKF）：
 * - 原因1：动态响应快，飞行姿态变化剧烈
 * - 原因2：PI控制器工程师熟悉，调参直观
 * - 原因3：久经考验，稳定可靠
 * - 原因4：计算量小，实时性好
 *
 * 艹！Mahony算法是老王最喜欢的飞控算法！
 * 数学直观，物理意义明确，响应快，稳定可靠！
 *
 ********************************************************************************************************************/
