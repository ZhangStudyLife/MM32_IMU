/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 互补滤波算法
*
* 文件名称          complementary.h
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-28
*
* 功能说明:
*   1. 实现互补滤波算法（Complementary Filter）
*   2. 融合加速度计（低频准确）和陀螺仪（高频准确）
*   3. 使用四元数表示姿态，避免万向锁
*   4. 可选磁力计融合，修正Yaw角零漂
*   5. 支持加速度计有效性检测，运动时降低权重
*
* 算法原理:
*   - 核心思想: 高通滤波陀螺仪 + 低通滤波加速度计
*   - 陀螺仪: 高频响应快，但有零漂（低频不准）
*   - 加速度计: 低频准确，但有噪声（高频不准）
*   - 融合公式: attitude = α × gyro_integration + (1-α) × acc_correction
*   - α系数: 0.96~0.99，采样率越高α越大
*
* 改进版本:
*   - 使用四元数避免万向锁
*   - 加速度计修正Roll/Pitch
*   - 磁力计修正Yaw（可选）
*   - 自适应权重：运动时降低加速度计权重
*   - 陀螺仪零漂估计与补偿
*
* 优点:
*   ✅ 计算量极小（~200条指令）
*   ✅ 实时性极佳（延迟<1ms）
*   ✅ 参数少，易调试
*   ✅ 对传感器噪声不敏感
*   ✅ 有零漂补偿机制
*
* 缺点:
*   ⚠️ 动态精度一般（剧烈运动时会有偏差）
*   ⚠️ 参数固定，无法自适应调整
*   ⚠️ Yaw角依赖磁力计（易受干扰）
*
* 适用场景:
*   - 平衡车、两轮机器人
*   - 简单姿态控制
*   - 对实时性要求高的场合
*   - 智能车姿态估计
*
********************************************************************************************************************/

#ifndef _COMPLEMENTARY_H
#define _COMPLEMENTARY_H

#include "../common/imu_common.h"
#include "../common/imu_math.h"

/********************************************************************************************************************
 * 算法参数配置
 ********************************************************************************************************************/

// 融合系数α（越大越信任陀螺仪，越小越信任加速度计）
// 推荐值：100Hz→0.96, 500Hz→0.98, 1000Hz→0.99
#define COMPLEMENTARY_ALPHA_DEFAULT         (0.98f)

// 加速度计权重（静态时）
#define COMPLEMENTARY_ACC_WEIGHT_STATIC     (0.02f)  // 1 - α

// 加速度计权重（动态时，运动中降低）
#define COMPLEMENTARY_ACC_WEIGHT_DYNAMIC    (0.005f)

// 加速度模值检测阈值（判断是否在运动）
#define COMPLEMENTARY_ACC_NORM_MIN          (0.8f)   // 0.8g
#define COMPLEMENTARY_ACC_NORM_MAX          (1.2f)   // 1.2g

// 陀螺仪零漂估计系数（越小估计越慢越稳定）
#define COMPLEMENTARY_GYRO_BIAS_ALPHA       (0.001f)

// 是否使能陀螺仪零漂补偿（消除长期漂移）
#define COMPLEMENTARY_ENABLE_GYRO_BIAS_COMP (1)

// 是否使能磁力计Yaw修正
#define COMPLEMENTARY_ENABLE_MAG_YAW        (1)

// 磁力计权重（修正Yaw角）
#define COMPLEMENTARY_MAG_WEIGHT            (0.01f)

/********************************************************************************************************************
 * 算法私有数据结构
 ********************************************************************************************************************/

/**
 * @brief 互补滤波算法私有数据
 */
typedef struct
{
    // 陀螺仪零漂估计值（rad/s）
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    // 加速度计权重（动态调整）
    float acc_weight;

    // 磁力计权重
    float mag_weight;

    // 融合系数α
    float alpha;

    // 是否使能磁力计
    bool enable_mag;

    // 是否使能陀螺仪零漂补偿
    bool enable_gyro_bias_comp;

    // 统计信息（用于调试）
    uint32_t acc_invalid_count;     // 加速度计无效次数
    uint32_t gyro_bias_update_count; // 零漂更新次数

} complementary_algo_data_t;

/********************************************************************************************************************
 * 标准算法接口（必须实现）
 ********************************************************************************************************************/

/**
 * @brief 初始化互补滤波算法
 * @param algo_data 算法私有数据指针
 */
void complementary_init(void *algo_data);

/**
 * @brief 更新互补滤波算法（核心函数）
 * @param algo_data 算法私有数据指针
 * @param raw_data IMU原始数据（只读）
 * @param dt 时间步长（秒）
 */
void complementary_update(void *algo_data, const imu_raw_data_t *raw_data, float dt);

/**
 * @brief 获取互补滤波算法的姿态结果
 * @param algo_data 算法私有数据指针
 * @param attitude 输出姿态数据
 */
void complementary_get_attitude(void *algo_data, imu_attitude_t *attitude);

/**
 * @brief 重置互补滤波算法
 * @param algo_data 算法私有数据指针
 */
void complementary_reset(void *algo_data);

/**
 * @brief 注册互补滤波算法到IMU系统
 * @param instance 算法实例指针
 * @return 0-成功, 非0-失败
 */
int complementary_register(imu_algorithm_instance_t *instance);

/********************************************************************************************************************
 * 算法参数配置接口
 ********************************************************************************************************************/

/**
 * @brief 设置融合系数α
 * @param alpha 融合系数（0.9~0.99）
 * @note  α越大越信任陀螺仪，越小越信任加速度计
 *        推荐值：100Hz→0.96, 500Hz→0.98, 1000Hz→0.99
 */
void complementary_set_alpha(float alpha);

/**
 * @brief 设置加速度计权重
 * @param weight_static 静态权重（0.01~0.05）
 * @param weight_dynamic 动态权重（0.001~0.01）
 */
void complementary_set_acc_weight(float weight_static, float weight_dynamic);

/**
 * @brief 设置是否使能磁力计Yaw修正
 * @param enable true-使能, false-禁用
 */
void complementary_set_mag_enable(bool enable);

/**
 * @brief 设置是否使能陀螺仪零漂补偿
 * @param enable true-使能, false-禁用
 */
void complementary_set_gyro_bias_comp_enable(bool enable);

/**
 * @brief 获取当前陀螺仪零漂估计值
 * @param bias_x 输出X轴零漂（rad/s）
 * @param bias_y 输出Y轴零漂（rad/s）
 * @param bias_z 输出Z轴零漂（rad/s）
 */
void complementary_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z);

/**
 * @brief 手动设置陀螺仪零漂值（用于快速校准）
 * @param bias_x X轴零漂（rad/s）
 * @param bias_y Y轴零漂（rad/s）
 * @param bias_z Z轴零漂（rad/s）
 */
void complementary_set_gyro_bias(float bias_x, float bias_y, float bias_z);

#endif // _COMPLEMENTARY_H

/********************************************************************************************************************
 * 老王的技术笔记:
 *
 * 【互补滤波的数学原理】
 * 互补滤波器本质是高通滤波器和低通滤波器的组合：
 * - 陀螺仪积分 = 高通滤波（保留高频运动，但有零漂）
 * - 加速度计 = 低通滤波（去除高频噪声，但有延迟）
 * - 融合：attitude = α × gyro + (1-α) × acc
 *
 * 【四元数互补滤波】
 * 传统互补滤波直接对欧拉角操作，会万向锁！
 * 老王这个版本用四元数表示姿态：
 * 1. 陀螺仪积分更新四元数
 * 2. 加速度计计算Roll/Pitch修正
 * 3. 磁力计计算Yaw修正
 * 4. 用修正向量旋转四元数
 *
 * 【零漂补偿机制】
 * 陀螺仪有零漂是无法避免的！必须补偿！
 * 方法：
 * 1. 静态时：加速度计姿态 - 陀螺仪积分姿态 = 陀螺仪误差
 * 2. 低通滤波误差 → 估计零漂
 * 3. 下次更新时：gyro_corrected = gyro_raw - gyro_bias
 *
 * 【自适应权重】
 * 运动时加速度计不可信（有外力加速度）！
 * 检测方法：|加速度模值 - 1g| > 阈值
 * 如果在运动：降低加速度计权重，更信任陀螺仪
 *
 * 【参数调优建议】
 * 1. 静态测试：调α使Pitch/Roll稳定，无抖动
 * 2. 动态测试：快速旋转，调权重使响应快且不超调
 * 3. 长期测试：30分钟后Yaw漂移<5°，说明零漂补偿有效
 *
 * 艹！互补滤波虽然简单，但调好了效果也很NB！
 * 关键是零漂补偿！没有零漂补偿的互补滤波就是辣鸡！
 *
 ********************************************************************************************************************/
