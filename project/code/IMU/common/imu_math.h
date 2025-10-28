/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 四元数数学运算库
*
* 文件名称          imu_math.h
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-27
*
* 功能说明:
*   1. 提供四元数基本运算（加减乘除、归一化）
*   2. 提供四元数与欧拉角相互转换
*   3. 提供向量运算（叉积、点积、归一化）
*   4. 提供旋转相关运算
*   5. 为所有姿态解算算法提供数学基础
*
* 使用注意:
*   - 四元数必须保持归一化！否则会数值发散！
*   - 欧拉角转换使用ZYX顺序，避免万向锁
*   - 所有角度参数使用弧度制，输出角度制需手动转换
*
********************************************************************************************************************/

#ifndef _IMU_MATH_H
#define _IMU_MATH_H

#include "imu_common.h"
#include <math.h>

/********************************************************************************************************************
 * 四元数基本运算
 ********************************************************************************************************************/

/**
 * @brief 初始化四元数为单位四元数 [1, 0, 0, 0]
 * @param q 四元数指针
 * @note  单位四元数表示无旋转状态
 */
void imu_quat_identity(quaternion_t *q);

/**
 * @brief 四元数归一化（超级重要！）
 * @param q 四元数指针
 * @note  艹！每次更新四元数后必须调用这个函数！
 *        否则数值误差累积，1小时后姿态就tm乱了！
 */
void imu_quat_normalize(quaternion_t *q);

/**
 * @brief 四元数共轭（相当于逆旋转）
 * @param q 输入四元数
 * @param q_conj 输出共轭四元数
 * @note  q* = [w, -x, -y, -z]
 */
void imu_quat_conjugate(const quaternion_t *q, quaternion_t *q_conj);

/**
 * @brief 四元数乘法 q_result = q1 * q2
 * @param q1 左操作数
 * @param q2 右操作数
 * @param q_result 结果（可以与q1或q2相同）
 * @note  四元数乘法不满足交换律！q1*q2 != q2*q1
 *        表示旋转复合：先q2旋转，再q1旋转
 */
void imu_quat_multiply(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *q_result);

/**
 * @brief 四元数与标量相加 q_result = q + scalar
 * @param q 输入四元数
 * @param scalar 标量值（只加到实部）
 * @param q_result 结果
 */
void imu_quat_add_scalar(const quaternion_t *q, float scalar, quaternion_t *q_result);

/**
 * @brief 四元数与标量相乘 q_result = q * scalar
 * @param q 输入四元数
 * @param scalar 标量值
 * @param q_result 结果
 */
void imu_quat_multiply_scalar(const quaternion_t *q, float scalar, quaternion_t *q_result);

/**
 * @brief 四元数模长
 * @param q 输入四元数
 * @return 模长 sqrt(w^2 + x^2 + y^2 + z^2)
 */
float imu_quat_norm(const quaternion_t *q);

/********************************************************************************************************************
 * 四元数姿态更新
 ********************************************************************************************************************/

/**
 * @brief 使用角速度更新四元数（一阶龙格库塔）
 * @param q 当前四元数（输入/输出）
 * @param gyro_x X轴角速度 (rad/s)
 * @param gyro_y Y轴角速度 (rad/s)
 * @param gyro_z Z轴角速度 (rad/s)
 * @param dt 时间步长 (秒)
 * @note  这是所有算法的基础！陀螺仪积分更新四元数
 *        数学公式: q(t+dt) = q(t) + 0.5 * q(t) ⊗ [0, ωx, ωy, ωz] * dt
 */
void imu_quat_update_gyro(quaternion_t *q, float gyro_x, float gyro_y, float gyro_z, float dt);

/********************************************************************************************************************
 * 四元数与欧拉角转换（避免万向锁）
 ********************************************************************************************************************/

/**
 * @brief 四元数转欧拉角（ZYX顺序，避免万向锁）
 * @param q 输入四元数
 * @param pitch 输出俯仰角 (弧度, 范围: -π/2 ~ π/2)
 * @param roll 输出横滚角 (弧度, 范围: -π ~ π)
 * @param yaw 输出偏航角 (弧度, 范围: -π ~ π)
 * @note  使用ZYX欧拉角顺序，当pitch = ±90°时会有数值不稳定
 *        但不会出现万向锁（因为内部一直用四元数）
 */
void imu_quat_to_euler(const quaternion_t *q, float *pitch, float *roll, float *yaw);

/**
 * @brief 欧拉角转四元数
 * @param pitch 俯仰角 (弧度)
 * @param roll 横滚角 (弧度)
 * @param yaw 偏航角 (弧度)
 * @param q 输出四元数
 * @note  用于初始化或设定初始姿态
 */
void imu_euler_to_quat(float pitch, float roll, float yaw, quaternion_t *q);

/**
 * @brief 从加速度计提取Roll和Pitch角
 * @param acc_x X轴加速度 (g)
 * @param acc_y Y轴加速度 (g)
 * @param acc_z Z轴加速度 (g)
 * @param roll 输出横滚角 (弧度)
 * @param pitch 输出俯仰角 (弧度)
 * @note  假设只有重力加速度，无外力
 *        如果有外力加速度（如运动），此结果不可信！
 */
void imu_acc_to_rp(float acc_x, float acc_y, float acc_z, float *roll, float *pitch);

/**
 * @brief 从磁力计提取Yaw角（需要Roll和Pitch补偿）
 * @param mag_x X轴磁场强度
 * @param mag_y Y轴磁场强度
 * @param mag_z Z轴磁场强度
 * @param roll 当前横滚角 (弧度)
 * @param pitch 当前俯仰角 (弧度)
 * @param yaw 输出偏航角 (弧度)
 * @note  需要先用加速度计得到Roll/Pitch，然后补偿磁力计
 *        磁力计易受环境干扰，使用前必须校准！
 */
void imu_mag_to_yaw(float mag_x, float mag_y, float mag_z, float roll, float pitch, float *yaw);

/********************************************************************************************************************
 * 向量运算
 ********************************************************************************************************************/

/**
 * @brief 三维向量归一化
 * @param vx, vy, vz 输入向量分量（也作为输出）
 * @note  将向量归一化为单位向量（模长为1）
 */
void imu_vector_normalize(float *vx, float *vy, float *vz);

/**
 * @brief 三维向量模长
 * @param vx, vy, vz 向量分量
 * @return 模长 sqrt(vx^2 + vy^2 + vz^2)
 */
float imu_vector_norm(float vx, float vy, float vz);

/**
 * @brief 三维向量点积
 * @param v1x, v1y, v1z 向量1分量
 * @param v2x, v2y, v2z 向量2分量
 * @return 点积 v1·v2 = v1x*v2x + v1y*v2y + v1z*v2z
 */
float imu_vector_dot(float v1x, float v1y, float v1z, float v2x, float v2y, float v2z);

/**
 * @brief 三维向量叉积 v_result = v1 × v2
 * @param v1x, v1y, v1z 向量1分量
 * @param v2x, v2y, v2z 向量2分量
 * @param rx, ry, rz 输出结果向量分量
 * @note  叉积不满足交换律！v1×v2 = -(v2×v1)
 *        叉积结果垂直于v1和v2构成的平面
 */
void imu_vector_cross(float v1x, float v1y, float v1z,
                      float v2x, float v2y, float v2z,
                      float *rx, float *ry, float *rz);

/********************************************************************************************************************
 * 旋转相关运算
 ********************************************************************************************************************/

/**
 * @brief 使用四元数旋转向量
 * @param q 旋转四元数
 * @param vx, vy, vz 输入向量（也作为输出）
 * @note  v' = q * v * q*
 *        用于将向量从一个坐标系旋转到另一个坐标系
 */
void imu_quat_rotate_vector(const quaternion_t *q, float *vx, float *vy, float *vz);

/**
 * @brief 从旋转矩阵提取四元数
 * @param R 3x3旋转矩阵（行优先存储，9个元素）
 * @param q 输出四元数
 * @note  用于某些高级算法
 */
void imu_rotation_matrix_to_quat(const float R[9], quaternion_t *q);

/********************************************************************************************************************
 * 数学工具函数
 ********************************************************************************************************************/

/**
 * @brief 快速平方根倒数（Quake III算法优化）
 * @param x 输入值
 * @return 1/sqrt(x)
 * @note  比标准库的 1/sqrtf(x) 快约4倍
 *        用于向量和四元数归一化优化
 */
float imu_inv_sqrt(float x);

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
float imu_constrain(float value, float min, float max);

/**
 * @brief 弧度归一化到 [-π, π]
 * @param angle 输入角度（弧度）
 * @return 归一化后的角度
 */
float imu_normalize_angle_rad(float angle);

/**
 * @brief 角度归一化到 [-180°, 180°]
 * @param angle 输入角度（度）
 * @return 归一化后的角度
 */
float imu_normalize_angle_deg(float angle);

/********************************************************************************************************************
 * 低通滤波器（简单实用）
 ********************************************************************************************************************/

/**
 * @brief 一阶低通滤波器
 * @param input 当前输入值
 * @param output_prev 上次输出值
 * @param alpha 滤波系数 (0~1, 越小滤波越强)
 * @return 滤波后的值
 * @note  output = alpha * input + (1-alpha) * output_prev
 *        alpha = dt / (dt + RC)，其中RC是时间常数
 */
float imu_lowpass_filter(float input, float output_prev, float alpha);

#endif // _IMU_MATH_H

/********************************************************************************************************************
 * 老王的数学小课堂:
 *
 * 【四元数为什么能避免万向锁？】
 * 万向锁是欧拉角的问题，当pitch = ±90°时，roll和yaw会重合，丢失一个自由度。
 * 四元数是4维的，用4个数表示3D旋转，有一个冗余维度，所以不会出现奇异点！
 *
 * 【四元数乘法的物理意义】
 * q_result = q1 * q2 表示：先q2旋转，再q1旋转
 * 注意顺序！四元数乘法不满足交换律！
 *
 * 【为什么必须归一化？】
 * 理论上四元数模长应该始终为1，但浮点运算有误差，会累积导致模长偏离1。
 * 模长不为1的四元数表示的不是纯旋转，还包含缩放，会导致姿态错误！
 * 所以每次更新后必须归一化！艹！
 *
 * 【四元数更新公式推导】
 * 角速度ω = [ωx, ωy, ωz]，转换为四元数形式：ω_q = [0, ωx, ωy, ωz]
 * 四元数微分方程：dq/dt = 0.5 * q ⊗ ω_q
 * 一阶欧拉积分：q(t+dt) = q(t) + (dq/dt) * dt = q(t) + 0.5 * q(t) ⊗ ω_q * dt
 *
 ********************************************************************************************************************/
