/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 四元数数学运算库实现
*
* 文件名称          imu_math.c
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-27
*
* 功能说明:
*   实现imu_math.h中声明的所有数学函数
*
********************************************************************************************************************/

#include "imu_math.h"
#include <math.h>
#include <string.h>

/********************************************************************************************************************
 * 四元数基本运算实现
 ********************************************************************************************************************/

/**
 * @brief 初始化四元数为单位四元数 [1, 0, 0, 0]
 */
void imu_quat_identity(quaternion_t *q)
{
    q->q0 = 1.0f;
    q->q1 = 0.0f;
    q->q2 = 0.0f;
    q->q3 = 0.0f;
}

/**
 * @brief 四元数归一化（艹！超级重要！）
 */
void imu_quat_normalize(quaternion_t *q)
{
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);

    // 防止除以0（虽然正常情况不会发生）
    if (norm < 1e-6f)
    {
        // 如果四元数接近零，重置为单位四元数
        imu_quat_identity(q);
        return;
    }

    // 归一化
    float inv_norm = 1.0f / norm;
    q->q0 *= inv_norm;
    q->q1 *= inv_norm;
    q->q2 *= inv_norm;
    q->q3 *= inv_norm;
}

/**
 * @brief 四元数共轭
 */
void imu_quat_conjugate(const quaternion_t *q, quaternion_t *q_conj)
{
    q_conj->q0 =  q->q0;
    q_conj->q1 = -q->q1;
    q_conj->q2 = -q->q2;
    q_conj->q3 = -q->q3;
}

/**
 * @brief 四元数乘法 q_result = q1 * q2
 * @note  数学推导：Hamilton乘积
 *        (w1 + x1*i + y1*j + z1*k) * (w2 + x2*i + y2*j + z2*k)
 */
void imu_quat_multiply(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *q_result)
{
    // 使用临时变量，以支持 q_result 与 q1 或 q2 相同的情况
    float w = q1->q0 * q2->q0 - q1->q1 * q2->q1 - q1->q2 * q2->q2 - q1->q3 * q2->q3;
    float x = q1->q0 * q2->q1 + q1->q1 * q2->q0 + q1->q2 * q2->q3 - q1->q3 * q2->q2;
    float y = q1->q0 * q2->q2 - q1->q1 * q2->q3 + q1->q2 * q2->q0 + q1->q3 * q2->q1;
    float z = q1->q0 * q2->q3 + q1->q1 * q2->q2 - q1->q2 * q2->q1 + q1->q3 * q2->q0;

    q_result->q0 = w;
    q_result->q1 = x;
    q_result->q2 = y;
    q_result->q3 = z;
}

/**
 * @brief 四元数与标量相加
 */
void imu_quat_add_scalar(const quaternion_t *q, float scalar, quaternion_t *q_result)
{
    q_result->q0 = q->q0 + scalar;
    q_result->q1 = q->q1;
    q_result->q2 = q->q2;
    q_result->q3 = q->q3;
}

/**
 * @brief 四元数与标量相乘
 */
void imu_quat_multiply_scalar(const quaternion_t *q, float scalar, quaternion_t *q_result)
{
    q_result->q0 = q->q0 * scalar;
    q_result->q1 = q->q1 * scalar;
    q_result->q2 = q->q2 * scalar;
    q_result->q3 = q->q3 * scalar;
}

/**
 * @brief 四元数模长
 */
float imu_quat_norm(const quaternion_t *q)
{
    return sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
}

/********************************************************************************************************************
 * 四元数姿态更新实现
 ********************************************************************************************************************/

/**
 * @brief 使用角速度更新四元数（一阶龙格库塔）
 * @note  核心公式：q(t+dt) = q(t) + 0.5 * q(t) ⊗ [0, ωx, ωy, ωz] * dt
 */
void imu_quat_update_gyro(quaternion_t *q, float gyro_x, float gyro_y, float gyro_z, float dt)
{
    // 构造角速度四元数 [0, ωx, ωy, ωz]
    quaternion_t q_omega;
    q_omega.q0 = 0.0f;
    q_omega.q1 = gyro_x;
    q_omega.q2 = gyro_y;
    q_omega.q3 = gyro_z;

    // 计算四元数导数：dq/dt = 0.5 * q ⊗ q_omega
    quaternion_t q_dot;
    imu_quat_multiply(q, &q_omega, &q_dot);

    // 乘以0.5
    q_dot.q0 *= 0.5f;
    q_dot.q1 *= 0.5f;
    q_dot.q2 *= 0.5f;
    q_dot.q3 *= 0.5f;

    // 一阶欧拉积分：q = q + dq/dt * dt
    q->q0 += q_dot.q0 * dt;
    q->q1 += q_dot.q1 * dt;
    q->q2 += q_dot.q2 * dt;
    q->q3 += q_dot.q3 * dt;

    // 艹！必须归一化！否则误差累积会导致姿态发散！
    imu_quat_normalize(q);
}

/********************************************************************************************************************
 * 四元数与欧拉角转换实现（避免万向锁）
 ********************************************************************************************************************/

/**
 * @brief 四元数转欧拉角（ZYX顺序）
 * @note  使用ZYX欧拉角顺序（Yaw-Pitch-Roll）
 *        参考：https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
void imu_quat_to_euler(const quaternion_t *q, float *pitch, float *roll, float *yaw)
{
    float q0 = q->q0;
    float q1 = q->q1;
    float q2 = q->q2;
    float q3 = q->q3;

    // Pitch (俯仰角) - 绕Y轴旋转
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    // 限制范围避免asin溢出
    sin_pitch = imu_constrain(sin_pitch, -1.0f, 1.0f);
    *pitch = asinf(sin_pitch);

    // Roll (横滚角) - 绕X轴旋转
    float sin_roll_cos_pitch = 2.0f * (q0 * q1 + q2 * q3);
    float cos_roll_cos_pitch = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sin_roll_cos_pitch, cos_roll_cos_pitch);

    // Yaw (偏航角) - 绕Z轴旋转
    float sin_yaw_cos_pitch = 2.0f * (q0 * q3 + q1 * q2);
    float cos_yaw_cos_pitch = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(sin_yaw_cos_pitch, cos_yaw_cos_pitch);
}

/**
 * @brief 欧拉角转四元数
 */
void imu_euler_to_quat(float pitch, float roll, float yaw, quaternion_t *q)
{
    float cos_pitch = cosf(pitch * 0.5f);
    float sin_pitch = sinf(pitch * 0.5f);
    float cos_roll  = cosf(roll  * 0.5f);
    float sin_roll  = sinf(roll  * 0.5f);
    float cos_yaw   = cosf(yaw   * 0.5f);
    float sin_yaw   = sinf(yaw   * 0.5f);

    q->q0 = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
    q->q1 = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
    q->q2 = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    q->q3 = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
}

/**
 * @brief 从加速度计提取Roll和Pitch角
 * @note  假设只有重力加速度，无外力
 */
void imu_acc_to_rp(float acc_x, float acc_y, float acc_z, float *roll, float *pitch)
{
    // Roll = atan2(ay, az)
    *roll = atan2f(acc_y, acc_z);

    // Pitch = atan2(-ax, sqrt(ay^2 + az^2))
    *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));
}

/**
 * @brief 从磁力计提取Yaw角（需要Roll和Pitch补偿）
 * @note  将磁力计数据旋转到水平面，然后计算Yaw
 */
void imu_mag_to_yaw(float mag_x, float mag_y, float mag_z, float roll, float pitch, float *yaw)
{
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);
    float cos_roll  = cosf(roll);
    float sin_roll  = sinf(roll);

    // 将磁力计数据旋转到水平面
    float mag_x_h = mag_x * cos_pitch + mag_z * sin_pitch;
    float mag_y_h = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;

    // 计算Yaw角
    *yaw = atan2f(-mag_y_h, mag_x_h);
}

/********************************************************************************************************************
 * 向量运算实现
 ********************************************************************************************************************/

/**
 * @brief 三维向量归一化
 */
void imu_vector_normalize(float *vx, float *vy, float *vz)
{
    float norm = sqrtf((*vx) * (*vx) + (*vy) * (*vy) + (*vz) * (*vz));

    if (norm < 1e-6f)
    {
        // 接近零向量，无法归一化
        *vx = 0.0f;
        *vy = 0.0f;
        *vz = 0.0f;
        return;
    }

    float inv_norm = 1.0f / norm;
    *vx *= inv_norm;
    *vy *= inv_norm;
    *vz *= inv_norm;
}

/**
 * @brief 三维向量模长
 */
float imu_vector_norm(float vx, float vy, float vz)
{
    return sqrtf(vx * vx + vy * vy + vz * vz);
}

/**
 * @brief 三维向量点积
 */
float imu_vector_dot(float v1x, float v1y, float v1z, float v2x, float v2y, float v2z)
{
    return v1x * v2x + v1y * v2y + v1z * v2z;
}

/**
 * @brief 三维向量叉积 v_result = v1 × v2
 */
void imu_vector_cross(float v1x, float v1y, float v1z,
                      float v2x, float v2y, float v2z,
                      float *rx, float *ry, float *rz)
{
    *rx = v1y * v2z - v1z * v2y;
    *ry = v1z * v2x - v1x * v2z;
    *rz = v1x * v2y - v1y * v2x;
}

/********************************************************************************************************************
 * 旋转相关运算实现
 ********************************************************************************************************************/

/**
 * @brief 使用四元数旋转向量
 * @note  v' = q * v * q*
 *        其中v表示为纯四元数 [0, vx, vy, vz]
 */
void imu_quat_rotate_vector(const quaternion_t *q, float *vx, float *vy, float *vz)
{
    // 将向量表示为纯四元数
    quaternion_t v_quat;
    v_quat.q0 = 0.0f;
    v_quat.q1 = *vx;
    v_quat.q2 = *vy;
    v_quat.q3 = *vz;

    // 计算q的共轭
    quaternion_t q_conj;
    imu_quat_conjugate(q, &q_conj);

    // 计算 q * v
    quaternion_t temp;
    imu_quat_multiply(q, &v_quat, &temp);

    // 计算 (q * v) * q*
    quaternion_t result;
    imu_quat_multiply(&temp, &q_conj, &result);

    // 提取结果向量（实部应该为0）
    *vx = result.q1;
    *vy = result.q2;
    *vz = result.q3;
}

/**
 * @brief 从旋转矩阵提取四元数（Shepperd方法）
 * @note  旋转矩阵R是行优先存储的3x3矩阵
 */
void imu_rotation_matrix_to_quat(const float R[9], quaternion_t *q)
{
    float trace = R[0] + R[4] + R[8];

    if (trace > 0.0f)
    {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        q->q0 = 0.25f * s;
        q->q1 = (R[7] - R[5]) / s;
        q->q2 = (R[2] - R[6]) / s;
        q->q3 = (R[3] - R[1]) / s;
    }
    else if ((R[0] > R[4]) && (R[0] > R[8]))
    {
        float s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        q->q0 = (R[7] - R[5]) / s;
        q->q1 = 0.25f * s;
        q->q2 = (R[1] + R[3]) / s;
        q->q3 = (R[2] + R[6]) / s;
    }
    else if (R[4] > R[8])
    {
        float s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        q->q0 = (R[2] - R[6]) / s;
        q->q1 = (R[1] + R[3]) / s;
        q->q2 = 0.25f * s;
        q->q3 = (R[5] + R[7]) / s;
    }
    else
    {
        float s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        q->q0 = (R[3] - R[1]) / s;
        q->q1 = (R[2] + R[6]) / s;
        q->q2 = (R[5] + R[7]) / s;
        q->q3 = 0.25f * s;
    }

    imu_quat_normalize(q);
}

/********************************************************************************************************************
 * 数学工具函数实现
 ********************************************************************************************************************/

/**
 * @brief 快速平方根倒数（Quake III算法）
 * @note  比标准库快，但精度稍低（对姿态解算够用）
 */
float imu_inv_sqrt(float x)
{
    // 标准库实现（安全起见，暂时用这个）
    // 如果需要极致性能，可以用Quake III的魔法数字版本
    return 1.0f / sqrtf(x);

    // Quake III快速实现（注释掉，需要时启用）
    /*
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
    */
}

/**
 * @brief 限幅函数
 */
float imu_constrain(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 弧度归一化到 [-π, π]
 */
float imu_normalize_angle_rad(float angle)
{
    while (angle > IMU_PI)  angle -= 2.0f * IMU_PI;
    while (angle < -IMU_PI) angle += 2.0f * IMU_PI;
    return angle;
}

/**
 * @brief 角度归一化到 [-180°, 180°]
 */
float imu_normalize_angle_deg(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 一阶低通滤波器
 */
float imu_lowpass_filter(float input, float output_prev, float alpha)
{
    return alpha * input + (1.0f - alpha) * output_prev;
}

/********************************************************************************************************************
 * 老王的实现笔记:
 *
 * 1. 四元数乘法的实现用了临时变量，这样可以支持 q_result = q1 * q1 这种情况
 *
 * 2. 归一化函数加了防护，如果四元数接近零就重置为单位四元数
 *
 * 3. 欧拉角转换时对sin_pitch做了限幅，避免asin溢出
 *
 * 4. 向量旋转用的是 v' = q * v * q* 公式，标准做法
 *
 * 5. 快速平方根倒数暂时用标准库，需要时可以启用Quake III版本
 *
 * 艹！这些数学函数写起来还tm挺费劲的，但是必须保证正确性！
 *
 ********************************************************************************************************************/
