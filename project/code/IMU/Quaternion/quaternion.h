/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 纯四元数积分算法（基线）
*
* 文件名称          quaternion.h
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-27
*
* 功能说明:
*   1. 实现纯四元数积分算法（Baseline）
*   2. 仅使用陀螺仪数据进行四元数积分
*   3. 无任何修正，会产生零漂
*   4. 用于基线对比和验证框架
*
* 算法原理:
*   - 使用陀螺仪角速度直接积分更新四元数
*   - 数学公式: q(t+dt) = q(t) + 0.5 * q(t) ⊗ [0, ωx, ωy, ωz] * dt
*   - 优点: 计算量极小，实时性最佳
*   - 缺点: 陀螺仪零漂会累积，几分钟内偏移数十度
*
********************************************************************************************************************/

#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "../common/imu_common.h"
#include "../common/imu_math.h"

typedef struct
{
    uint8_t reserved;  // 保留字节，保持接口一致性
} quaternion_algo_data_t;

// 标准回调函数
void quaternion_init(void *algo_data);
void quaternion_update(void *algo_data, const imu_raw_data_t *raw_data, float dt);
void quaternion_get_attitude(void *algo_data, imu_attitude_t *attitude);
void quaternion_reset(void *algo_data);

// 注册函数
int quaternion_register(imu_algorithm_instance_t *instance);

#endif // _QUATERNION_H
