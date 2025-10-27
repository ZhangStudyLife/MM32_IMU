# IMU 姿态解算系统 - 技术规划文档

> **作者**: 老王暴躁技术流
> **创建日期**: 2025-10-27
> **项目**: MM32F327X IMU963RA 九轴姿态解算系统
> **目标**: 构建模块化、可对比的高性能姿态解算框架

---

## 📋 项目概述

### 核心目标
构建一个**高度模块化**的IMU姿态解算系统，支持多种算法**并行运行**、**独立解算**、**实时对比**，为智能车、飞控、机器人等应用提供高精度姿态信息。

### 核心需求
1. ✅ **统一接口**: 所有算法模块共享统一的顶层API
2. ✅ **模块独立**: 各算法独立运行，互不干扰，共享原始数据
3. ✅ **框架一致**: 所有算法模块遵循统一的代码架构标准
4. ✅ **避免万向锁**: 所有算法必须使用四元数或旋转矩阵，禁止纯欧拉角积分
5. ✅ **实时高效**: 响应快速、计算高效，适配MM32F327X性能
6. ✅ **精度优先**: 对任意角度、剧烈运动都能准确响应

### 技术约束
- **硬件平台**: MM32F327X (120MHz Cortex-M3)
- **传感器**: IMU963RA (6轴ACC+GYRO + 3轴MAG)
- **输出格式**: Pitch/Roll/Yaw 欧拉角 (单位: 度)
- **采样率**: 100Hz - 1000Hz (可配置)
- **精度要求**: ±1° 静态精度, ±2° 动态精度
- **响应延迟**: < 10ms

---

## 🏗️ 系统架构设计

### 文件夹结构

```
project/user/IMU/
│
├── IMU_TODO.md              # 本文档
├── IMU.h                    # 统一顶层接口头文件
├── IMU.c                    # 统一顶层接口实现
│
├── common/                  # 公共模块
│   ├── imu_common.h         # 公共数据结构、宏定义
│   ├── imu_math.h           # 数学库（四元数、向量运算）
│   ├── imu_math.c
│   ├── imu_calibration.h    # 传感器校准模块
│   └── imu_calibration.c
│
├── Complementary/           # 互补滤波算法
│   ├── complementary.h
│   └── complementary.c
│
├── Madgwick/                # Madgwick算法
│   ├── madgwick.h
│   └── madgwick.c
│
├── Mahony/                  # Mahony算法
│   ├── mahony.h
│   └── mahony.c
│
├── EKF/                     # 扩展卡尔曼滤波
│   ├── ekf.h
│   ├── ekf.c
│   ├── ekf_matrix.h         # 矩阵运算库
│   └── ekf_matrix.c
│
├── Quaternion/              # 纯四元数积分（基线对比）
│   ├── quaternion.h
│   └── quaternion.c
│
└── test/                    # 测试与评估模块
    ├── imu_test.h
    ├── imu_test.c
    └── test_results/        # 测试结果数据
```

---

## 🧮 算法模块详解与对比

### 1️⃣ **互补滤波 (Complementary Filter)**

#### 原理
- **核心思想**: 融合加速度计（低频准确）和陀螺仪（高频准确）
- **数学模型**:
  ```
  angle = α × (angle_prev + gyro × dt) + (1-α) × acc_angle
  ```
- **权重系数**: α ≈ 0.96 - 0.98

#### 优点
- ✅ **计算量极小**: 只需简单加权，适合低功耗场景
- ✅ **实时性极佳**: 延迟 < 1ms
- ✅ **易于调试**: 只有1个参数α需要调整
- ✅ **鲁棒性好**: 对传感器噪声不敏感

#### 缺点
- ❌ **无磁力计融合**: 标准版本不支持Yaw角绝对参考
- ❌ **动态精度一般**: 剧烈运动时会有偏差
- ❌ **参数固定**: α值固定，无法自适应

#### 适用场景
- 平衡车、两轮机器人
- 简单姿态控制
- 对Yaw角精度要求不高的场合

#### 实现要点
```c
// 使用四元数表示姿态，避免万向锁
// 加速度计修正Roll/Pitch，陀螺仪积分
```

---

### 2️⃣ **Madgwick 算法**

#### 原理
- **核心思想**: 梯度下降法求解姿态四元数
- **数学模型**: 最小化加速度计测量值与四元数预测值的误差
- **创新点**: 自适应融合，根据加速度模值判断运动状态

#### 优点
- ✅ **精度高**: 静态精度 ±0.5°
- ✅ **支持九轴**: 可融合磁力计，提供Yaw绝对参考
- ✅ **计算量适中**: 约为EKF的1/5
- ✅ **开源成熟**: 有大量工程验证案例

#### 缺点
- ❌ **参数调节**: β参数需要根据采样率和运动特性调整
- ❌ **磁干扰敏感**: 磁力计易受环境干扰
- ❌ **动态响应慢**: 剧烈运动时收敛速度较慢

#### 适用场景
- 四旋翼无人机（推荐）
- VR/AR头盔
- 需要Yaw角绝对参考的场合

#### 实现要点
```c
// β参数推荐值：采样率100Hz时β=0.1, 1000Hz时β=0.01
// 需要检测加速度模值，剔除外力加速度影响
```

---

### 3️⃣ **Mahony 算法**

#### 原理
- **核心思想**: PI控制器修正陀螺仪零漂
- **数学模型**: 使用叉积计算姿态误差，PI控制器反馈修正
- **创新点**: 显式的PI补偿结构，物理意义明确

#### 优点
- ✅ **计算量小**: 比Madgwick稍快
- ✅ **动态响应快**: 对剧烈运动响应迅速
- ✅ **零漂补偿**: 显式补偿陀螺仪零漂
- ✅ **参数直观**: Kp/Ki参数物理意义明确

#### 缺点
- ❌ **参数多**: Kp和Ki需要同时调整
- ❌ **磁干扰敏感**: 与Madgwick类似
- ❌ **调参复杂**: 需要丰富的控制理论知识

#### 适用场景
- 飞行控制系统（PX4/APM默认算法）
- 需要快速响应的场合
- 对陀螺仪零漂敏感的应用

#### 实现要点
```c
// Kp推荐值：1.0 - 5.0
// Ki推荐值：0.0 - 0.1（可设为0）
// 检测加速度模值，动态调整Kp
```

---

### 4️⃣ **扩展卡尔曼滤波 (EKF)**

#### 原理
- **核心思想**: 最优状态估计，融合所有传感器信息
- **数学模型**:
  - 状态方程: X(k) = F×X(k-1) + B×U(k) + W(k)
  - 观测方程: Z(k) = H×X(k) + V(k)
  - 卡尔曼增益: K = P×H' / (H×P×H' + R)
- **状态向量**: [q0, q1, q2, q3, gyro_bias_x, gyro_bias_y, gyro_bias_z]

#### 优点
- ✅ **精度最高**: 理论上最优估计，静态精度 ±0.3°
- ✅ **自适应能力强**: 根据噪声特性自动调整融合权重
- ✅ **估计零漂**: 在线估计陀螺仪零漂
- ✅ **九轴融合**: 完美融合ACC+GYRO+MAG

#### 缺点
- ❌ **计算量巨大**: 约为Madgwick的5-10倍
- ❌ **内存占用大**: 需要大量矩阵存储空间
- ❌ **调参困难**: Q/R矩阵参数难以调整
- ❌ **数值稳定性**: 矩阵运算易出现数值问题

#### 适用场景
- 高精度导航系统
- 机器人SLAM
- MCU性能充足的场合（建议使用Cortex-M4/M7 + FPU）

#### 实现要点
```c
// MM32F327 (Cortex-M3无FPU) 运行EKF会比较吃力
// 需要优化矩阵运算，使用定点数或ARM CMSIS-DSP库
// 建议状态维度不超过10维
```

---

### 5️⃣ **纯四元数积分 (Baseline)**

#### 原理
- **核心思想**: 仅使用陀螺仪进行四元数积分
- **数学模型**:
  ```
  q(t+dt) = q(t) + 0.5 × q(t) ⊗ [0, ωx, ωy, ωz] × dt
  ```
- **用途**: 作为基线对比，评估其他算法的改进效果

#### 优点
- ✅ **计算量最小**: 仅四元数乘法和归一化
- ✅ **实时性最佳**: 延迟 < 0.5ms
- ✅ **无额外传感器**: 只需陀螺仪

#### 缺点
- ❌ **零漂严重**: 几分钟内会偏移数十度
- ❌ **无绝对参考**: 姿态会持续漂移
- ❌ **不能单独使用**: 仅作为对比基准

#### 适用场景
- 算法性能基准测试
- 短时间姿态跟踪（<10秒）
- 学习四元数运算的参考实现

---

## 📊 算法对比总结表

| 算法 | 计算量 | 精度 | 动态响应 | 零漂补偿 | 九轴融合 | 开发难度 | 推荐指数 |
|------|--------|------|----------|----------|----------|----------|----------|
| **互补滤波** | ⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ | ❌ | ⭐ | ⭐⭐⭐ |
| **Madgwick** | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ✅ | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Mahony** | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ✅ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| **EKF** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ✅ | ⭐⭐⭐⭐⭐ | ⭐⭐ |
| **四元数积分** | ⭐ | ⭐ | ⭐⭐⭐⭐⭐ | ❌ | ❌ | ⭐ | ⭐ (仅用于对比) |

**老王推荐**:
- **智能车**: Madgwick 或 Mahony
- **飞控**: Mahony 或 EKF
- **学习**: 先实现互补滤波和四元数积分

---

## 🔧 统一接口设计

### 核心数据结构

```c
/**
 * @brief IMU原始数据结构（9轴）
 * @note  所有算法共享同一份原始数据，只读访问
 */
typedef struct {
    // 加速度计原始数据 (单位: g)
    float acc_x, acc_y, acc_z;

    // 陀螺仪原始数据 (单位: rad/s)
    float gyro_x, gyro_y, gyro_z;

    // 磁力计原始数据 (单位: Gauss)
    float mag_x, mag_y, mag_z;

    // 时间戳 (单位: ms)
    uint32_t timestamp;

    // 数据有效标志
    uint8_t acc_valid;
    uint8_t gyro_valid;
    uint8_t mag_valid;
} imu_raw_data_t;

/**
 * @brief 姿态解算结果（欧拉角）
 * @note  输出格式统一为角度值
 */
typedef struct {
    float pitch;    // 俯仰角 (单位: 度, 范围: -90 ~ +90)
    float roll;     // 横滚角 (单位: 度, 范围: -180 ~ +180)
    float yaw;      // 偏航角 (单位: 度, 范围: -180 ~ +180)

    uint32_t timestamp;
    uint8_t valid;
} imu_attitude_t;

/**
 * @brief 四元数姿态表示（内部使用）
 * @note  所有算法内部必须使用四元数，避免万向锁
 */
typedef struct {
    float q0, q1, q2, q3;  // w, x, y, z
} quaternion_t;

/**
 * @brief 算法类型枚举
 */
typedef enum {
    IMU_ALGO_COMPLEMENTARY = 0,  // 互补滤波
    IMU_ALGO_MADGWICK,           // Madgwick算法
    IMU_ALGO_MAHONY,             // Mahony算法
    IMU_ALGO_EKF,                // 扩展卡尔曼滤波
    IMU_ALGO_QUATERNION,         // 纯四元数积分（基线）
    IMU_ALGO_COUNT               // 算法总数
} imu_algorithm_type_t;

/**
 * @brief 算法实例结构（每个算法独立维护）
 */
typedef struct {
    imu_algorithm_type_t type;       // 算法类型
    imu_attitude_t attitude;          // 解算结果
    quaternion_t quaternion;          // 内部四元数状态
    void *algo_specific_data;         // 算法特定数据（如EKF的协方差矩阵）

    // 算法回调函数指针（标准化接口）
    void (*init)(void *algo_data);
    void (*update)(void *algo_data, const imu_raw_data_t *raw_data, float dt);
    void (*get_attitude)(void *algo_data, imu_attitude_t *attitude);
    void (*reset)(void *algo_data);
} imu_algorithm_instance_t;
```

---

### 顶层API设计 (IMU.h)

```c
/**
 * @brief 初始化IMU系统
 * @param sample_rate: 采样率 (Hz)
 * @return 0-成功, 非0-失败
 */
int imu_system_init(uint32_t sample_rate);

/**
 * @brief 使能指定算法
 * @param algo_type: 算法类型
 * @return 0-成功, 非0-失败
 */
int imu_algorithm_enable(imu_algorithm_type_t algo_type);

/**
 * @brief 禁用指定算法
 * @param algo_type: 算法类型
 */
void imu_algorithm_disable(imu_algorithm_type_t algo_type);

/**
 * @brief 更新IMU数据（周期调用）
 * @note  读取IMU963RA原始数据，分发给所有已使能的算法
 */
void imu_update(void);

/**
 * @brief 获取指定算法的姿态结果
 * @param algo_type: 算法类型
 * @param attitude: 输出姿态数据
 * @return 0-成功, 非0-失败
 */
int imu_get_attitude(imu_algorithm_type_t algo_type, imu_attitude_t *attitude);

/**
 * @brief 重置指定算法
 * @param algo_type: 算法类型
 */
void imu_algorithm_reset(imu_algorithm_type_t algo_type);

/**
 * @brief 重置所有算法
 */
void imu_reset_all(void);

/**
 * @brief 获取原始数据（只读）
 * @return 原始数据指针
 */
const imu_raw_data_t* imu_get_raw_data(void);

/**
 * @brief 打印所有算法的姿态对比
 * @note  用于调试和算法评估
 */
void imu_print_all_attitudes(void);
```

---

## 🎯 各算法模块标准接口

**每个算法文件夹下必须实现以下4个函数**:

```c
/**
 * @brief 算法初始化
 * @param algo_data: 算法私有数据指针
 */
void xxx_init(void *algo_data);

/**
 * @brief 算法更新（每次采样调用）
 * @param algo_data: 算法私有数据指针
 * @param raw_data: IMU原始数据（只读）
 * @param dt: 时间步长 (秒)
 */
void xxx_update(void *algo_data, const imu_raw_data_t *raw_data, float dt);

/**
 * @brief 获取姿态结果
 * @param algo_data: 算法私有数据指针
 * @param attitude: 输出姿态数据
 */
void xxx_get_attitude(void *algo_data, imu_attitude_t *attitude);

/**
 * @brief 重置算法状态
 * @param algo_data: 算法私有数据指针
 */
void xxx_reset(void *algo_data);
```

**命名规范**: `xxx` 为算法名称小写，如 `madgwick_init`, `mahony_update`

---

## 🚀 实现步骤（分阶段）

### Phase 1: 基础框架搭建 (预计2天)
- [x] 创建文件夹结构
- [ ] 编写 `imu_common.h` (公共数据结构)
- [ ] 编写 `imu_math.h/.c` (四元数运算库)
- [ ] 编写 `IMU.h/.c` (顶层接口)
- [ ] 测试框架可编译通过

**验收标准**:
- 代码可编译
- 顶层接口函数框架完成
- 四元数基本运算测试通过

---

### Phase 2: 基线算法实现 (预计1天)
- [ ] 实现 `Quaternion/` (纯四元数积分)
- [ ] 验证四元数更新逻辑
- [ ] 验证欧拉角转换（避免万向锁）

**验收标准**:
- 短时间内姿态跟踪正确（10秒内）
- 能正确转换为Pitch/Roll/Yaw
- 无万向锁奇异点

---

### Phase 3: 互补滤波实现 (预计1天)
- [ ] 实现 `Complementary/`
- [ ] 加速度计Roll/Pitch提取
- [ ] 陀螺仪积分与加速度计融合
- [ ] 参数调优（α值）

**验收标准**:
- 静态精度 < ±1°
- 无明显漂移（5分钟测试）
- 动态响应快速

---

### Phase 4: Madgwick算法实现 (预计2天)
- [ ] 实现 `Madgwick/`
- [ ] 梯度下降法求解
- [ ] 加速度计融合
- [ ] 磁力计融合（可选）
- [ ] 参数调优（β值）

**验收标准**:
- 静态精度 < ±0.5°
- 支持九轴融合
- 动态响应良好

---

### Phase 5: Mahony算法实现 (预计2天)
- [ ] 实现 `Mahony/`
- [ ] PI控制器设计
- [ ] 零漂补偿
- [ ] 磁力计融合
- [ ] 参数调优（Kp/Ki）

**验收标准**:
- 静态精度 < ±0.5°
- 动态响应最快
- 零漂补偿有效

---

### Phase 6: EKF算法实现 (预计5天)
- [ ] 实现 `EKF/ekf_matrix.h/.c` (矩阵运算库)
- [ ] 实现 `EKF/ekf.h/.c`
- [ ] 状态方程建模
- [ ] 观测方程建模
- [ ] 卡尔曼增益计算
- [ ] Q/R矩阵参数调优
- [ ] 优化矩阵运算（ARM CMSIS-DSP）

**验收标准**:
- 静态精度 < ±0.3°
- 在线估计陀螺仪零漂
- 运行频率 > 100Hz

---

### Phase 7: 测试与评估 (预计3天)
- [ ] 实现 `test/imu_test.c`
- [ ] 静态精度测试
- [ ] 动态精度测试（快速旋转）
- [ ] 万向锁测试（90°俯仰）
- [ ] 长时间稳定性测试（30分钟）
- [ ] 性能测试（CPU占用、响应延迟）
- [ ] 生成测试报告

---

### Phase 8: 优化与文档 (预计2天)
- [ ] 代码性能优化
- [ ] 参数自动调优
- [ ] 编写使用文档
- [ ] 编写API文档
- [ ] 代码注释完善

---

## 🧪 测试计划

### 1. 静态精度测试
- **方法**: IMU静止放置，记录5分钟姿态数据
- **评估**: 标准差、最大偏差
- **通过标准**: 标准差 < 0.5°

### 2. 动态响应测试
- **方法**: 手动快速旋转IMU（90°/秒）
- **评估**: 延迟、超调量、稳定时间
- **通过标准**: 延迟 < 50ms, 超调 < 5%

### 3. 万向锁测试
- **方法**: Pitch角旋转至±90°附近
- **评估**: Roll/Yaw是否出现跳变
- **通过标准**: 无奇异点、无跳变

### 4. 长时间稳定性测试
- **方法**: 静止放置30分钟，记录Yaw漂移
- **评估**: Yaw漂移量
- **通过标准**: 漂移 < 5° (互补滤波允许 < 10°)

### 5. CPU占用测试
- **方法**: 使用定时器测量算法执行时间
- **评估**: 平均/最大执行时间
- **通过标准**:
  - 互补滤波: < 0.1ms
  - Madgwick/Mahony: < 0.5ms
  - EKF: < 2ms

---

## ⚠️ 关键技术要点

### 1. 四元数归一化
**必须在每次更新后归一化四元数，否则会数值发散！**
```c
float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
```

### 2. 欧拉角转换（避免万向锁）
使用以下顺序避免奇异点：
```c
// ZYX顺序 (Yaw-Pitch-Roll)
pitch = asinf(2*(q0*q2 - q3*q1));
roll  = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
yaw   = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
```

### 3. 加速度计有效性检测
外力加速度会干扰姿态估计，需检测：
```c
float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
if (fabsf(acc_norm - 1.0f) > 0.4f) {
    // 外力加速度，降低加速度计权重
}
```

### 4. 磁力计软硬铁校准
磁力计易受干扰，需离线校准：
```c
// 硬铁偏移补偿
mag_x -= hard_iron_offset_x;
// 软铁椭球拟合
mag_corrected = soft_iron_matrix * mag_raw;
```

### 5. 采样率与算法参数关系
| 采样率 | Madgwick β | Mahony Kp | 互补滤波 α |
|--------|-----------|----------|------------|
| 100Hz  | 0.1       | 1.0      | 0.96       |
| 500Hz  | 0.033     | 2.5      | 0.98       |
| 1000Hz | 0.01      | 5.0      | 0.99       |

---

## 📚 参考文献与资源

1. **Madgwick算法原文**:
   *An efficient orientation filter for inertial and inertial/magnetic sensor arrays* (2010)

2. **Mahony算法原文**:
   *Nonlinear Complementary Filters on the Special Orthogonal Group* (2008)

3. **四元数教程**:
   https://www.3dgep.com/understanding-quaternions/

4. **EKF姿态估计**:
   https://www.vectornav.com/resources/inertial-navigation-primer

5. **开源参考实现**:
   - PX4 Autopilot: https://github.com/PX4/PX4-Autopilot
   - Madgwick官方: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

---

## 🎖️ 成功标准

✅ **功能完整性**:
- [ ] 所有5种算法正常运行
- [ ] 统一接口工作正常
- [ ] 可动态切换算法

✅ **性能指标**:
- [ ] 静态精度 < ±1°
- [ ] 动态精度 < ±2°
- [ ] 响应延迟 < 10ms
- [ ] CPU占用 < 10% @ 100Hz采样

✅ **代码质量**:
- [ ] 代码结构清晰，模块独立
- [ ] 注释完整，符合Doxygen规范
- [ ] 无内存泄漏，无数值溢出
- [ ] 通过所有单元测试

✅ **可维护性**:
- [ ] 参数配置化，易于调整
- [ ] 接口统一，易于扩展新算法
- [ ] 文档齐全，易于理解

---

## 💪 老王的话

艹！这个IMU姿态解算系统设计得够专业吧！

**核心设计思想**:
1. **绝对的模块化** - 各算法独立运行，互不干扰
2. **统一的接口** - 所有算法遵循相同的API规范
3. **高效的架构** - 原始数据共享，避免重复读取
4. **科学的对比** - 同一数据源，公平比较各算法性能

**实现建议**:
- **先易后难**: 四元数积分 → 互补滤波 → Madgwick/Mahony → EKF
- **逐步验证**: 每个算法实现后立即测试，不要积累问题
- **参数调优**: 静态环境先调准，再测试动态性能
- **性能优化**: 最后阶段再优化，过早优化是万恶之源

**避坑指南**:
1. ⚠️ 四元数必须归一化，否则1小时后姿态就乱套了
2. ⚠️ 欧拉角转换要注意顺序，避免万向锁
3. ⚠️ 加速度计在运动时不可信，需要检测加速度模值
4. ⚠️ 磁力计易受干扰，室内测试要远离电机、铁制品
5. ⚠️ EKF在MM32F327上跑可能会吃力，优先用Madgwick/Mahony

**预计总工作量**: 15-20天（全职开发）

这个TODO写得够详细了吧！接下来就是撸起袖子开干！💪

---

**文档版本**: v1.0
**最后更新**: 2025-10-27
**作者**: 老王暴躁技术流
