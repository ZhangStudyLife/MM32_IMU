# IMU963RA 飞控配置入门指南

## 📋 目录
1. [传感器概述](#传感器概述)
2. [核心寄存器详解](#核心寄存器详解)
3. [采样率配置（输出数据率ODR）](#采样率配置)
4. [量程配置](#量程配置)
5. [滤波器配置](#滤波器配置)
6. [飞控优化推荐配置](#飞控优化推荐配置)
7. [数据读取与转换](#数据读取与转换)
8. [实战配置案例](#实战配置案例)

---

## 传感器概述

### 硬件构成
IMU963RA 是一款**九轴惯性测量单元**（9-DOF IMU），由以下部件组成：

| 传感器类型 | 芯片型号 | 测量维度 | 用途 |
|-----------|---------|---------|------|
| **加速度计** (Accelerometer) | LSM6DSM | 3轴 (X/Y/Z) | 测量线性加速度，计算姿态角（俯仰Pitch、横滚Roll） |
| **陀螺仪** (Gyroscope) | LSM6DSM | 3轴 (X/Y/Z) | 测量角速度，提供高频姿态变化 |
| **磁力计** (Magnetometer) | MMC5983MA | 3轴 (X/Y/Z) | 测量地磁场，计算航向角（偏航Yaw） |

### 通信接口
- **主控↔加速度计/陀螺仪**：SPI（高速，默认10MHz）或 I2C
- **加速度计↔磁力计**：I2C Sensor Hub（自动轮询）

---

## 核心寄存器详解

### 1️⃣ **CTRL1_XL (0x10)** - 加速度计控制寄存器

该寄存器控制加速度计的**输出数据率（ODR）**和**量程**。

#### 寄存器位定义：
```
Bit 7-4: ODR_XL[3:0] - 输出数据率（采样频率）
Bit 3-2: FS_XL[1:0]  - 量程选择
Bit 1-0: BW_XL[1:0]  - 抗混叠滤波器带宽
```

#### 配置值与效果：

| 配置值 | ODR (Hz) | 量程 (±g) | 灵敏度 (LSB/g) | 飞控推荐 |
|-------|----------|-----------|---------------|---------|
| `0x30` | 104 | 2  | 16393 | ❌ 太慢 |
| `0x40` | 208 | 2  | 16393 | ⚠️ 可用 |
| `0x50` | 416 | 2  | 16393 | ⚠️ 可用 |
| `0x60` | 833 | 2  | 16393 | ✅ **推荐** |
| `0x70` | 1666 | 2  | 16393 | ✅ 高性能 |
| `0x80` | 3333 | 2  | 16393 | ✅ 竞速机 |
| `0x38` | 104 | 4  | 8197  | ❌ 太慢 |
| `0x68` | 833 | 4  | 8197  | ✅ 平衡选择 |
| `0x3C` | 104 | 8  | 4098  | ❌ 太慢 |
| `0x6C` | 833 | 8  | 4098  | ✅ **默认配置** |
| `0x7C` | 1666 | 8  | 4098  | ✅ 推荐 |
| `0x34` | 104 | 16 | 2049  | ❌ 不推荐（精度低）|

**飞控关键点**：
- **ODR 必须 ≥400Hz**，推荐 800-1000Hz（匹配PID控制周期）
- **量程选择**：±8g 适合大部分四旋翼，±16g 用于暴力机动

---

### 2️⃣ **CTRL2_G (0x11)** - 陀螺仪控制寄存器

#### 寄存器位定义：
```
Bit 7-4: ODR_G[3:0] - 输出数据率
Bit 3-1: FS_G[2:0]  - 量程选择
```

#### 配置值与效果：

| 配置值 | ODR (Hz) | 量程 (±°/s) | 灵敏度 (LSB/dps) | 飞控推荐 | 应用场景 |
|-------|----------|-------------|-----------------|---------|---------|
| `0x50` | 208 | 250  | 114.3 | ❌ | 太慢 |
| `0x60` | 416 | 250  | 114.3 | ⚠️ | 慢速机 |
| `0x70` | 833 | 250  | 114.3 | ❌ | 量程太小 |
| `0x54` | 208 | 500  | 57.1  | ❌ | 太慢 |
| `0x64` | 416 | 500  | 57.1  | ⚠️ | 慢速机 |
| `0x74` | 833 | 500  | 57.1  | ⚠️ | 量程偏小 |
| `0x58` | 208 | 1000 | 28.6  | ❌ | 太慢 |
| `0x68` | 416 | 1000 | 28.6  | ⚠️ | 可用 |
| `0x78` | 833 | 1000 | 28.6  | ✅ | 平稳飞行 |
| `0x5C` | 208 | 2000 | 14.3  | ❌ | 太慢 |
| `0x6C` | 416 | 2000 | 14.3  | ⚠️ | 可用 |
| `0x7C` | 833 | 2000 | 14.3  | ✅ | **推荐配置** |
| `0x8C` | 1666 | 2000 | 14.3  | ✅ | 高响应 |

**飞控关键点**：
- **ODR 推荐 ≥833Hz**（与加速度计同步）
- **量程选择**：
  - ±1000°/s：平稳飞行、航拍
  - ±2000°/s：穿越机、特技飞行（默认）
  - ±4000°/s：极限暴力机动（仅专业竞速）

---

### 3️⃣ **CTRL3_C (0x12)** - 通用控制寄存器

```c
// 当前配置：0x44
Bit 6: BDU = 1      // 块数据更新（防止读取中途数据更新）
Bit 2: IF_INC = 1   // 自动增量地址（连续读取）
```

**重要位说明**：
- **Bit 0 (SW_RESET)**: 软件复位（写1触发）
- **Bit 6 (BDU)**: 启用后，高低字节同步更新（避免数据撕裂）

---

### 4️⃣ **CTRL4_C (0x13)** - 控制寄存器4

```c
// 当前配置：0x02
Bit 1: LPF1_SEL_G = 1  // 启用陀螺仪数字低通滤波器1
```

**飞控关键**：必须启用，否则陀螺仪数据噪声大。

---

### 5️⃣ **CTRL6_C (0x15)** - 性能与滤波器配置

```c
// 当前配置：0x00
Bit 7-5: TRIG_MODE[2:0] = 000  // 触发模式
Bit 4:   XL_HM_MODE = 0        // 加速度计高性能模式
Bit 3-0: FTYPE[3:0] = 0000     // 陀螺仪LPF1截止频率
```

#### 陀螺仪低通滤波器截止频率（FTYPE设置）：

| FTYPE[3:0] | 截止频率 (ODR=833Hz) | 延迟 | 飞控推荐 |
|-----------|---------------------|------|---------|
| `0000` | 324 Hz (ODR/2.5) | 最小 | ✅ **默认（低延迟）** |
| `0001` | 232 Hz (ODR/3.6) | 低 | ✅ 推荐 |
| `0010` | 173 Hz (ODR/4.8) | 中 | ⚠️ 较平滑 |
| `0011` | 138 Hz (ODR/6.0) | 高 | ❌ 延迟明显 |

**配置建议**：
```c
// 推荐飞控配置：
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00);  // 高性能 + 324Hz截止
// 或
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x01);  // 高性能 + 232Hz截止
```

---

### 6️⃣ **CTRL7_G (0x16)** - 陀螺仪配置寄存器

```c
// 当前配置：0x00
Bit 7: G_HM_MODE = 0      // 陀螺仪高性能模式（0=启用）
Bit 6: HP_EN_G = 0        // 高通滤波器禁用
Bit 5-4: HPCF_G[1:0] = 00 // 高通滤波器截止频率
```

**飞控要点**：
- **不要启用高通滤波器**（会过滤低频运动，导致积分漂移）
- 保持 `0x00`（高性能模式 + 无高通滤波）

---

### 7️⃣ **CTRL8_XL (0x17)** - 加速度计滤波器配置

```c
Bit 7: LPF2_XL_EN       // 启用二级低通滤波器（可选）
Bit 6: HPCF_XL          // 高通滤波器截止频率
Bit 5: HP_REF_MODE_XL   // 高通滤波参考模式
Bit 2-0: FDS/BW_SEL[2:0] // 滤波器选择
```

**飞控推荐**：
```c
// 不启用二级滤波器（避免延迟）
imu963ra_write_acc_gyro_register(IMU963RA_CTRL8_XL, 0x00);
```

---

## 采样率配置

### 🎯 采样率（ODR）对飞控的影响

| 性能指标 | 100Hz | 400Hz | 833Hz | 1666Hz |
|---------|-------|-------|-------|--------|
| **响应延迟** | 10ms | 2.5ms | 1.2ms | 0.6ms |
| **PID控制周期** | ❌ | ⚠️ | ✅ | ✅ |
| **姿态更新速度** | 慢 | 中等 | 快 | 极快 |
| **CPU负载** | 低 | 中 | 中高 | 高 |
| **功耗** | 低 | 中 | 高 | 极高 |

### 🚀 飞控推荐设置

#### **标准四旋翼（航拍/平稳飞行）**
```c
// 加速度计：833Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x6C); // 833Hz, ±8g

// 陀螺仪：833Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x7C);  // 833Hz, ±2000°/s
```

#### **高性能竞速机**
```c
// 加速度计：1666Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x7C); // 1666Hz, ±8g

// 陀螺仪：1666Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x8C);  // 1666Hz, ±2000°/s
```

---

## 量程配置

### 📊 量程选择原则

#### **加速度计量程**
| 量程 | 灵敏度 | 噪声 | 适用场景 |
|-----|-------|------|---------|
| ±2g | 16393 LSB/g | 最低 | ❌ 飞控不适用（易饱和） |
| ±4g | 8197 LSB/g | 低 | ⚠️ 慢速机、固定翼 |
| ±8g | 4098 LSB/g | 中 | ✅ **通用推荐** |
| ±16g | 2049 LSB/g | 高 | ⚠️ 暴力机动（精度损失） |

**飞控选择**：
- **多旋翼**：±8g（平衡精度与动态范围）
- **固定翼**：±4g（加速度变化小）
- **暴力穿越机**：±16g（高G值机动）

#### **陀螺仪量程**
| 量程 | 灵敏度 | 对应角加速度 | 适用场景 |
|-----|-------|------------|---------|
| ±125°/s | 228.6 LSB/dps | 低 | ❌ 飞控不适用 |
| ±250°/s | 114.3 LSB/dps | 低 | ❌ 固定翼慢速 |
| ±500°/s | 57.1 LSB/dps | 中低 | ⚠️ 航拍机 |
| ±1000°/s | 28.6 LSB/dps | 中 | ✅ 平稳飞行 |
| ±2000°/s | 14.3 LSB/dps | 高 | ✅ **通用推荐** |
| ±4000°/s | 7.1 LSB/dps | 极高 | ⚠️ 竞速机（精度损失）|

**计算示例**：
```
翻滚一圈（360°）所需时间：
- ±1000°/s: 0.36秒（慢）
- ±2000°/s: 0.18秒（快）
- ±4000°/s: 0.09秒（极快）
```

---

## 滤波器配置

### 🎛️ 滤波器类型与作用

IMU963RA 提供**两级滤波器**：

```
原始数据 → LPF1 (一级低通) → LPF2 (二级低通) → 输出数据
                                ↓
                           (可选，增加延迟)
```

### 1️⃣ 一级低通滤波器（LPF1）- **必须启用**

#### 陀螺仪 LPF1 配置（CTRL6_C.FTYPE）

| 截止频率 | 噪声抑制 | 相位延迟 | 飞控推荐 |
|---------|---------|---------|---------|
| 324 Hz | 弱 | 0.3ms | ✅ **低延迟优先** |
| 232 Hz | 中 | 0.5ms | ✅ **平衡选择** |
| 173 Hz | 强 | 0.8ms | ⚠️ 高噪声环境 |
| 138 Hz | 极强 | 1.2ms | ❌ 延迟过大 |

**配置代码**：
```c
// 方案1：最低延迟（默认）
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00); // 324Hz

// 方案2：平衡模式（推荐）
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x01); // 232Hz
```

#### 加速度计 LPF1
- 由 CTRL1_XL 的 BW_XL 位控制
- 默认配置即可（400Hz 截止频率）

### 2️⃣ 二级低通滤波器（LPF2）- **飞控不推荐**

```c
// 不启用 LPF2（避免额外延迟）
imu963ra_write_acc_gyro_register(IMU963RA_CTRL8_XL, 0x00);
```

**原因**：
- 增加 2-5ms 延迟
- 对于 PID 控制，延迟 > 噪声影响
- 软件滤波器更灵活

---

## 飞控优化推荐配置

### 🏆 最佳配置方案

#### **方案1：通用多旋翼配置**（推荐）
```c
void imu963ra_flight_controller_init(void)
{
    // 复位设备
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x01);
    system_delay_ms(10);

    // 加速度计：833Hz, ±8g
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x6C);

    // 陀螺仪：833Hz, ±2000°/s
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x7C);

    // 启用块数据更新 + 陀螺仪LPF
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x44);

    // 启用陀螺仪LPF1
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL4_C, 0x02);

    // 加速度计与陀螺仪四舍五入
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL5_C, 0x00);

    // 陀螺仪LPF1截止频率：232Hz（平衡模式）
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x01);

    // 陀螺仪高性能 + 无高通滤波
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL7_G, 0x00);

    // 加速度计无二级滤波
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL8_XL, 0x00);

    // 关闭I3C接口
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL9_XL, 0x01);

    // 启用数据就绪中断（INT1引脚）
    imu963ra_write_acc_gyro_register(IMU963RA_INT1_CTRL, 0x03);
}
```

#### **方案2：高性能竞速机**
```c
// 仅修改采样率
imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x7C); // 1666Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x8C);  // 1666Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00);  // 324Hz截止（最低延迟）
```

#### **方案3：慢速/航拍机**
```c
// 降低采样率节省功耗
imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x5C); // 416Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x6C);  // 416Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x02);  // 173Hz截止（强滤波）
```

### 📊 配置对比表

| 项目 | 通用配置 | 竞速机 | 航拍机 |
|-----|---------|-------|-------|
| 加速度计ODR | 833Hz | 1666Hz | 416Hz |
| 陀螺仪ODR | 833Hz | 1666Hz | 416Hz |
| 加速度计量程 | ±8g | ±8g | ±4g |
| 陀螺仪量程 | ±2000°/s | ±2000°/s | ±1000°/s |
| LPF1截止频率 | 232Hz | 324Hz | 173Hz |
| 响应延迟 | 1.7ms | 1.0ms | 3.2ms |
| 数据质量 | 平衡 | 噪声略高 | 最平滑 |

---

## 数据读取与转换

### 📖 数据读取流程

#### 1. 中断驱动读取（推荐）
```c
// 在 INT1 中断服务函数中
void EXTI_INT1_IRQHandler(void)
{
    // 读取加速度计
    imu963ra_get_acc();

    // 读取陀螺仪
    imu963ra_get_gyro();

    // 转换为物理单位
    float acc_x_g = imu963ra_acc_transition(imu963ra_acc_x);    // 单位：g
    float gyro_x_dps = imu963ra_gyro_transition(imu963ra_gyro_x); // 单位：°/s

    // 传递给姿态解算
    attitude_update(acc_x_g, acc_y_g, acc_z_g,
                   gyro_x_dps, gyro_y_dps, gyro_z_dps);
}
```

#### 2. 轮询读取（调试用）
```c
void imu_polling_read(void)
{
    uint8 status = imu963ra_read_acc_gyro_register(IMU963RA_STATUS_REG);

    // 检查数据就绪位
    if(status & 0x01) // 陀螺仪数据就绪
    {
        imu963ra_get_gyro();
    }

    if(status & 0x02) // 加速度计数据就绪
    {
        imu963ra_get_acc();
    }
}
```

### 🔄 单位转换公式

#### 加速度计
```c
// 原始数据 → g（重力加速度）
float acc_g = (float)imu963ra_acc_x / imu963ra_transition_factor[0];

// g → m/s² （国际单位）
float acc_ms2 = acc_g * 9.80665f;

// 示例：±8g 配置
// transition_factor[0] = 4098
// 16位数据范围：-32768 ~ +32767
// 对应物理量：-8g ~ +8g
```

#### 陀螺仪
```c
// 原始数据 → °/s（角速度）
float gyro_dps = (float)imu963ra_gyro_x / imu963ra_transition_factor[1];

// °/s → rad/s（国际单位）
float gyro_rads = gyro_dps * 0.017453293f; // π/180

// 示例：±2000°/s 配置
// transition_factor[1] = 14.3
// 16位数据范围：-32768 ~ +32767
// 对应物理量：-2000°/s ~ +2000°/s
```

### 🛠️ 数据预处理（推荐）

```c
typedef struct {
    float acc_offset[3];   // 加速度计零偏
    float gyro_offset[3];  // 陀螺仪零偏
    float acc_scale[3];    // 加速度计缩放系数
} imu_calibration_t;

imu_calibration_t calib;

// 应用校准
void imu_apply_calibration(float* acc, float* gyro)
{
    // 陀螺仪零偏补偿
    gyro[0] -= calib.gyro_offset[0];
    gyro[1] -= calib.gyro_offset[1];
    gyro[2] -= calib.gyro_offset[2];

    // 加速度计校准
    acc[0] = (acc[0] - calib.acc_offset[0]) * calib.acc_scale[0];
    acc[1] = (acc[1] - calib.acc_offset[1]) * calib.acc_scale[1];
    acc[2] = (acc[2] - calib.acc_offset[2]) * calib.acc_scale[2];
}
```

---

## 实战配置案例

### 🎯 案例1：修改现有代码为高性能配置

#### 当前配置（代码中）
```c
// 文件：zf_device_imu963ra.c:398
imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x30); // 104Hz, ±2g ❌ 不适合飞控

// 文件：zf_device_imu963ra.c:458
imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x5C);  // 208Hz, ±2000°/s ❌ 太慢
```

#### 修改步骤
**步骤1**：打开 `zf_device_imu963ra.h`，修改默认配置：
```c
// 行105-107，修改为：
#define IMU963RA_ACC_SAMPLE_DEFAULT     ( IMU963RA_ACC_SAMPLE_SGN_8G )
#define IMU963RA_GYRO_SAMPLE_DEFAULT    ( IMU963RA_GYRO_SAMPLE_SGN_2000DPS )
#define IMU963RA_MAG_SAMPLE_DEFAULT     ( IMU963RA_MAG_SAMPLE_8G )
```

**步骤2**：打开 `zf_device_imu963ra.c`，修改采样率：

**修改加速度计**（约在第408行）：
```c
case IMU963RA_ACC_SAMPLE_SGN_8G:
{
    // 原代码：
    // imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x3C); // 104Hz

    // 修改为：
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x6C); // 833Hz ✅
    imu963ra_transition_factor[0] = 4098;
}break;
```

**修改陀螺仪**（约在第458行）：
```c
case IMU963RA_GYRO_SAMPLE_SGN_2000DPS:
{
    // 原代码：
    // imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x5C); // 208Hz

    // 修改为：
    imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x7C); // 833Hz ✅
    imu963ra_transition_factor[1] = 14.3;
}break;
```

**步骤3**：调整滤波器（约在第475行）：
```c
// 原代码：
// imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00); // 133Hz截止

// 修改为：
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x01); // 232Hz截止 ✅
```

---

### 🎯 案例2：姿态解算算法适配

#### Mahony 滤波器配置
```c
#define MAHONY_KP  2.0f     // 比例增益（Kp越大，收敛越快，但噪声敏感）
#define MAHONY_KI  0.05f    // 积分增益（补偿陀螺仪漂移）
#define SAMPLE_FREQ 833.0f  // 与IMU采样率匹配 ✅

void mahony_update(float ax, float ay, float az,  // 加速度（g）
                   float gx, float gy, float gz)  // 角速度（°/s）
{
    // 角速度转 rad/s
    gx *= 0.017453293f;
    gy *= 0.017453293f;
    gz *= 0.017453293f;

    // 归一化加速度
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // ... Mahony 算法核心代码 ...
}
```

#### Complementary 滤波器配置
```c
#define ALPHA 0.98f  // 陀螺仪权重（高ODR时可提高到0.99）

float pitch = 0.0f, roll = 0.0f;

void complementary_filter_update(float ax, float ay, float az,
                                 float gx, float gy, float gz,
                                 float dt) // dt = 1/833 ≈ 0.0012s
{
    // 加速度计计算姿态角
    float acc_pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2957795f;
    float acc_roll = atan2f(ay, az) * 57.2957795f;

    // 陀螺仪积分
    pitch += gx * dt;
    roll += gy * dt;

    // 互补滤波
    pitch = ALPHA * pitch + (1.0f - ALPHA) * acc_pitch;
    roll = ALPHA * roll + (1.0f - ALPHA) * acc_roll;
}
```

---

## 📝 快速参考表

### 关键寄存器速查

| 寄存器 | 地址 | 功能 | 推荐值 | 说明 |
|-------|------|------|-------|------|
| CTRL1_XL | 0x10 | 加速度计控制 | 0x6C | 833Hz, ±8g |
| CTRL2_G | 0x11 | 陀螺仪控制 | 0x7C | 833Hz, ±2000°/s |
| CTRL3_C | 0x12 | 通用控制 | 0x44 | BDU启用 |
| CTRL4_C | 0x13 | 滤波器使能 | 0x02 | 陀螺仪LPF1 |
| CTRL6_C | 0x15 | 滤波器截止频率 | 0x01 | 232Hz截止 |
| CTRL7_G | 0x16 | 陀螺仪高级配置 | 0x00 | 高性能模式 |
| INT1_CTRL | 0x0D | 中断配置 | 0x03 | 数据就绪中断 |

### ODR 十六进制速查

| 频率 | CTRL1_XL高4位 | CTRL2_G高4位 |
|------|--------------|-------------|
| 104 Hz | 3x | 5x |
| 208 Hz | 4x | 6x |
| 416 Hz | 5x | 6x |
| 833 Hz | 6x | 7x |
| 1666 Hz | 7x | 8x |
| 3333 Hz | 8x | - |

---

## 🚨 常见问题排查

### 问题1：数据噪声大
**原因**：
- 滤波器未启用（CTRL4_C.LPF1_SEL_G = 0）
- 截止频率过高（CTRL6_C.FTYPE = 0x00）
- 量程选择过大（如 ±16g）

**解决**：
```c
imu963ra_write_acc_gyro_register(IMU963RA_CTRL4_C, 0x02); // 启用LPF1
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x02); // 降低截止频率到173Hz
```

### 问题2：响应迟钝
**原因**：
- ODR 过低（< 400Hz）
- 滤波器截止频率过低
- 启用了二级滤波器

**解决**：
```c
imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x7C); // 提高到1666Hz
imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00);  // 提高截止频率
```

### 问题3：数据突变/跳变
**原因**：
- BDU（块数据更新）未启用
- 读取时序错误

**解决**：
```c
imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x44); // 启用BDU
```

### 问题4：陀螺仪漂移严重
**原因**：
- 未进行零偏校准
- 温度漂移

**解决**：
```c
// 静止状态下采集1000个样本
for(int i=0; i<1000; i++)
{
    imu963ra_get_gyro();
    gyro_offset_x += imu963ra_gyro_transition(imu963ra_gyro_x);
    system_delay_ms(1);
}
gyro_offset_x /= 1000.0f;

// 使用时减去零偏
float gyro_x_calibrated = imu963ra_gyro_transition(imu963ra_gyro_x) - gyro_offset_x;
```

---

## 📚 进阶学习资源

### 姿态解算算法
1. **Mahony Filter**：低计算量，适合嵌入式（推荐）
2. **Madgwick Filter**：精度高，需要磁力计
3. **EKF（扩展卡尔曼滤波）**：工业级，计算量大

### 飞控框架参考
- **Betaflight**：开源穿越机固件（可参考滤波器配置）
- **PX4**：开源多旋翼/固定翼系统
- **ArduPilot**：功能丰富的开源飞控

### 数据手册
- **LSM6DSM Datasheet**：详细寄存器说明
- **AN4987**：ST官方姿态解算应用笔记

---

## ✅ 配置检查清单

在完成配置后，请确认：

- [ ] 加速度计 ODR ≥ 400Hz（推荐 833Hz）
- [ ] 陀螺仪 ODR ≥ 400Hz（推荐 833Hz）
- [ ] 加速度计量程为 ±8g
- [ ] 陀螺仪量程为 ±2000°/s
- [ ] 启用陀螺仪 LPF1（CTRL4_C = 0x02）
- [ ] 启用 BDU（CTRL3_C.BDU = 1）
- [ ] 启用数据就绪中断（INT1_CTRL = 0x03）
- [ ] 关闭二级滤波器（CTRL8_XL.LPF2_XL_EN = 0）
- [ ] 验证数据转换系数正确
- [ ] 完成陀螺仪零偏校准

---

**文档版本**：v1.0
**最后更新**：2025-10-27
**适用硬件**：IMU963RA (LSM6DSM + MMC5983MA)
**适用平台**：MM32F327X-G8P / 其他嵌入式平台

---

## 📧 反馈与改进
如有问题或建议，欢迎提出！
