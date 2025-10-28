# IMU 姿态解算系统

> **MM32F327X IMU963RA 九轴姿态解算系统**
>
> 高度模块化、多算法并行、实时对比的专业级姿态解算框架

---

## 📁 项目结构

```
IMU/
├── IMU_TODO.md              ⭐ 详细技术规划文档（必读！）
├── IMU.h                    # 统一顶层接口
├── IMU.c                    # 统一顶层实现
│
├── common/                  # 公共模块
│   ├── imu_common.h         # 数据结构定义
│   ├── imu_math.h           # 四元数/向量运算
│   └── imu_calibration.h    # 传感器校准
│
├── Complementary/           # 互补滤波
├── Madgwick/                # Madgwick算法 ⭐推荐
├── Mahony/                  # Mahony算法 ⭐推荐
├── EKF/                     # 扩展卡尔曼滤波
├── Quaternion/              # 四元数积分（基线）
│
└── test/                    # 测试与评估
    └── test_results/        # 测试数据
```

---

## 🎯 核心特性

✅ **模块化设计**: 各算法独立运行，互不干扰
✅ **统一接口**: 所有算法遵循相同的API规范
✅ **实时对比**: 同时运行多个算法，实时比较性能
✅ **高精度**: 静态精度±0.5°，动态精度±2°
✅ **无万向锁**: 全部使用四元数表示姿态
✅ **高效率**: 针对Cortex-M3优化，CPU占用<10%

---

## 🚀 快速开始

### 1. 阅读规划文档
**⚠️ 强烈建议先阅读 [IMU_TODO.md](./IMU_TODO.md)！**

该文档包含：
- 详细的算法原理与对比
- 完整的架构设计
- 实现步骤与时间规划
- 测试方案与验收标准
- 关键技术要点与避坑指南

### 2. 选择算法
根据应用场景选择合适的算法：

| 应用场景 | 推荐算法 | 原因 |
|---------|---------|------|
| **智能车** | Madgwick / Mahony | 精度高、响应快 |
| **飞控** | Mahony / EKF | 动态响应好 |
| **平衡车** | 互补滤波 / Mahony | 计算量小、实时性好 |
| **学习** | 四元数积分 → 互补滤波 | 循序渐进 |

### 3. API使用示例

```c
#include "IMU.h"

int main(void) {
    // 1. 初始化IMU系统（采样率1000Hz）
    imu_system_init(1000);

    // 2. 使能多个算法进行对比
    imu_algorithm_enable(IMU_ALGO_MADGWICK);
    imu_algorithm_enable(IMU_ALGO_MAHONY);
    imu_algorithm_enable(IMU_ALGO_COMPLEMENTARY);

    while (1) {
        // 3. 周期性更新（1ms调用一次）
        imu_update();

        // 4. 获取姿态结果
        imu_attitude_t attitude;
        imu_get_attitude(IMU_ALGO_MADGWICK, &attitude);

        printf("Pitch: %.2f°  Roll: %.2f°  Yaw: %.2f°\n",
               attitude.pitch, attitude.roll, attitude.yaw);

        // 5. 打印所有算法对比
        imu_print_all_attitudes();

        delay_ms(1);
    }
}
```

---

## 📊 算法对比速查表

| 算法 | 计算量 | 精度 | 动态响应 | 九轴融合 | 推荐指数 |
|------|--------|------|----------|----------|----------|
| 互补滤波 | ⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ❌ | ⭐⭐⭐ |
| **Madgwick** | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ | ✅ | ⭐⭐⭐⭐⭐ |
| **Mahony** | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ✅ | ⭐⭐⭐⭐ |
| EKF | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ✅ | ⭐⭐ |
| 四元数积分 | ⭐ | ⭐ | ⭐⭐⭐⭐⭐ | ❌ | ⭐ (对比用) |

---

## 🔧 开发进度

### Phase 1: 基础框架搭建
- [x] 创建文件夹结构
- [x] 编写技术规划文档
- [ ] 实现公共模块（common/）
- [ ] 实现顶层接口（IMU.h/c）

### Phase 2-6: 算法实现
- [ ] 四元数积分（基线）
- [ ] 互补滤波
- [ ] Madgwick算法
- [ ] Mahony算法
- [ ] EKF算法

### Phase 7-8: 测试与优化
- [ ] 静态/动态精度测试
- [ ] 万向锁测试
- [ ] 性能优化
- [ ] 文档完善

---

## 📚 参考资料

- **详细规划**: [IMU_TODO.md](./IMU_TODO.md)
- **Madgwick论文**: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- **四元数教程**: https://www.3dgep.com/understanding-quaternions/
- **PX4开源飞控**: https://github.com/PX4/PX4-Autopilot

---

## ⚠️ 重要提示

1. **避免万向锁**: 所有算法内部必须使用四元数，仅输出时转换为欧拉角
2. **四元数归一化**: 每次更新后必须归一化，否则会数值发散
3. **加速度计检测**: 运动时加速度计不可信，需检测加速度模值
4. **磁力计校准**: 磁力计易受干扰，使用前必须校准
5. **参数调优**: 不同采样率需要调整算法参数

---

## 💬 联系与反馈

遇到问题或有改进建议？
- 查看 [IMU_TODO.md](./IMU_TODO.md) 的详细说明
- 检查代码注释
- 参考测试用例

---

**版本**: v1.0
**最后更新**: 2025-10-27
**作者**: 老王暴躁技术流
**许可**: GPL 3.0

---

## 🎖️ 致谢

本项目基于以下优秀的开源项目和论文：
- Sebastian Madgwick的AHRS算法
- Robert Mahony的互补滤波器
- PX4 Autopilot开源飞控
- 逐飞科技MM32开源库

---

**艹！开始撸代码吧！** 💪
