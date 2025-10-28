#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
陀螺仪EKF数据分析脚本
静止状态下的IMU数据质量评估

数据格式：
列1-3: gyro_x, gyro_y, gyro_z (rad/s) - 角速度
列4-6: acc_x, acc_y, acc_z (g) - 加速度
列7-9: pitch, roll, yaw (度) - 姿态角
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体支持（艹，matplotlib中文显示真tm麻烦）
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False

# 文件路径配置
CSV_FILE = Path("Mahony.csv")
OUTPUT_DIR = Path("./")
OUTPUT_DIR.mkdir(exist_ok=True)

# 数据列名
COLUMNS = ['gyro_x', 'gyro_y', 'gyro_z',
           'acc_x', 'acc_y', 'acc_z',
           'pitch', 'roll', 'yaw']

print("=" * 80)
print("🔧 老王的陀螺仪数据分析工具 v1.0")
print("=" * 80)

# ============================================================================
# 1. 读取数据（这个SB大文件可能要读一会儿）
# ============================================================================
print(f"\n📂 正在读取CSV文件: {CSV_FILE}")
try:
    # CSV有标题行(I0-I8)，跳过它并使用自定义列名
    df = pd.read_csv(CSV_FILE, header=0, names=COLUMNS, dtype=float)
    print(f"✅ 数据读取成功！共 {len(df)} 行数据")
    print(f"   数据时长: 约 {len(df) / 100:.1f} 秒 (假设采样率100Hz)")
except Exception as e:
    print(f"❌ 艹！读取失败: {e}")
    exit(1)

# 数据基本信息
print(f"\n📊 数据概览:")
print(df.describe())

# ============================================================================
# 2. 数据质量分析（重点！静止状态评估）
# ============================================================================
print("\n" + "=" * 80)
print("📈 静止状态数据质量分析")
print("=" * 80)

results = {}

for col in COLUMNS:
    data = df[col].values

    # 基本统计
    mean_val = np.mean(data)
    std_val = np.std(data)
    max_val = np.max(data)
    min_val = np.min(data)
    peak_to_peak = max_val - min_val

    # 线性漂移检测（用线性拟合评估长期趋势）
    time_idx = np.arange(len(data))
    # 使用numpy实现线性回归
    coeffs = np.polyfit(time_idx, data, 1)  # [slope, intercept]
    slope = coeffs[0]
    intercept = coeffs[1]
    fitted_values = slope * time_idx + intercept
    # 计算R²值
    ss_res = np.sum((data - fitted_values) ** 2)
    ss_tot = np.sum((data - np.mean(data)) ** 2)
    r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
    drift_rate = slope * 100  # 每100个采样点的漂移量

    results[col] = {
        'mean': mean_val,
        'std': std_val,
        'min': min_val,
        'max': max_val,
        'peak_to_peak': peak_to_peak,
        'drift_rate': drift_rate,
        'drift_r2': r_squared
    }

    print(f"\n📌 {col}:")
    print(f"   均值: {mean_val:+.6f}")
    print(f"   标准差: {std_val:.6f}")
    print(f"   峰峰值: {peak_to_peak:.6f}")
    print(f"   漂移率: {drift_rate:.6f} /秒 (R²={r_squared:.4f})")

# ============================================================================
# 3. 评估标准（老王的经验值）
# ============================================================================
print("\n" + "=" * 80)
print("🎯 EKF参数质量评估")
print("=" * 80)

def evaluate_quality(col, res):
    """评估数据质量并给出建议"""
    issues = []
    score = 100

    if 'gyro' in col:
        # 陀螺仪评估标准（rad/s）
        if abs(res['mean']) > 0.01:  # 零偏 > 0.01 rad/s ≈ 0.57°/s
            issues.append(f"⚠️  零偏过大: {res['mean']:+.4f} rad/s")
            score -= 30
        elif abs(res['mean']) > 0.005:
            issues.append(f"⚠️  零偏偏高: {res['mean']:+.4f} rad/s")
            score -= 15

        if res['std'] > 0.005:  # 噪声 > 0.005 rad/s
            issues.append(f"⚠️  噪声过大: {res['std']:.4f} rad/s")
            score -= 20
        elif res['std'] > 0.002:
            issues.append(f"⚠️  噪声偏高: {res['std']:.4f} rad/s")
            score -= 10

        if abs(res['drift_rate']) > 0.001 and res['drift_r2'] > 0.5:
            issues.append(f"⚠️  存在明显漂移: {res['drift_rate']:.6f} rad/s/s")
            score -= 25

    elif 'acc' in col:
        # 加速度计评估标准（g）
        if col == 'acc_z':
            # Z轴应该接近1g（重力）
            if abs(res['mean'] - 1.0) > 0.1:
                issues.append(f"⚠️  Z轴重力偏差过大: {res['mean']:.4f}g (应≈1.0g)")
                score -= 30
            elif abs(res['mean'] - 1.0) > 0.05:
                issues.append(f"⚠️  Z轴重力偏差偏高: {res['mean']:.4f}g")
                score -= 15
        else:
            # X/Y轴应该接近0
            if abs(res['mean']) > 0.1:
                issues.append(f"⚠️  零偏过大: {res['mean']:+.4f}g")
                score -= 30
            elif abs(res['mean']) > 0.05:
                issues.append(f"⚠️  零偏偏高: {res['mean']:+.4f}g")
                score -= 15

        if res['std'] > 0.02:
            issues.append(f"⚠️  噪声过大: {res['std']:.4f}g")
            score -= 20
        elif res['std'] > 0.01:
            issues.append(f"⚠️  噪声偏高: {res['std']:.4f}g")
            score -= 10

    else:  # 姿态角
        # 姿态角评估标准（度）
        if res['std'] > 1.0:
            issues.append(f"⚠️  姿态抖动过大: {res['std']:.4f}°")
            score -= 30
        elif res['std'] > 0.5:
            issues.append(f"⚠️  姿态抖动偏高: {res['std']:.4f}°")
            score -= 15

        if abs(res['drift_rate']) > 0.1 and res['drift_r2'] > 0.5:
            issues.append(f"⚠️  姿态漂移严重: {res['drift_rate']:.4f}°/s")
            score -= 35
        elif abs(res['drift_rate']) > 0.05 and res['drift_r2'] > 0.5:
            issues.append(f"⚠️  姿态存在漂移: {res['drift_rate']:.4f}°/s")
            score -= 20

    return score, issues

total_score = 0
print("\n逐项评估:")
for col, res in results.items():
    score, issues = evaluate_quality(col, res)
    total_score += score

    if score >= 90:
        status = "✅ 优秀"
    elif score >= 70:
        status = "⚠️  良好"
    elif score >= 50:
        status = "⚠️  一般"
    else:
        status = "❌ 较差"

    print(f"\n{col}: {status} (评分: {score}/100)")
    if issues:
        for issue in issues:
            print(f"  {issue}")

avg_score = total_score / len(COLUMNS)
print(f"\n{'='*80}")
print(f"📊 总体评分: {avg_score:.1f}/100")

if avg_score >= 90:
    print("✅ EKF参数整体表现优秀！传感器数据质量很好！")
elif avg_score >= 70:
    print("⚠️  EKF参数整体表现良好，但仍有优化空间")
elif avg_score >= 50:
    print("⚠️  EKF参数表现一般，建议检查传感器校准和参数调优")
else:
    print("❌ EKF参数表现较差！需要重新校准传感器或调整参数！")

# ============================================================================
# 4. 数据可视化（重头戏来了！）
# ============================================================================
print("\n" + "=" * 80)
print("📊 正在生成可视化图表...")
print("=" * 80)

# 创建时间轴（假设100Hz采样）
time = np.arange(len(df)) / 100.0  # 秒

# --- 图1: 陀螺仪数据时序图 ---
fig1, axes = plt.subplots(3, 1, figsize=(14, 10))
fig1.suptitle('陀螺仪角速度时序图 (静止状态)', fontsize=16, fontweight='bold')

gyro_cols = ['gyro_x', 'gyro_y', 'gyro_z']
colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']
for idx, (col, color) in enumerate(zip(gyro_cols, colors)):
    ax = axes[idx]
    ax.plot(time, df[col], color=color, linewidth=0.5, alpha=0.7)
    ax.axhline(y=results[col]['mean'], color='red', linestyle='--',
               linewidth=2, label=f"均值: {results[col]['mean']:+.4f} rad/s")
    ax.fill_between(time,
                     results[col]['mean'] - results[col]['std'],
                     results[col]['mean'] + results[col]['std'],
                     color=color, alpha=0.2, label=f"±1σ: {results[col]['std']:.4f} rad/s")
    ax.set_ylabel(f'{col}\n(rad/s)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')

axes[-1].set_xlabel('时间 (秒)', fontsize=11)
plt.tight_layout()
fig1.savefig(OUTPUT_DIR / 'gyro_timeseries.png', dpi=150, bbox_inches='tight')
print("✅ 保存: gyro_timeseries.png")

# --- 图2: 加速度计数据时序图 ---
fig2, axes = plt.subplots(3, 1, figsize=(14, 10))
fig2.suptitle('加速度计时序图 (静止状态)', fontsize=16, fontweight='bold')

acc_cols = ['acc_x', 'acc_y', 'acc_z']
colors = ['#FF6B6B', '#4ECDC4', '#95E1D3']
for idx, (col, color) in enumerate(zip(acc_cols, colors)):
    ax = axes[idx]
    ax.plot(time, df[col], color=color, linewidth=0.5, alpha=0.7)
    ax.axhline(y=results[col]['mean'], color='red', linestyle='--',
               linewidth=2, label=f"均值: {results[col]['mean']:+.4f} g")

    # Z轴标注1g参考线
    if col == 'acc_z':
        ax.axhline(y=1.0, color='green', linestyle=':',
                   linewidth=2, label='理论值: 1.0 g')
    else:
        ax.axhline(y=0.0, color='green', linestyle=':',
                   linewidth=2, label='理论值: 0.0 g')

    ax.fill_between(time,
                     results[col]['mean'] - results[col]['std'],
                     results[col]['mean'] + results[col]['std'],
                     color=color, alpha=0.2, label=f"±1σ: {results[col]['std']:.4f} g")
    ax.set_ylabel(f'{col}\n(g)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')

axes[-1].set_xlabel('时间 (秒)', fontsize=11)
plt.tight_layout()
fig2.savefig(OUTPUT_DIR / 'acc_timeseries.png', dpi=150, bbox_inches='tight')
print("✅ 保存: acc_timeseries.png")

# --- 图3: 姿态角时序图 ---
fig3, axes = plt.subplots(3, 1, figsize=(14, 10))
fig3.suptitle('EKF姿态解算结果 (静止状态)', fontsize=16, fontweight='bold')

angle_cols = ['pitch', 'roll', 'yaw']
colors = ['#FFA07A', '#98D8C8', '#F7DC6F']
for idx, (col, color) in enumerate(zip(angle_cols, colors)):
    ax = axes[idx]
    ax.plot(time, df[col], color=color, linewidth=0.8, alpha=0.8)
    ax.axhline(y=results[col]['mean'], color='red', linestyle='--',
               linewidth=2, label=f"均值: {results[col]['mean']:+.2f}°")
    ax.fill_between(time,
                     results[col]['mean'] - results[col]['std'],
                     results[col]['mean'] + results[col]['std'],
                     color=color, alpha=0.2, label=f"±1σ: {results[col]['std']:.3f}°")
    ax.set_ylabel(f'{col}\n(度)', fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')

axes[-1].set_xlabel('时间 (秒)', fontsize=11)
plt.tight_layout()
fig3.savefig(OUTPUT_DIR / 'attitude_timeseries.png', dpi=150, bbox_inches='tight')
print("✅ 保存: attitude_timeseries.png")

# --- 图4: 数据分布直方图 ---
fig4, axes = plt.subplots(3, 3, figsize=(15, 12))
fig4.suptitle('数据分布直方图 (静止状态)', fontsize=16, fontweight='bold')

for idx, col in enumerate(COLUMNS):
    row = idx // 3
    col_idx = idx % 3
    ax = axes[row, col_idx]

    data = df[col].values
    ax.hist(data, bins=50, color='skyblue', edgecolor='black', alpha=0.7)
    ax.axvline(x=results[col]['mean'], color='red', linestyle='--',
               linewidth=2, label=f"均值: {results[col]['mean']:.4f}")
    ax.set_xlabel('数值', fontsize=10)
    ax.set_ylabel('频次', fontsize=10)
    ax.set_title(col, fontsize=12, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

plt.tight_layout()
fig4.savefig(OUTPUT_DIR / 'data_distribution.png', dpi=150, bbox_inches='tight')
print("✅ 保存: data_distribution.png")

# --- 图5: 综合仪表盘 ---
fig5 = plt.figure(figsize=(16, 10))
gs = fig5.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

fig5.suptitle('IMU数据质量综合仪表盘', fontsize=18, fontweight='bold')

# 陀螺仪综合视图
ax1 = fig5.add_subplot(gs[0, :])
for col, color in zip(gyro_cols, ['r', 'g', 'b']):
    ax1.plot(time, df[col], color=color, linewidth=0.5, alpha=0.6, label=col)
ax1.set_ylabel('角速度 (rad/s)', fontsize=11)
ax1.set_title('陀螺仪三轴数据', fontsize=13, fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)

# 加速度计综合视图
ax2 = fig5.add_subplot(gs[1, :])
for col, color in zip(acc_cols, ['r', 'g', 'b']):
    ax2.plot(time, df[col], color=color, linewidth=0.5, alpha=0.6, label=col)
ax2.set_ylabel('加速度 (g)', fontsize=11)
ax2.set_title('加速度计三轴数据', fontsize=13, fontweight='bold')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 姿态角综合视图
ax3 = fig5.add_subplot(gs[2, :])
for col, color in zip(angle_cols, ['orange', 'purple', 'brown']):
    ax3.plot(time, df[col], color=color, linewidth=0.8, alpha=0.7, label=col)
ax3.set_xlabel('时间 (秒)', fontsize=11)
ax3.set_ylabel('角度 (度)', fontsize=11)
ax3.set_title('EKF姿态解算', fontsize=13, fontweight='bold')
ax3.legend()
ax3.grid(True, alpha=0.3)

plt.tight_layout()
fig5.savefig(OUTPUT_DIR / 'dashboard.png', dpi=150, bbox_inches='tight')
print("✅ 保存: dashboard.png")

# ============================================================================
# 5. 生成分析报告
# ============================================================================
report_file = OUTPUT_DIR / 'analysis_report.txt'
with open(report_file, 'w', encoding='utf-8') as f:
    f.write("=" * 80 + "\n")
    f.write("陀螺仪EKF数据分析报告\n")
    f.write("=" * 80 + "\n\n")

    f.write(f"数据文件: {CSV_FILE}\n")
    f.write(f"数据量: {len(df)} 个采样点\n")
    f.write(f"采样时长: 约 {len(df) / 100:.1f} 秒 (假设100Hz)\n")
    f.write(f"测试条件: 陀螺仪完全静止\n\n")

    f.write("=" * 80 + "\n")
    f.write("详细数据统计\n")
    f.write("=" * 80 + "\n\n")

    for col, res in results.items():
        f.write(f"\n{col}:\n")
        f.write(f"  均值:       {res['mean']:+.6f}\n")
        f.write(f"  标准差:     {res['std']:.6f}\n")
        f.write(f"  最小值:     {res['min']:+.6f}\n")
        f.write(f"  最大值:     {res['max']:+.6f}\n")
        f.write(f"  峰峰值:     {res['peak_to_peak']:.6f}\n")
        f.write(f"  漂移率:     {res['drift_rate']:.6f} /秒\n")
        f.write(f"  漂移R²:     {res['drift_r2']:.4f}\n")

        score, issues = evaluate_quality(col, res)
        f.write(f"  质量评分:   {score}/100\n")
        if issues:
            f.write("  问题诊断:\n")
            for issue in issues:
                f.write(f"    {issue}\n")

    f.write("\n" + "=" * 80 + "\n")
    f.write("总体评估\n")
    f.write("=" * 80 + "\n\n")
    f.write(f"总体评分: {avg_score:.1f}/100\n\n")

    if avg_score >= 90:
        f.write("✅ EKF参数整体表现优秀！\n")
        f.write("   传感器零偏小、噪声低、无明显漂移，数据质量很好！\n")
    elif avg_score >= 70:
        f.write("⚠️  EKF参数整体表现良好\n")
        f.write("   传感器基本正常，但仍有优化空间，建议微调参数。\n")
    elif avg_score >= 50:
        f.write("⚠️  EKF参数表现一般\n")
        f.write("   建议检查传感器校准流程，优化EKF过程噪声和测量噪声参数。\n")
    else:
        f.write("❌ EKF参数表现较差！\n")
        f.write("   需要重新进行传感器校准！检查硬件连接和参数设置！\n")

    f.write("\n" + "=" * 80 + "\n")
    f.write("优化建议\n")
    f.write("=" * 80 + "\n\n")

    # 针对性建议
    gyro_score = sum([evaluate_quality(col, results[col])[0] for col in gyro_cols]) / 3
    acc_score = sum([evaluate_quality(col, results[col])[0] for col in acc_cols]) / 3
    angle_score = sum([evaluate_quality(col, results[col])[0] for col in angle_cols]) / 3

    if gyro_score < 70:
        f.write("📌 陀螺仪参数优化:\n")
        f.write("   1. 重新执行静态零偏校准 (imu_calibrate_gyro_start)\n")
        f.write("   2. 增加校准采样次数，提高零偏估计精度\n")
        f.write("   3. 检查陀螺仪量程设置是否合适\n")
        f.write("   4. 考虑启用温度补偿（如果硬件支持）\n\n")

    if acc_score < 70:
        f.write("📌 加速度计参数优化:\n")
        f.write("   1. 检查加速度计六面校准是否正确执行\n")
        f.write("   2. 验证零偏和比例因子参数\n")
        f.write("   3. 检查传感器安装方向是否正确\n\n")

    if angle_score < 70:
        f.write("📌 EKF滤波参数优化:\n")
        f.write("   1. 调整过程噪声协方差Q矩阵（降低以减少响应速度，增加稳定性）\n")
        f.write("   2. 调整测量噪声协方差R矩阵（根据实际传感器噪声水平设定）\n")
        f.write("   3. 检查初始协方差P矩阵设置\n")
        f.write("   4. 考虑增加互补滤波预处理环节\n\n")

    f.write("\n生成时间: " + pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S') + "\n")

print(f"✅ 保存: analysis_report.txt")

print("\n" + "=" * 80)
print("🎉 分析完成！所有文件已保存到 data_analysis 目录")
print("=" * 80)
print("\n生成的文件:")
print("  📊 gyro_timeseries.png      - 陀螺仪时序图")
print("  📊 acc_timeseries.png       - 加速度计时序图")
print("  📊 attitude_timeseries.png  - 姿态角时序图")
print("  📊 data_distribution.png    - 数据分布直方图")
print("  📊 dashboard.png            - 综合仪表盘")
print("  📄 analysis_report.txt      - 详细分析报告")
print("\n建议先查看 dashboard.png 了解整体情况，再查看详细报告！\n")
