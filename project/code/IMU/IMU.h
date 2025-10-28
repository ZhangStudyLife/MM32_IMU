/*********************************************************************************************************************
* MM32F327X IMU姿态解算系统 - 顶层统一接口
*
* 文件名称          IMU.h
* 作者              老王暴躁技术流
* 版本              v1.0
* 创建日期          2025-10-27
*
* 功能说明:
*   1. 提供统一的IMU系统初始化接口
*   2. 管理所有姿态解算算法实例
*   3. 统一读取IMU963RA原始数据并分发给各算法
*   4. 提供算法使能/禁用、结果获取等功能
*   5. 支持多算法并行运行和实时对比
*
* 使用示例:
*   1. 初始化IMU系统（采样率1000Hz）
*   imu_system_init(1000);
*
*   2. 使能要使用的算法
*   imu_algorithm_enable(IMU_ALGO_MADGWICK);
*   imu_algorithm_enable(IMU_ALGO_MAHONY);
*
*   3. 在主循环中周期调用update
*   while (1) {
*       imu_update();  // 读取IMU数据，更新所有使能的算法
*       delay_ms(1);   // 1ms周期 = 1000Hz
*   }
*
*   4. 获取姿态结果
*   imu_attitude_t attitude;
*   imu_get_attitude(IMU_ALGO_MADGWICK, &attitude);
*   printf("Pitch: %.2f° Roll: %.2f° Yaw: %.2f°\n",
*          attitude.pitch, attitude.roll, attitude.yaw);
*
********************************************************************************************************************/

#ifndef _IMU_H
#define _IMU_H

#include "common/imu_common.h"
#include "common/imu_math.h"

/********************************************************************************************************************
 * 系统初始化与配置
 ********************************************************************************************************************/

/**
 * @brief 初始化IMU系统
 * @param sample_rate 采样率 (Hz), 范围: 50~1000
 * @return 0-成功, 非0-失败
 * @note  这个函数会：
 *        1. 初始化IMU963RA硬件
 *        2. 初始化所有算法实例（但不使能）
 *        3. 配置系统参数
 *        4. 准备好原始数据缓冲区
 */
int imu_system_init(uint32_t sample_rate);

/**
 * @brief 获取IMU系统配置
 * @param config 输出配置结构体指针
 */
void imu_get_config(imu_config_t *config);

/**
 * @brief 设置IMU系统配置
 * @param config 输入配置结构体指针
 * @return 0-成功, 非0-失败
 */
int imu_set_config(const imu_config_t *config);

/********************************************************************************************************************
 * 算法管理
 ********************************************************************************************************************/

/**
 * @brief 使能指定算法
 * @param algo_type 算法类型
 * @return 0-成功, 非0-失败
 * @note  使能后，该算法会在每次imu_update()时自动更新
 *        可以同时使能多个算法进行对比
 */
int imu_algorithm_enable(imu_algorithm_type_t algo_type);

/**
 * @brief 禁用指定算法
 * @param algo_type 算法类型
 * @note  禁用后，该算法不再更新，节省CPU
 */
void imu_algorithm_disable(imu_algorithm_type_t algo_type);

/**
 * @brief 检查算法是否已使能
 * @param algo_type 算法类型
 * @return true-已使能, false-未使能
 */
bool imu_algorithm_is_enabled(imu_algorithm_type_t algo_type);

/**
 * @brief 获取算法状态
 * @param algo_type 算法类型
 * @return 算法状态
 */
imu_algo_state_t imu_algorithm_get_state(imu_algorithm_type_t algo_type);

/**
 * @brief 重置指定算法
 * @param algo_type 算法类型
 * @note  重置算法状态，四元数恢复为单位四元数
 */
void imu_algorithm_reset(imu_algorithm_type_t algo_type);

/**
 * @brief 重置所有算法
 */
void imu_reset_all(void);

/********************************************************************************************************************
 * 数据更新与获取
 ********************************************************************************************************************/

/**
 * @brief 更新IMU数据（周期调用）
 * @note  这个函数会：
 *        1. 从IMU963RA读取原始数据
 *        2. 数据预处理（单位转换、校准）
 *        3. 更新所有已使能的算法
 *        4. 记录性能统计数据
 *
 *        调用频率应该与初始化时设置的采样率一致！
 */
void imu_update(void);

/**
 * @brief 获取指定算法的姿态结果
 * @param algo_type 算法类型
 * @param attitude 输出姿态数据指针
 * @return 0-成功, 非0-失败
 * @note  输出的角度单位是度（°）
 *        Pitch: -90° ~ +90°
 *        Roll:  -180° ~ +180°
 *        Yaw:   -180° ~ +180°
 */
int imu_get_attitude(imu_algorithm_type_t algo_type, imu_attitude_t *attitude);

/**
 * @brief 获取指定算法的四元数（供高级用户使用）
 * @param algo_type 算法类型
 * @param quat 输出四元数指针
 * @return 0-成功, 非0-失败
 */
int imu_get_quaternion(imu_algorithm_type_t algo_type, quaternion_t *quat);

/**
 * @brief 获取IMU原始数据（只读）
 * @return 原始数据指针
 * @note  返回的指针指向内部缓冲区，只能读取不能修改！
 *        所有算法共享这份数据，保证公平对比
 */
const imu_raw_data_t* imu_get_raw_data(void);

/********************************************************************************************************************
 * 调试与测试
 ********************************************************************************************************************/

/**
 * @brief 打印所有算法的姿态对比
 * @note  用于调试和算法评估，会输出所有已使能算法的姿态
 *        格式示例：
 *        [Madgwick] Pitch:  12.34° Roll:  -5.67° Yaw:  89.01° (0.5ms)
 *        [Mahony]   Pitch:  12.31° Roll:  -5.65° Yaw:  89.05° (0.4ms)
 */
void imu_print_all_attitudes(void);

/**
 * @brief 打印指定算法的姿态
 * @param algo_type 算法类型
 */
void imu_print_attitude(imu_algorithm_type_t algo_type);

/**
 * @brief 打印原始传感器数据
 * @note  输出加速度计、陀螺仪、磁力计原始数据
 *        用于传感器校准和故障诊断
 */
void imu_print_raw_data(void);

/**
 * @brief 获取算法性能统计
 * @param algo_type 算法类型
 * @param update_count 输出更新次数
 * @param avg_time_us 输出平均执行时间（微秒）
 * @param max_time_us 输出最大执行时间（微秒）
 * @return 0-成功, 非0-失败
 */
int imu_get_performance(imu_algorithm_type_t algo_type,
                        uint32_t *update_count,
                        uint32_t *avg_time_us,
                        uint32_t *max_time_us);

/**
 * @brief 打印所有算法的性能统计
 * @note  用于性能测试和优化
 */
void imu_print_performance(void);

/********************************************************************************************************************
 * 传感器校准（后续扩展）
 ********************************************************************************************************************/

/**
 * @brief 开始加速度计校准
 * @note  IMU需要静止放置在水平面上
 */
void imu_calibrate_acc_start(void);

/**
 * @brief 开始陀螺仪校准
 * @note  IMU需要完全静止
 */
void imu_calibrate_gyro_start(void);

/**
 * @brief 开始磁力计校准
 * @note  需要将IMU绕各个轴旋转，采集数据用于椭球拟合
 */
void imu_calibrate_mag_start(void);

/**
 * @brief 检查校准是否完成
 * @return true-完成, false-进行中
 */
bool imu_calibrate_is_done(void);

/**
 * @brief 保存校准参数
 * @note  将校准参数保存到Flash
 */
void imu_calibrate_save(void);

/**
 * @brief 加载校准参数
 * @note  从Flash加载校准参数
 */
void imu_calibrate_load(void);

#endif // _IMU_H

/********************************************************************************************************************
 * 老王的使用指南:
 *
 * 【基本使用流程】
 * 1. 调用 imu_system_init(1000) 初始化系统（1000Hz采样率）
 * 2. 调用 imu_algorithm_enable() 使能需要的算法
 * 3. 在定时器中断或主循环中周期调用 imu_update()
 * 4. 随时调用 imu_get_attitude() 获取姿态结果
 *
 * 【多算法对比】
 * 同时使能多个算法，然后调用 imu_print_all_attitudes() 查看对比结果
 * 例如：
 *   imu_algorithm_enable(IMU_ALGO_MADGWICK);
 *   imu_algorithm_enable(IMU_ALGO_MAHONY);
 *   imu_algorithm_enable(IMU_ALGO_EKF);
 *   // 在主循环中
 *   imu_print_all_attitudes();  // 实时对比三种算法
 *
 * 【性能测试】
 * 调用 imu_print_performance() 查看各算法的CPU占用情况
 *
 * 【传感器校准】
 * 首次使用或更换传感器后，需要校准：
 * 1. 陀螺仪校准：静止放置，调用 imu_calibrate_gyro_start()
 * 2. 加速度计校准：水平放置，调用 imu_calibrate_acc_start()
 * 3. 磁力计校准：8字形旋转，调用 imu_calibrate_mag_start()
 * 4. 校准完成后调用 imu_calibrate_save() 保存参数
 *
 * 艹！接口设计得够清晰了吧！用起来很简单的！
 *
 ********************************************************************************************************************/
