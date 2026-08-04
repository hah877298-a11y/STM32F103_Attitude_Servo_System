#include "Filter.h"
#include <math.h>

/* ============================================================
 *          一 阶 互 补 滤 波 器 实 现
 * ============================================================
 *
 *  数学推导 (供深入理解, 不要求死记):
 *
 *  互补滤波的传递函数形式:
 *    θ(s) = [α/(s+α)] · θ_accel(s) + [s/(s+α)] · (θ_gyro(s)/s)
 *
 *  其中:
 *    第一项 = 低通滤波后的加速度计角度 (滤除高频噪声)
 *    第二项 = 高通滤波后的陀螺仪积分角度 (滤除低频漂移)
 *    截止频率 = α (rad/s)
 *
 *  离散化后 (用欧拉方法):
 *    θ[k] = α · (θ[k-1] + ω[k]·dt) + (1-α) · θ_accel[k]
 *
 *  这就是我们代码中使用的公式, 只需要一次乘法和一次加法,
 *  计算量极小, 非常适合在 STM32F103 上实时运行.
 *
 *  α (alpha) 的选择:
 *    α = τ / (τ + dt)
 *    其中 τ 是时间常数 (高通滤波器对陀螺漂移的截止时间).
 *
 *    常用值:
 *      α = 0.98, dt = 0.01s  → τ ≈ 0.49s (响应较快)
 *      α = 0.96, dt = 0.01s  → τ ≈ 0.24s (更快的响应)
 *      α = 0.99, dt = 0.01s  → τ ≈ 0.99s (更平滑但响应慢)
 *
 *  实际调参建议:
 *    先设 α = 0.98, 晃一晃传感器看响应.
 *    如果角度"反应迟钝" → 减小 α (比如 0.96)
 *    如果角度"太抖"     → 增大 α (比如 0.99)
 *
 *  ★ 加速度计求角度原理:
 *    静止时加速度计只受重力, 三个轴读到的分量就是重力在
 *    传感器坐标系上的投影. 通过反正切可以反算出倾斜角度.
 *
 *    pitch = arctan(ax / sqrt(ay² + az²))
 *    roll  = arctan(ay / sqrt(ax² + az²))
 *
 *    注意: 这里用的是原始值 (int16_t), 不需要先转换成 g!
 *    因为 atan2 只关心比值, 分子分母同除以 scale 结果不变.
 *    直接用原始值可以省掉浮点除法, 在 STM32F103 上更快.
 * ============================================================
 */

/* ============================================================
 *             初 始 化 滤 波 器
 * ============================================================
 */
/**
 * @brief  初始化互补滤波器结构体
 * @param  f:     滤波器实例指针
 * @param  alpha: 滤波器系数 (典型值 0.96 ~ 0.99)
 * @param  dt:    采样间隔 (秒), 例如 0.01 表示 10ms 调用一次
 *
 *  初始角度设为 0, 即假设上电时传感器处于水平状态.
 *  如果上电时传感器是斜的, 第一次计算会自动收敛到正确角度,
 *  只是需要几个采样周期的时间.
 */
void Filter_Init(ComplementaryFilter *f, float alpha, float dt)
{
    f->angle = 0.0f;     /* 初始角度 0° */
    f->bias  = 0.0f;     /* 零偏 0 (暂未使用) */
    f->alpha = alpha;    /* 滤波器系数 */
    f->dt    = dt;       /* 采样间隔 */
}

/* ============================================================
 *      从 加 速 度 计 原 始 值 计 算 角 度
 * ============================================================
 */
/**
 * @brief  用加速度计原始值计算 pitch 和 roll
 * @param  ax: 加速度 X 轴原始值 (int16_t, 范围 -32768~32767)
 * @param  ay: 加速度 Y 轴原始值
 * @param  az: 加速度 Z 轴原始值
 * @param  pitch: [输出] 俯仰角 (°)
 * @param  roll:  [输出] 横滚角 (°)
 *
 *  数学原理:
 *
 *    当传感器静止时, 加速度计只测量重力加速度 g.
 *    重力在传感器 X/Y/Z 三个轴上的分量分别是 ax, ay, az.
 *
 *    传感器水平放置: ax≈0, ay≈0, az≈+1g → pitch=0°, roll=0°
 *    传感器前倾 30°: ax≠0 (有分量), az 变小 → pitch≈30°
 *    传感器右倾 30°: ay≠0 (有分量), az 变小 → roll≈30°
 *
 *  公式推导:
 *    pitch (绕 Y 轴): 重力在 XZ 平面的角度
 *      pitch = arctan(ax / az)   ← 简化版
 *      pitch = arctan(ax / sqrt(ay² + az²))  ← 完整版, 更准确
 *
 *    roll (绕 X 轴):  重力在 YZ 平面的角度
 *      roll = arctan(ay / sqrt(ax² + az²))
 *
 *  atan2f(y, x) 返回值范围: -π ~ +π (弧度)
 *  乘以 180/π = 57.29578 转换为角度.
 *
 *  使用 atan2f 而不是 atanf 的好处:
 *    atan2f 自动处理分母为 0 的情况 (返回 ±90°),
 *    而且返回值覆盖整个 -180°~180° 范围.
 *
 *  为什么直接用原始值而不先转换为 g?
 *    因为 atan2f(ax/2048, az/2048) = atan2f(ax, az),
 *    缩放因子被约掉了! 直接算还省了除法运算.
 */
void Filter_AccelAngle(int16_t ax, int16_t ay, int16_t az,
                       float *pitch, float *roll)
{
    /*
     * 注意: atan2f 只接受 float/double 参数,
     * 传入 int16_t 会被自动转为 float.
     * 对于没有硬件 FPU 的 STM32F103, 浮点运算由编译器
     * 插入软件浮点库完成, 速度较慢但精度足够.
     */

    /* pitch: arctan(ax / sqrt(ay² + az²)) */
    *pitch = atan2f((float)ax,
                    sqrtf((float)ay * ay + (float)az * az))
             * 57.29578f;   /* 弧度 → 度: 180 / PI */

    /* roll: arctan(ay / sqrt(ax² + az²)) */
    *roll  = atan2f((float)ay,
                    sqrtf((float)ax * ax + (float)az * az))
             * 57.29578f;
}

/* ============================================================
 *          互 补 滤 波 单 步 更 新
 * ============================================================
 */
/**
 * @brief  对单个轴执行一次互补滤波更新
 * @param  f:           滤波器实例
 * @param  gyro_rate:   陀螺仪角速度 (°/s), 注意符号!
 * @param  accel_angle: 加速度计推算的角度 (°)
 * @return 融合后的角度 (°)
 *
 *  核心公式 (就一行!):
 *    angle = α × (angle + gyro × dt) + (1 - α) × accel_angle
 *
 *  拆解理解:
 *    第 1 项: α × (angle + gyro × dt)
 *            上次角度 + 陀螺仪积分增量 = 陀螺仪预测的新角度
 *            乘 α → 大部分相信陀螺仪
 *
 *    第 2 项: (1 - α) × accel_angle
 *            加速度计直接测到的角度
 *            乘 (1-α) → 小部分相信加速度计来"拉回"漂移
 *
 *  为什么能抑制漂移?
 *    假设 α = 0.98:
 *      - 每次迭代, 融合结果中保留 98% 的陀螺仪预测 + 2% 的加速度计读数
 *      - 如果陀螺仪积分一直在漂移 (比如每 10ms 漂 0.01°),
 *        加速度计每次"纠正" 2% 的误差
 *      - 经过几十次迭代后, 漂移被拉回到真值附近
 *
 *  时间复杂度: O(1), 就是 3 次乘法 + 2 次加法.
 *  在 72MHz STM32 上仅需约 50µs, 完全不影响实时性.
 */
float Filter_Update(ComplementaryFilter *f, float gyro_rate, float accel_angle)
{
    /*
     * 公式: angle = α * (angle + gyro * dt) + (1-α) * accel_angle
     *
     * 注意: 括号不能省!
     *   angle + gyro * dt  先算乘法(gyro积分), 再加到旧角度
     *   然后整体乘 α
     */
    f->angle = f->alpha * (f->angle + gyro_rate * f->dt)
             + (1.0f - f->alpha) * accel_angle;

    return f->angle;
}

/* ============================================================
 *       一 次 性 更 新 pitch 和 roll (便 捷 函 数)
 * ============================================================
 */
/**
 * @brief  同时更新 pitch 和 roll 的滤波结果
 * @param  pitch_f:    pitch 滤波器实例
 * @param  roll_f:     roll 滤波器实例
 * @param  gx,gy,gz:   陀螺仪三轴角速度 (°/s)
 * @param  acc_pitch:  加速度计算出的 pitch (°)
 * @param  acc_roll:   加速度计算出的 roll (°)
 * @param  out:        [输出] 融合后的姿态角
 *
 *  注意: 陀螺仪的轴和角度的对应关系:
 *    绕 X 轴的角速度 (gx) → roll 的变化率
 *    绕 Y 轴的角速度 (gy) → pitch 的变化率
 *    绕 Z 轴的角速度 (gz) → yaw 的变化率 (本工程不使用)
 *
 *  为什么 gy 对应 pitch 而不是 gx?
 *    pitch (俯仰) 是绕 Y 轴旋转 → 所以用 Y 轴陀螺仪
 *    roll  (横滚) 是绕 X 轴旋转 → 所以用 X 轴陀螺仪
 *    别搞混! 搞混了角度会飞.
 */
void Filter_UpdateAttitude(ComplementaryFilter *pitch_f, ComplementaryFilter *roll_f,
                           float gx, float gy, float gz,
                           float acc_pitch, float acc_roll,
                           AttitudeAngle *out)
{
    /*
     * 关键对应关系 (务必牢记!):
     *   pitch ← 绕 Y 轴旋转 ← gyro_y ← gy
     *   roll  ← 绕 X 轴旋转 ← gyro_x ← gx
     *
     * 符号约定:
     *   如果滤波后的角度方向和预期相反, 在传入 gyro_rate 前加负号即可.
     *   MPU6050 的默认方向见数据手册坐标图.
     */
    out->pitch = Filter_Update(pitch_f, gy, acc_pitch);
    out->roll  = Filter_Update(roll_f,  gx, acc_roll);

    /* yaw 不使用, 保持为 0 */
    out->yaw = 0.0f;

    /*
     * 注意: gz 参数显式传入但未使用, 明确标记意图:
     * "我知道有 yaw 角速度, 但我选择不使用它"
     * 这是为了防止将来的开发者疑惑"为什么没有 gz".
     */
    (void)gz;
}
