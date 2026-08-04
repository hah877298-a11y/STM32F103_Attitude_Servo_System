#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

/* ============================================================
 *          位 置 式 PID 控 制 器 头 文 件
 * ============================================================
 *
 *  PID 是最经典、最常用的闭环控制算法, 由三项组成:
 *
 *   P (Proportional) 比例: 根据"当前偏差"调节
 *     → 偏差越大, 输出越猛, 快速接近目标
 *     → 但纯比例控制会有"稳态误差" (永远差一点点到不了)
 *
 *   I (Integral)      积分: 根据"历史累积偏差"调节
 *     → 把过去的偏差全部累加起来, 消除稳态误差
 *     → 但如果积分太多会"超调"和"振荡"→ 需要抗积分饱和!
 *
 *   D (Derivative)    微分: 根据"偏差变化趋势"调节
 *     → 偏差变化越快, 阻尼越大, 抑制超调和振荡
 *     → 但对噪声敏感 (噪声的瞬间变化率很大)
 *
 *  通俗比喻——洗澡调水温:
 *    P: 水太凉 → 往热拧一大把  (比例)
 *    I: 怎么还是有点凉 → 再拧一点点 (积分, 消除那一点偏差)
 *    D: 水突然变烫了 → 赶紧往回调 (微分, 预判趋势)
 *
 *  位置式 vs 增量式 PID:
 *    位置式: u(k) = Kp·e(k) + Ki·Σe(i) + Kd·[e(k)-e(k-1)]
 *            输出 = 控制量的绝对值 (如"舵机转到 75°")
 *            适合: 舵机角度、阀门开度等需要绝对位置的控制
 *
 *    增量式: Δu(k) = u(k) - u(k-1)
 *            输出 = 控制量的变化量 (如"电机再转快一点")
 *            适合: 步进电机、加热功率等需要变化率的控制
 *
 *    本工程用位置式, 因为我们要告诉舵机"转到xx度"而不是"再转xx度".
 *
 *  采样周期注意事项:
 *    PID 的计算频率必须和传感器读取频率一致.
 *    如果传感器每 10ms 更新一次, PID 也每 10ms 计算一次.
 *    频率不匹配会导致积分项/微分项计算错误!
 *
 *  舵机场景的 PID 特殊考虑:
 *    - 舵机本身已经有内部的闭环控制 (电位器反馈)
 *    - 我们的 PID 是"外环"——根据 IMU 角度控制舵机
 *    - 这种 PID 输出的是"目标角度", 让舵机自己去执行
 *    - 舵机响应速度有限 (约 0.12s/60°), PID 频率不能太高
 * ============================================================
 */

/* ========== PID 参 数 结 构 体 ========== */
/**
 * @brief  PID 控制器的可调参数
 *
 *  调参口诀 (Ziegler-Nichols 经验法):
 *    1. 先把 Ki 和 Kd 设为 0, 只调 Kp
 *    2. 增大 Kp 直到系统开始振荡 (临界振荡)
 *    3. 记录临界 Kp 值 (Ku) 和振荡周期 (Tu)
 *    4. 按公式计算:
 *       P 控制:  Kp = 0.50 × Ku
 *       PI 控制: Kp = 0.45 × Ku,  Ki = 0.54 × Ku / Tu
 *       PID 控制:Kp = 0.60 × Ku,  Ki = 1.20 × Ku / Tu,  Kd = 0.075 × Ku × Tu
 *
 *  实际调参建议 (先粗后细):
 *    第 1 轮: Kp=1.0,  Ki=0,    Kd=0  → 看响应是否振荡
 *    第 2 轮: Kp=0.5,  Ki=0.01, Kd=0  → 看稳态误差是否消除
 *    第 3 轮: Kp=0.5,  Ki=0.02, Kd=0.1 → 看超调是否被抑制
 */
typedef struct
{
    float Kp;       /* 比例系数 (Proportional Gain) */
    float Ki;       /* 积分系数 (Integral Gain) */
    float Kd;       /* 微分系数 (Derivative Gain) */
} PID_Params;

/* ========== PID 状 态 结 构 体 ========== */
/**
 * @brief  PID 控制器运行状态 (每次计算都会更新)
 *
 *  为什么要区分参数和状态?
 *    参数 (Params): 调好之后一般不变, 可以共享给多个 PID 实例
 *    状态 (State):  每次计算都变, 每个 PID 实例独立维护
 *
 *  积分限幅 (integral_limit):
 *    积分项如果无限累积 → "积分饱和" (Integral Windup)
 *    → 输出被钳位后, 积分还在增加 → 反向时需要很长时间"放掉"
 *    → 导致严重的超调和振荡!
 *    解决方法: 限制积分项的绝对值.
 */
typedef struct
{
    float setpoint;         /* 目标值 (设定值) */
    float prev_error;       /* 上一次的偏差 (用于微分计算) */
    float integral;         /* 积分累积值 */
    float integral_limit;   /* 积分限幅 (正数) */
    float output_min;       /* 输出下限 */
    float output_max;       /* 输出上限 */
} PID_State;

/* ========== 对 外 接 口 ========== */

void PID_Init(PID_State *state, float integral_limit,
              float output_min, float output_max);

float PID_Compute(PID_State *state, const PID_Params *params,
                  float measurement, float dt);

void PID_SetSetpoint(PID_State *state, float setpoint);

void PID_Reset(PID_State *state);

#endif /* __PID_H */
