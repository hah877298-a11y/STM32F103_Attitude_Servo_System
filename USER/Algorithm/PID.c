#include "PID.h"

/* ============================================================
 *          位 置 式 PID 控 制 器 实 现
 * ============================================================
 *
 *  位置式 PID 公式:
 *    e(k) = setpoint - measurement               // 当前偏差
 *    integral += e(k) * dt                        // 积分累加
 *    derivative = (e(k) - e(k-1)) / dt            // 微分 (偏差变化率)
 *    output = Kp·e(k) + Ki·integral + Kd·derivative
 *
 *  公式拆解:
 *    ┌──────────────────────────────────────────────────┐
 *    │ P 项 = Kp × e(k)                                 │
 *    │   偏差 10° → P输出 = 10Kp,  偏差 1° → P输出 = 1Kp│
 *    │   P 越大响应越快, 但太大就振荡                    │
 *    ├──────────────────────────────────────────────────┤
 *    │ I 项 = Ki × integral                             │
 *    │   如果一直有 1° 的偏差, integral 每秒 +0.01      │
 *    │   I 越大消除静差越快, 但太大就超调               │
 *    ├──────────────────────────────────────────────────┤
 *    │ D 项 = Kd × (e(k) - e(k-1)) / dt                 │
 *    │   偏差变化越快, D 输出越大(反向), 像"刹车"       │
 *    │   D 越大阻尼越强, 但太大会放大噪声               │
 *    └──────────────────────────────────────────────────┘
 *
 *  ★ 抗积分饱和 (Anti-Windup) 详解:
 *
 *  场景: 设定角度为 90°, 但舵机被人用手按住, 转不过去.
 *        → 偏差持续为正
 *        → 积分项不断累加, 越来越大
 *        → 输出被钳位到最大值 180°
 *        → 当手松开后, 积分项已经累积到几百甚至几千
 *        → 即使传感器角度已经是 90°, 积分项还要反向"放电"很久
 *        → 导致严重的超调和长时间振荡!
 *
 *  解决方法: 积分限幅 (Clamping)
 *    限制 |integral| ≤ integral_limit
 *    一旦积分超过限幅, 停止继续累加, 保持当前值.
 *
 *  如何选取 integral_limit?
 *    通常设为: integral_limit = 输出范围 / (2 × Ki)
 *    例: 输出范围 180°, Ki=0.1 → integral_limit = 180/0.2 = 900
 *    但更实用的方法: 设为 Ki 发挥"正常作用"时的积分值
 *    比如: 偏差 5° 持续 1 秒时积分 = 5×1×Ki = 5Ki
 *    若 Ki=0.1, 则 integral_limit = 0.5~1.0 够用
 *
 *  实际建议:
 *    先设小的 integral_limit (如 5.0), 观察是否有静差.
 *    有静差 → 增大 integral_limit (如 10.0, 20.0)
 *    超调大 → 减小 integral_limit (如 2.0, 1.0)
 * ============================================================
 */

/* ============================================================
 *              初 始 化 PID 控 制 器
 * ============================================================
 */
/**
 * @brief  初始化 PID 控制器的状态
 * @param  state:           PID 状态结构体
 * @param  integral_limit:  积分限幅 (>0)
 * @param  output_min:      输出下限 (如 0°)
 * @param  output_max:      输出上限 (如 180°)
 *
 *  调用示例:
 *    PID_State pid_servo;
 *    PID_Init(&pid_servo, 10.0f, 0.0f, 180.0f);
 *    // 积分上限 ±10, 输出 0~180°
 */
void PID_Init(PID_State *state, float integral_limit,
              float output_min, float output_max)
{
    state->setpoint       = 0.0f;
    state->prev_error     = 0.0f;
    state->integral       = 0.0f;
    state->integral_limit = integral_limit;
    state->output_min     = output_min;
    state->output_max     = output_max;
}

/* ============================================================
 *            PID 计 算 (每 次 调 用 输 出 一 个 值)
 * ============================================================
 */
/**
 * @brief  执行一次 PID 计算
 * @param  state:       PID 状态 (包含积分、上次偏差等)
 * @param  params:      PID 参数 (Kp, Ki, Kd)
 * @param  measurement: 当前测量值 (如传感器读到的角度)
 * @param  dt:          距离上次计算的时间间隔 (秒)
 * @return PID 输出值 (已被钳位到 [output_min, output_max])
 *
 *  完整的计算流程:
 *
 *  第 1 步: 计算当前偏差
 *    error = setpoint - measurement
 *    例: 目标 90°, 当前 75° → error = +15° (说明还没转到, 需要继续转)
 *        目标 0°,  当前 30° → error = -30° (说明转过头了, 需要回调)
 *
 *  第 2 步: 积分累加 + 抗饱和
 *    integral += error × dt
 *    如果 |integral| > integral_limit → 钳位
 *
 *  第 3 步: 微分计算
 *    derivative = (error - prev_error) / dt
 *    注意: dt 不能为 0 (第一次调用时 prev_error=error, derivative=0)
 *
 *  第 4 步: 输出 = P + I + D
 *  第 5 步: 输出钳位
 *    output = clamp(output, output_min, output_max)
 *
 *  第 6 步: 保存 error 供下次使用
 */
float PID_Compute(PID_State *state, const PID_Params *params,
                  float measurement, float dt)
{
    float error, derivative, output;

    /* ===== 第 1 步: 计算偏差 ===== */
    error = state->setpoint - measurement;

    /* ===== 第 2 步: 积分累加 + 抗饱和 ===== */
    state->integral += error * dt;

    /* 积分限幅 (抗饱和的关键!) */
    if (state->integral > state->integral_limit)
    {
        state->integral = state->integral_limit;
    }
    else if (state->integral < -state->integral_limit)
    {
        state->integral = -state->integral_limit;
    }

    /* ===== 第 3 步: 微分计算 ===== */
    /*
     * 微分 = (本次偏差 - 上次偏差) / 时间间隔
     *
     * 物理意义: 偏差变化的速率.
     *   如果偏差在快速变小 → derivative < 0 → D 输出为负 → 减小总输出 → "刹车"
     *   如果偏差在快速变大 → derivative > 0 → D 输出为正 → 增大总输出 → "加速"
     *
     * D 项的"刹车"效应可以防止因 P 项过大导致的超调和振荡.
     *
     * 但 D 项也有问题:
     *   如果传感器有噪声 (比如 ±0.5° 的随机抖动),
     *   微分的"变化率"会被放大 → 导致输出抖动 → 舵机滋滋响.
     *   解决方案: 在计算 D 之前对测量值做低通滤波 (本例未实现, 供扩展).
     */
    if (dt > 0.0001f)  /* 防止除以 0 (dt 太小视为无效) */
    {
        derivative = (error - state->prev_error) / dt;
    }
    else
    {
        derivative = 0.0f;
    }

    /* ===== 第 4 步: 综合输出 ===== */
    /*
     *   P               I                  D
     *   ↑               ↑                  ↑
     * Kp × 当前偏差  +  Ki × 累积偏差  +  Kd × 偏差变化率
     */
    output = params->Kp * error
           + params->Ki * state->integral
           + params->Kd * derivative;

    /* ===== 第 5 步: 输出钳位 ===== */
    /*
     * 限制输出范围, 防止舵机收到无效角度指令.
     * 注意: 这里只钳位最终的 output, 不修改积分项!
     * 上面的积分限幅已经处理了积分饱和问题.
     */
    if (output > state->output_max)
    {
        output = state->output_max;
    }
    else if (output < state->output_min)
    {
        output = state->output_min;
    }

    /* ===== 第 6 步: 保存偏差, 供下次微分计算 ===== */
    state->prev_error = error;

    return output;
}

/* ============================================================
 *              设 置 目 标 值
 * ============================================================
 */
/**
 * @brief  修改 PID 的目标值 (设定值)
 * @param  state:    PID 状态
 * @param  setpoint: 新的目标值
 *
 *  改变目标值时, 建议同时复位积分项,
 *  否则旧的积分累积会影响新的目标跟踪.
 *  如果需要在运行时平滑切换目标值, 可以不清积分.
 *
 *  面试考点: 为什么改变设定值时要清积分?
 *    → 积分记录的是"之前偏差的累积", 换了目标后旧积分不适用
 *    → 不清积分会导致过冲 (旧积分还在"推动"输出朝原来的方向)
 */
void PID_SetSetpoint(PID_State *state, float setpoint)
{
    state->setpoint = setpoint;
    /* 切换目标时清积分, 防止过冲 */
    state->integral = 0.0f;
}

/* ============================================================
 *              复 位 PID (紧 急 停 止)
 * ============================================================
 */
/**
 * @brief  完全复位 PID 状态
 *
 *  调用场景:
 *    - 紧急停止 (如传感器读数异常)
 *    - 切换控制模式
 *    - 重新开始一个新的控制周期
 */
void PID_Reset(PID_State *state)
{
    state->prev_error = 0.0f;
    state->integral   = 0.0f;
    /* setpoint 保留, 复位后仍向同一个目标值调节 */
}
