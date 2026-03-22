#include "Copter.h"

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{

    //// 初始化垂直位置控制器
    if (!pos_control->is_active_U())
    {
        pos_control->init_U_controller();
    } // U轴：无人机垂直轴，对应上下方向

    pos_control->set_max_speed_accel_U_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_U_mss());        // pos_control位置控制器对象的指针（飞控核心模块之一），负责处理无人机位置闭环控制（比如根据期望位置计算需要的速度、姿态指令），是多旋翼 / 固定翼飞控中位置控制的核心实例
    pos_control->set_correction_speed_accel_U_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_U_mss()); // set（设置） + max_speed_accel（最大速度 / 加速度） + U_m（U=User，用户 / 飞行员配置；m=meter，米制单位）
                                                                                                                            // get_pilot_speed_dn_ms()：获取飞行员配置的「最大下降速度」，单位 m/s（米 / 秒）
                                                                                                                            // get_pilot_speed_up_ms()：获取飞行员配置的「最大上升速度」，单位 m/s
                                                                                                                            // get_pilot_accel_U_mss()：获取飞行员配置的「最大加速度」，单位 m/s²（米 / 二次方秒）
                                                                                                                            // 命名拆解：correction（修正）是核心差异 —— 这里的「修正运动」指无人机自动位置纠偏（比如悬停时被风吹偏、GPS 定位漂移导致位置偏离期望点，飞控自动回位的过程）
                                                                                                                            // 功能：限制飞控自动修正位置时的最大速度 / 加速度，确保纠偏过程平稳、不超调（比如不会因为修正太猛导致飞行器晃动）
    return true; // 定高模式激活前的初始化，返回true表示初始化成功
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_U_mss());

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll_rad, target_pitch_rad; // 没有赋初始值，后面函数通过引用赋值
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // get pilot's desired yaw rate
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // get pilot desired climb rate
    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();                                                  // 获取飞手的期望爬升速率
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms()); // 对飞手的期望速率限制，不允许超过的限制的最快速率

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state_U_ms(target_climb_rate_ms);

    // Alt Hold State Machine
    switch (althold_state)
    {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f); // U 控制器：飞控的坐标系中，U 轴对应垂直方向（Z 轴）的控制输出（也就是油门 /throttle），所以 “U 控制器” 本质是 “油门控制器”，负责根据高度 / 位置误差计算需要输出的油门大小。
                                               // relax：字面是 “放松、释放”，不是 “强制清零”，而是让控制器解除闭环控制，让油门输出以 “缓降” 的方式趋近目标值，核心是 “平滑过渡”。
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:                      // 起飞准备阶段被取消 / 中止，飞控需要从 “准备起飞” 状态平稳回退到 “电机停转” 的落地状态
        attitude_control->reset_rate_controller_I_terms_smoothly(); // reset_rate_controller_I_terms_smoothly()//平滑重置 姿态速率控制器的积分项（I 项），而非瞬间清零
        pos_control->relax_U_controller(0.0f);
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running())
        {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01, 0.0, 10.0)); // 全局参数（g 代表 global），存储飞行员设置的起飞目标高度，单位是 厘米（cm）
        } //* 0.01将高度单位从 “厘米” 转换为 “米（m）”
        /*
         对转换后的高度做安全约束：
             下限 0.0：防止高度为负数（无效高度，比如飞行员误设负值）；
             上限 10.0：限制最大起飞高度为 10 米（安全设计，避免误设过高高度导致失控、撞障）；
        */

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms); // 将原本的目标爬升 / 下降速率（比如飞行员指令、定高逻辑计算的速率）传入避障模块，由避障系统根据实时障碍物信息调整速率，最终得到 “避障优先” 的安全速率

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_AVOIDANCE_ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad());
#endif

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_U_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // run the vertical position controller and set output throttle
    pos_control->update_U_controller(); // 执行油门控制器的闭环 PID 计算，实时输出符合高度控制需求的油门值，是垂直方向稳定飞行的核心
}
