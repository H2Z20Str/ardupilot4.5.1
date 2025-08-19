#include "Rover.h"

/**
 *
 * Detects failures of the ekf and triggers a failsafe 检测ekf的故障并触发故障保护
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif


// EKF_check structure
static struct {
    uint8_t fail_count;         // number of iterations ekf or dcm have been out of tolerances
    uint8_t bad_variance : 1;   // true if ekf should be considered untrusted (fail_count has exceeded EKF_CHECK_ITERATIONS_MAX)
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe 检测ekf方差是否超出容差并触发故障保护
// should be called at 10hz
extern char hdt_ber;
void Rover::ekf_check()
{
    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    // 如果ekf还没有原点，则立即退出-这假设原点永远不会被取消设置
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if motors are not armed, or ekf check is disabled
    //如果电机未启动或ekf检查被禁用，请立即返回
    if (!arming.is_armed() || (g.fs_ekf_thresh <= 0.0f)) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

//    if(hdt_ber==1)//HDT数据异常时清除故障保护
//    {
//       // static unsigned char time_sum=0;
//        ekf_check_state.fail_count = 0;
//        ekf_check_state.bad_variance = false;
//        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
//        failsafe_ekf_off_event();   // clear failsafe
//        //if(time_sum++>200)
//            gcs().send_text(MAV_SEVERITY_CRITICAL, "HDT Abnormal heading data!!！");//,time_sum=0;
//       // hal.util->tip=5; ///SAAS
//        return;
//    }

    // compare compass and velocity variance vs threshold 比较指南针和速度方差与阈值
    if (ekf_over_threshold()) {
        if(hdt_ber==1)//HDT数据异常时清除故障保护
        {
            static unsigned char time_sum=0;
            ekf_check_state.fail_count = 0;
            ekf_check_state.bad_variance = false;
            AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
            failsafe_ekf_off_event();   // clear failsafe
            if(time_sum++>250)
                gcs().send_text(MAV_SEVERITY_CRITICAL, "HDT Abnormal heading data!!！"),time_sum=0;
           // hal.util->tip=7;
            return;
        }
        // if compass is not yet flagged as bad 如果指南针还没有标记为坏
        if (!ekf_check_state.bad_variance) {
            // increase counter 递增计数器
            ekf_check_state.fail_count++;
            // if counter above max then trigger failsafe 如果计数器高于最大值，则触发故障保护
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high 上限计数过高
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;

                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK,
                                         LogErrorCode::EKFCHECK_BAD_VARIANCE);
                // send message to gcs
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter 减少计数器
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // if variance is flagged as bad and the counter reaches zero then clear flag 如果方差被标记为坏，计数器达到零，则清除标记
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK,
                                         LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags 设置AP_Notify标志
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
}

// returns true if the ekf's variance are over the tolerance
//如果ekf的方差超过容差，则返回true
bool Rover::ekf_over_threshold()
{
    // return false immediately if disabled 如果禁用，则立即返回false
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }
    // use EKF to get variance 使用EKF获得方差
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance);

    // return true if two of compass, velocity and position variances are over the threshold
    //如果指南针、速度和位置方差中的两个超过阈值，则返回true
    uint8_t over_thresh_count = 0;
    if (mag_variance.length() >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    if (position_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    bool optflow_healthy = false;
#if AP_OPTICALFLOW_ENABLED
    optflow_healthy = optflow.healthy();
#endif
    if (!optflow_healthy && (vel_variance >= (2.0f * g.fs_ekf_thresh))) {
        over_thresh_count += 2;
    } else if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }
    
    if (over_thresh_count >= 2) {
        return true;
    }
    return !ekf_position_ok();
}

// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
//ekf_position_ok-如果ekf声称其水平绝对位置估计正常并且设置了起始位置，则返回true
bool Rover::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) { //无HDT数据
        // do not allow navigation with dcm position  do not allow navigation with dcm position
        //不允许使用dcm位置导航
        return false;
    }

    // get EKF filter status
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    // if disarmed we accept a predicted horizontal absolute or relative position
    if (!arming.is_armed()) {
        return (filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs || filt_status.flags.horiz_pos_rel || filt_status.flags.pred_horiz_pos_rel);
    } else {//有HDT数据
        // once armed we require a good absolute or relative position and EKF must not be in const_pos_mode
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.horiz_pos_rel) && !filt_status.flags.const_pos_mode);
    }
}

// perform ekf failsafe 执行ekf故障保护
void Rover::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered 如果ekf故障保护已触发，请立即返回
    if (failsafe.ekf) {
        return;
    }

    // EKF failsafe event has occurred EKF故障保护事件已发生
    failsafe.ekf = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV,
                             LogErrorCode::FAILSAFE_OCCURRED);

    // does this mode require position? 这种模式需要定位吗？
    if (!control_mode->requires_position()) {
        return;
    }

    // take action based on fs_ekf_action parameter 根据fsekf_action参数采取行动
    switch ((enum fs_ekf_action)g.fs_ekf_action.get()) {
        case FS_EKF_DISABLE:
            // do nothing
            return;
        case FS_EKF_REPORT_ONLY:
            break;
        case FS_EKF_HOLD:
        default:
            set_mode(mode_hold, ModeReason::EKF_FAILSAFE);
            break;
    }

    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF failsafe");
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
//failsafe_ekf_off_event-清除ekf故障保护时要采取的操作
void Rover::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe 如果不在ekf故障保护中，请立即返回
    if (!failsafe.ekf) {
        return;
    }

    failsafe.ekf = false;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_EKFINAV,
                             LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF failsafe cleared");
}
