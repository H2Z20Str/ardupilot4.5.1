#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}


// PID控制算法实现 2024.05.30
float ModeManual::PID_realize(PID *pid, float speed) {
    pid->SetSpeed=0.001;
    pid->ActualSpeed = speed;
    pid->err = pid->SetSpeed - pid->ActualSpeed;
    float incrementSpeed = pid->Kp * (pid->err - pid->err_next) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
    pid->ActualSpeed += incrementSpeed;
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    return pid->ActualSpeed;
}

PID pid;
int32_t yaw_old=0;
//class ParametersG3 g3;

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // apply manual steering expo
    desired_steering = 4500.0 * input_expo(desired_steering / 4500, g2.manual_steering_expo);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }

    // walking robots support roll, pitch and walking_height
    float desired_roll, desired_pitch, desired_walking_height;
    get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
    get_pilot_desired_walking_height(desired_walking_height);
    g2.motors.set_roll(desired_roll);
    g2.motors.set_pitch(desired_pitch);
    g2.motors.set_walking_height(desired_walking_height);

    // set sailboat sails
    float desired_mainsail;
    float desired_wingsail;
    float desired_mast_rotation;
    g2.sailboat.get_pilot_desired_mainsail(desired_mainsail, desired_wingsail, desired_mast_rotation);
    g2.motors.set_mainsail(desired_mainsail);
    g2.motors.set_wingsail(desired_wingsail);
    g2.motors.set_mast_rotation(desired_wingsail);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));
    g2.motors.set_lateral(desired_lateral);

    //手动下直线校准程序 2024.05.30 hzz
    int ch6_pwm=RC_Channels::rc_channel(CH_6)->get_radio_in(); //读方向遥杆
    if(ch6_pwm>=1800)
    {
        if(desired_throttle>0)
        {
            int yaw_change=0;
            pid.Kp = g2.velocity_Kp;
            pid.Ki = g2.velocity_Ki;
            pid.Kd = g2.velocity_Kd;
            if((int)desired_steering==0)
                {
                    if(yaw_old<9000 && AP::ahrs().yaw_sensor>27000)
                    {
                        yaw_change=AP::ahrs().yaw_sensor-yaw_old-36000;
                    }
                    else if(yaw_old>27000 && AP::ahrs().yaw_sensor<9000)
                    {
                        yaw_change=AP::ahrs().yaw_sensor-yaw_old+36000;
                    }
                    else yaw_change=AP::ahrs().yaw_sensor-yaw_old;//角度变化值

                    float st=0-yaw_change/10000.0;
                    st = PID_realize(&pid, st);
 //                   gcs().send_text(MAV_SEVERITY_CRITICAL, " 22 yaw_change=%d,st=%f",yaw_change,st);
                    calc_steering_from_turn_rate(st);

                }
            else yaw_old=AP::ahrs().yaw_sensor;
        }
        else yaw_old=AP::ahrs().yaw_sensor;
    }
    else yaw_old=AP::ahrs().yaw_sensor;
}
