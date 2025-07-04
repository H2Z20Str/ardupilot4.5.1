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
int ch1_pwm=1500,ch2_pwm=1500;
char Manual_2=0,Manual_3=0;
int ch3_pwm_max=0;
extern float manual_speed;
char calibration_flag=0;
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
    g2.motors.set_throttle(desired_throttle); //油门
    g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));//转向
    g2.motors.set_lateral(desired_lateral);//横向


//    g2.motors.set_throttle(75); //油门，可指定让全面输出油门
    int ch10_pwm=RC_Channels::rc_channel(CH_10)->get_radio_in(); //读控制按键
    int ch11_pwm=RC_Channels::rc_channel(CH_11)->get_radio_in(); //读速度控制旋钮
    int ch3_pwm=RC_Channels::rc_channel(CH_3)->get_radio_in();//读取左油门
    int ch6_pwm=RC_Channels::rc_channel(CH_6)->get_radio_in(); //读直线校准遥杆
    //定速巡航 2024.12.05，start
    {

 //       gcs().send_text(MAV_SEVERITY_CRITICAL, " st=%df",ch3_pwm);
        //手动按照定速直线航行，仅控制作用，拨动油门摇杆刹车时退出定速
        if(ch10_pwm>=1890)//最高值时触发
        {
            hal.util->tip=5; //提示定速巡航模式
//
//            int32_t ms_now = AP_HAL::millis();//获取现在的时间
//            static int32_t ms_old1=0;
//            static int ch3_pwm_old=0;
            if(ch3_pwm>1550)//前进油门
            {
                Manual_3=2;//开启前摇
//                if(ch3_pwm>=ch3_pwm_max)
//                {
//                    ch3_pwm_max=ch3_pwm;//取最大值
//                }
                if(manual_speed>0.3)
                {
                    if(ch6_pwm>=1800)ch3_pwm_max=1500+manual_speed*g2.velocity_MV1;//取最大值
                    else ch3_pwm_max=1500+manual_speed*g2.velocity_MV2;//取最大值
                }
            }
            if(ch3_pwm<1450&&Manual_3==1)//后退油门
            {
                Manual_3=0;//关闭直线控制
                ch3_pwm_max=0;
            }
            if(ch3_pwm>1450&&ch3_pwm<1550&&Manual_3==2)//中值油门
            {
                Manual_3=1;//开启
            }
           // if((int)desired_throttle==0)//油门为0
            if(Manual_3==1)
            {
                g2.motors.set_throttle(100); //满油门输出
            }

        }
        else Manual_3=0;//开启直线控制
    }
    //定速巡航 2024.12.05，end

    //手动下直线校准程序 2024.05.30 hzz

    if(ch6_pwm>=1800)
    {
        if(calibration_flag==0)
        {   calibration_flag=1;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Currently in straight line calibration mode!!!");//提示当前为直线校准模式
        }
        //普通手动模式
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
                    calc_steering_from_turn_rate(st);//调整转向

                }
            else yaw_old=AP::ahrs().yaw_sensor;
        }
        else yaw_old=AP::ahrs().yaw_sensor;

        //双杆模式
        if(Manual_2==1)
        {
            yaw_old=AP::ahrs().yaw_sensor;
        }
    }
    else
        {
            yaw_old=AP::ahrs().yaw_sensor;
            if(calibration_flag==1)
            {
                calibration_flag=0;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Exit the straight line calibration mode!!!");//提示退出直线校准模式
            }

        }




    //左右对应左推进器，右油门对应右推进器
    if(ch10_pwm>1450&&ch10_pwm<1600)//中值触发
    {
        hal.util->tip=6; //提示双杆模式
        float b=(ch11_pwm-1050)/900.0;
        int s1=RC_Channels::rc_channel(CH_3)->get_radio_in();//读取左油门
        int s2=3000-RC_Channels::rc_channel(CH_2)->get_radio_in();//读取右油门
//         Manual_2=1;
        if(s1>1940&&s2>1940)
        {
            Manual_2=2;//退出双杆控制并开启直线控制
        }
        else  Manual_2=1;
        //添加速度控制
        ch1_pwm=(int)(1500+(s1-1500)*b);
        ch2_pwm=(int)(1500+(s2-1500)*b);


    }
    else  Manual_2=0;

//    if(ch6_pwm<1400)
//    {
//        g2.motors.set_throttle(-100); //满负油门输出
//    }

}
