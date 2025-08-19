#include "Rover.h"

/*
  allow for runtime change of control channel ordering
 */
void Rover::set_control_channels(void)
{
    // check change on RCMAP
    channel_steer    = rc().channel(rcmap.roll()-1);
    channel_throttle = rc().channel(rcmap.throttle()-1);
    channel_lateral  = rc().channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_steer->set_angle(SERVO_MAX);
    channel_throttle->set_angle(100);
    if (channel_lateral != nullptr) {
        channel_lateral->set_angle(100);
    }

    // walking robots rc input init
    channel_roll = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ROLL);
    channel_pitch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::PITCH);
    channel_walking_height = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WALKING_HEIGHT);
    if (channel_roll != nullptr) {
        channel_roll->set_angle(SERVO_MAX);
        channel_roll->set_default_dead_zone(30);
    }
    if (channel_pitch != nullptr) {
        channel_pitch->set_angle(SERVO_MAX);
        channel_pitch->set_default_dead_zone(30);
    }
    if (channel_walking_height != nullptr) {
        channel_walking_height->set_angle(SERVO_MAX);
        channel_walking_height->set_default_dead_zone(30);
    }    

    // sailboat rc input init
    g2.sailboat.init_rc_in();

    // Allow to reconfigure output when not armed
    if (!arming.is_armed()) {
        g2.motors.setup_servo_output();
        // For a rover safety is TRIM throttle
        g2.motors.setup_safety_output();
    }
    // setup correct scaling for ESCs like the UAVCAN ESCs which
    // take a proportion of speed. Default to 1000 to 2000 for systems without
    // a k_throttle output
    hal.rcout->set_esc_scaling(1000, 2000);
    g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
}

void Rover::init_rc_in()
{
    // set rc dead zones
    channel_steer->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    if (channel_lateral != nullptr) {
        channel_lateral->set_default_dead_zone(30);
    }
}

/*
  check for driver input on rudder/steering stick for arming/disarming
*/
void Rover::rudder_arm_disarm_check()
{
    // check if arming/disarm using rudder is allowed
    const AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        return;
    }

    // In Rover we need to check that its set to the throttle trim and within the DZ
    // if throttle is not within trim dz, then pilot cannot rudder arm/disarm
    if (!channel_throttle->in_trim_dz()) {
        rudder_arm_timer = 0;
        return;
    }

    // check if arming/disarming allowed from this mode
    if (!control_mode->allows_arming_from_transmitter()) {
        rudder_arm_timer = 0;
        return;
    }

    if (!arming.is_armed()) {
        // when not armed, full right rudder starts arming counter
        if (channel_steer->get_control_in() > 4000) {
            const uint32_t now = millis();

            if (rudder_arm_timer == 0 ||
                now - rudder_arm_timer < ARM_DELAY_MS) {
                if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
            } else {
                // time to arm!
                arming.arm(AP_Arming::Method::RUDDER);
                rudder_arm_timer = 0;
            }
        } else {
            // not at full right rudder
            rudder_arm_timer = 0;
        }
    } else if ((arming_rudder == AP_Arming::RudderArming::ARMDISARM) && !g2.motors.active()) {
        // when armed and motor not active (not moving), full left rudder starts disarming counter
        if (channel_steer->get_control_in() < -4000) {
            const uint32_t now = millis();

            if (rudder_arm_timer == 0 ||
                now - rudder_arm_timer < ARM_DELAY_MS) {
                if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
            } else {
                // time to disarm!
                arming.disarm(AP_Arming::Method::RUDDER);
                rudder_arm_timer = 0;
            }
        } else {
            // not at full left rudder
            rudder_arm_timer = 0;
        }
    }
}

extern int ch1_pwm,ch2_pwm;
extern char Manual_2,Manual_3;
extern int ch3_pwm_max;
char Manual_3_v=0;
extern unsigned char boat_model;
void Rover::read_radio()
{
    if (!rc().read_input()) {
        // check if we lost RC link
        radio_failsafe_check(channel_throttle->get_radio_in());
        return;
    }

    failsafe.last_valid_rc_ms = AP_HAL::millis();
    // check that RC value are valid
    radio_failsafe_check(channel_throttle->get_radio_in());

    // check if we try to do RC arm/disarm
    rudder_arm_disarm_check();

     ////south h2z 2023.12.28///

    if (control_mode->mode_number()== Mode::Number::MANUAL) //手动模式
       {
           
           hal.util->ch5_pwm=RC_Channels::rc_channel(CH_5)->get_radio_in(); //读取速度档位     
            {
             //   yaw_old=AP::ahrs().yaw_sensor;
                if(hal.util->ch5_pwm<1200) //低速
                {
                  if(boat_model==20)//20船
                  {
                    if(g3.velocity_min_1>1500)
                        hal.util->pwm_out1=(int16_t)g3.velocity_min_1;
                    else hal.util->pwm_out1=1650;
                    if(g3.velocity_min_2>1500)
                        hal.util->pwm_out2=(int16_t)g3.velocity_min_2;
                    else hal.util->pwm_out2=1650;
                   }
                  else //30船
                  {
                      if(g3.velocity30_min_1>1500)
                          hal.util->pwm_out1=(int16_t)g3.velocity30_min_1;
                      else hal.util->pwm_out1=1640;
                      if(g3.velocity30_min_2>1500)
                          hal.util->pwm_out2=(int16_t)g3.velocity30_min_2;
                      else hal.util->pwm_out2=1640;
                  }


                    if(Manual_3==2)
                    {
                        if(ch3_pwm_max>g3.velocity_min_1)ch3_pwm_max=g3.velocity_min_1;
                    }
                    Manual_3_v=(Manual_3_v&0xf0)|(0x01<<0);
                }
                else if(hal.util->ch5_pwm<1600) //中速
                {
                  if(boat_model==20)//20船
                  {
                    if(g3.velocity_trim_1>1500)
                        hal.util->pwm_out1=(int16_t)g3.velocity_trim_1;
                    else hal.util->pwm_out1=1750;
                    if(g3.velocity_trim_2>1500)
                        hal.util->pwm_out2=(int16_t)g3.velocity_trim_2;
                    else hal.util->pwm_out2=1750;
                  }
                  else
                  {
                      if(g3.velocity30_trim_1>1500)
                          hal.util->pwm_out1=(int16_t)g3.velocity30_trim_1;
                      else hal.util->pwm_out1=1680;
                      if(g3.velocity30_trim_2>1500)
                          hal.util->pwm_out2=(int16_t)g3.velocity30_trim_2;
                      else hal.util->pwm_out2=1680;
                  }
                    if(Manual_3==2)
                    {
                        if(ch3_pwm_max>g3.velocity_trim_1)ch3_pwm_max=g3.velocity_trim_1;
                    }
                    Manual_3_v=(Manual_3_v&0xf0)|(0x01<<1);
                }
                else if(hal.util->ch5_pwm<2000)
                {
                  if(boat_model==20)//20船
                  {
                    if(g3.velocity_max_1>1500)
                        hal.util->pwm_out1=(int16_t)g3.velocity_max_1;
                    else hal.util->pwm_out1=1800;
                    if(g3.velocity_max_2>1500)
                        hal.util->pwm_out2=(int16_t)g3.velocity_max_2;
                    else hal.util->pwm_out2=1800;
                  }
                  else
                  {
                      if(g3.velocity30_max_1>1500)
                          hal.util->pwm_out1=(int16_t)g3.velocity30_max_1;
                      else hal.util->pwm_out1=1800;
                      if(g3.velocity30_max_2>1500)
                          hal.util->pwm_out2=(int16_t)g3.velocity30_max_2;
                      else hal.util->pwm_out2=1800;
                  }
                    if(Manual_3==2)
                    {
                        if(ch3_pwm_max>g3.velocity_max_1)ch3_pwm_max=g3.velocity_max_1;
                    }
                    Manual_3_v=(Manual_3_v&0xf0)|(0x01<<2);
                }
                if(Manual_2==2)
                {
                    hal.util->pwm_out1=ch1_pwm;
                    hal.util->pwm_out2=ch2_pwm;
                }
                if(Manual_3==1) //定速巡航根据档位增减速度
               {
                    hal.util->pwm_out1=hal.util->pwm_out2=ch3_pwm_max;

                    int ch14_pwm=RC_Channels::rc_channel(CH_14)->get_radio_in();
                    int32_t ms_now = AP_HAL::millis();//获取现在的时间
                    static int32_t ms_old1=0;
                    if((ch14_pwm>1900&&(Manual_3_v&0xf0)==0x00)||((ms_now-ms_old1)>500&&(Manual_3_v&0xf0)==0x10))
                    {
                        ms_old1=ms_now;
                        Manual_3_v=(Manual_3_v&0x0f)|0x10;
                        ch3_pwm_max+=(Manual_3_v&0x0f)*10;
                        if(ch3_pwm_max>g3.velocity_max_1)ch3_pwm_max=g3.velocity_max_1;
                    }
                    else if((ch14_pwm<1200&&(Manual_3_v&0xf0)==0x00)||((ms_now-ms_old1)>500&&(Manual_3_v&0xf0)==0x20))
                    {
                        ms_old1=ms_now;
                        Manual_3_v=(Manual_3_v&0x0f)|0x20;
                        ch3_pwm_max -=(Manual_3_v&0x0f)*10;
                        if(ch3_pwm_max<1550)ch3_pwm_max=1550;
                    }
                    else if(ch14_pwm<1550&&ch14_pwm>1450)
                    {
                        ms_old1=ms_now;
                        Manual_3_v=Manual_3_v&0x0f;
                    }

                   // gcs().send_text(MAV_SEVERITY_CRITICAL, " st=0x%02x",Manual_3_v);
                            //(int)(1500+(RC_Channels::rc_channel(CH_11)->get_radio_in()-1050)/1.8); //读速度控制旋钮
                }
            }
       }
    else  //非手动时的pwm
        {
            //2m/s 1800 -1500=300  5m 2100-1500=600
            if(hal.util->auto_speed<=1) hal.util->auto_speed+=1;

          if(boat_model==20)//20船
          {
            if((hal.util->auto_speed*g3.velocity_auto_1+1500)>2100)
                hal.util->pwm_out1=2100;
            else hal.util->pwm_out1=hal.util->auto_speed*g3.velocity_auto_1 +1500 ;

            if((hal.util->auto_speed*g3.velocity_auto_2+1500)>2100)
               hal.util->pwm_out2=2100;
            else hal.util->pwm_out2=hal.util->auto_speed*g3.velocity_auto_2+1500;
          }
          else
          {
              if((hal.util->auto_speed*g3.velocity30_auto_1+1500)>2100)
                  hal.util->pwm_out1=2100;
              else hal.util->pwm_out1=hal.util->auto_speed*g3.velocity30_auto_1 +1500 ;

              if((hal.util->auto_speed*g3.velocity30_auto_2+1500)>2100)
                 hal.util->pwm_out2=2100;
              else hal.util->pwm_out2=hal.util->auto_speed*g3.velocity30_auto_2+1500;
          }
        }
//    gcs().send_text(MAV_SEVERITY_CRITICAL, "pwm1=%d，pwm2=%d",hal.util->pwm_out1,hal.util->pwm_out2);
}

void Rover::radio_failsafe_check(uint16_t pwm)
{
    if (!g.fs_throttle_enabled) {
        // radio failsafe disabled
        return;
    }

    bool failed = pwm < static_cast<uint16_t>(g.fs_throttle_value);
    if (AP_HAL::millis() - failsafe.last_valid_rc_ms > 500) {
        failed = true;
    }
    AP_Notify::flags.failsafe_radio = failed;
    failsafe_trigger(FAILSAFE_EVENT_THROTTLE, "Radio", failed);
}
