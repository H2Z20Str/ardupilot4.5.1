#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point 将_destination设置为合理的停止点
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed 将所需速度初始化为当前速度
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    return true;
}

// PID控制算法实现 2024.05.30
float ModeLoiter::PID_realizeloit(PID *pid, float speed) {
    //pid->SetSpeed=0.001;
    pid->ActualSpeed = speed;
    pid->err = pid->SetSpeed - pid->ActualSpeed;
    float incrementSpeed = pid->Kp * (pid->err - pid->err_next) + pid->Ki * pid->err + pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);
    pid->ActualSpeed += incrementSpeed;
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    return pid->ActualSpeed;
}

PID pid1,pid2;

int32_t lat1=0,lng1=0;
int32_t latlod=0,lnglod=0;
#define colation_sum 10
float destination_buf[colation_sum],yaw_buf[colation_sum];;
uint8_t  dest_i=0,yaw_i=0;

//extern float deep_water_1,deep_water_2,deep_water_3,deep_water_4,deep_water_5;
float des_old=0,des_lv=0,des_now=0,yaw_oldl=0,yaw_nowl=0,turn_rate1;

void ModeLoiter::update()
{
    // get distance (in meters) to destination 获取到目的地的距离（单位：米）
    _distance_to_destination = rover.current_loc.get_distance(_destination);
//    deep_water_1=_distance_to_destination;
    des_old=_distance_to_destination;

    //距离滤波
       //1、中位值递推平均滤波：取一组数据，去除最大最小值再算平均值
    if(dest_i==colation_sum)//已获取N个数据，实现滤波
      {
         int i=0;
         for(i=0;i<colation_sum-1;i++)//循环更新最新的数据
         {
             destination_buf[i] = destination_buf[i+1];
          }
         destination_buf[i]=_distance_to_destination;//更新最新的距离数据

         //求平均值滤波
         float sum=0,max=0,min=0;
         for(i=0;i<colation_sum;i++)
          {
             sum +=destination_buf[i];
             if(max<destination_buf[i])max=destination_buf[i];
             if(min>destination_buf[i])min=destination_buf[i];
          }
            //gcs().send_text(MAV_SEVERITY_CRITICAL, "buf_deeps[%d]=%f",i,buf_deeps[i]);;
            _distance_to_destination=(sum-max-min)/(colation_sum-2); //滤波后的距离数据
          }
         if(dest_i<colation_sum) //初始先获取N个数据
             destination_buf[dest_i++]=_distance_to_destination;

// deep_water_2=_distance_to_destination;
 des_lv=_distance_to_destination;
///////////////////////////////////////////////////////////////////

    if(hal.util->hzz_test[1]==9)//定点悬停测试
    {
        if(latlod!=lat1&&lnglod!=lng1)
        {
            _destination.lat=lat1;
            _destination.lng=lng1;
           // gcs().send_text(MAV_SEVERITY_CRITICAL, "悬停点：%d,%d",_destination.lat,_destination.lng);
        }
        latlod=lat1;
        lnglod=lng1;
    }
    // 0 turn rate is no limit 0转弯率没有限制
    float turn_rate = 0.0;

    const float loiter_radius = g2.wp_nav.get_radius();//直接取航点半径//rover.g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;

    // if within loiter radius slew desired speed towards zero and use existing desired heading 如果在巡航半径内，将所需速度转向零，并使用现有的所需航向
    if(hal.util->hzz_test[0]==8)
        gcs().send_text(MAV_SEVERITY_CRITICAL, "距离：%f,r;%f",_distance_to_destination,g2.wp_nav.get_radius());

    if (_distance_to_destination <= (loiter_radius+0.4)) {
        // sailboats should not stop unless motoring 帆船除非开着车，否则不应该停下来
        const float desired_speed_within_radius = rover.g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);
        //gcs().send_text(MAV_SEVERITY_CRITICAL, "速度：%f",_desired_speed);
        // if we have a sail but not trying to use it then point into the wind 如果我们有帆，但不想用它，那就指向风
        if (!rover.g2.sailboat.tack_enabled() && rover.g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }
        //////////再缩小一半范围
       if(_distance_to_destination >= (loiter_radius)*0.4)
        {


            pid1.SetSpeed=(loiter_radius)*0.4; //目标值
            pid1.Kp = g3.loiter_Kp;
            pid1.Ki = g3.loiter_Ki;
            pid1.Kd = g3.loiter_Kd;
            _distance_to_destination = PID_realizeloit(&pid1, _distance_to_destination);//pid
            des_now=_distance_to_destination;
           // gcs().send_text(MAV_SEVERITY_CRITICAL, "距离2：%f",_distance_to_destination);
            //这里相当于一个P环控制。将位置误差转成速度
            _desired_speed = MIN((_distance_to_destination - loiter_radius*0.4) * g2.loiter_speed_gain*2, g2.wp_nav.get_default_speed());
            // calculate bearing to destination 计算到达目的地的方位
            _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
            float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            //计算出yaw_error_cd以正前方为0，正后方为18000，顺时针为正，逆时针为负
            // if destination is behind vehicle, reverse towards it 如果目的地在车辆后面，则向其倒车
            if (fabsf(yaw_error_cd) > 15000 ) { //后方150度，小范围内允许倒车
                _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
                yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
                _desired_speed = -_desired_speed;
            }
            //如果偏航误差较大，则降低期望速度
            //45度的误差使速度降低到75%，90度的误差将速度降低到50%
            float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
            _desired_speed *= yaw_error_ratio;
            turn_rate=g3.loiter_turn_rate;//限制转弯速率

        }
    } else {
        // P controller with hard-coded gain to convert distance to desired speed P控制器，具有硬编码增益，可将距离转换为所需速度
        //根据距离悬停点的距离计算所需要的速度（计算速度，航行速度）
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());
        // calculate bearing to destination 计算到达目的地的方位,顺时针0-36000
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        //计算出yaw_error_cd以正前方为0，正后方为18000，顺时针为正，逆时针为负
        // if destination is behind vehicle, reverse towards it 如果目的地在车辆后面，则向其倒车
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
          //  gcs().send_text(MAV_SEVERITY_CRITICAL, "111");
        }

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        //如果偏航误差较大，则降低期望速度
        //45度的误差使速度降低到75%，90度的误差将速度降低到50%
        //这里的目的应该是优先转向，然后再驶入悬停点。
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
       // gcs().send_text(MAV_SEVERITY_CRITICAL, "c：%f,d：%f",_desired_yaw_cd,_desired_speed);
    }



    // make sure sailboats don't try and sail directly into the wind 确保帆船不要试图直接迎风航行
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
      //  gcs().send_text(MAV_SEVERITY_CRITICAL, "55");
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        if (g2.sailboat.tacking()) {
            // use pivot turn rate for tacks
            turn_rate = g2.wp_nav.get_pivot_rate();
           // gcs().send_text(MAV_SEVERITY_CRITICAL, "00");
        }
    }
    if(hal.util->hzz_test[0]==8)
        gcs().send_text(MAV_SEVERITY_CRITICAL, "c,%f,d,%f",_desired_yaw_cd,_desired_speed);

    // run steering and throttle controllers 运行转向和油门控制器

    //转向滤波
   // deep_water_3=_desired_yaw_cd;
    yaw_oldl=_desired_yaw_cd;
                    if(yaw_i==colation_sum)//已获取N个数据，实现滤波
                    {
                        int i=0;
                        for(i=0;i<colation_sum-1;i++)//循环更新最新的数据
                        {
                            yaw_buf[i] = yaw_buf[i+1];
                        }
                        yaw_buf[i]=_desired_yaw_cd;//更新最新的距离数据


                        //求平均值滤波
                        float sum=0,max=0,min=0;
                        for(i=0;i<colation_sum;i++)
                            {
                                sum +=yaw_buf[i];
                                if(max<yaw_buf[i])max=yaw_buf[i];
                                if(min>yaw_buf[i])min=yaw_buf[i];
                            }
                        //gcs().send_text(MAV_SEVERITY_CRITICAL, "buf_deeps[%d]=%f",i,buf_deeps[i]);;
                        _desired_yaw_cd=(sum-max-min)/(colation_sum-2); //滤波后的距离数据
                    }
                    if(yaw_i<colation_sum) //初始先获取N个数据
                        yaw_buf[yaw_i++]=_desired_yaw_cd;

                  //  deep_water_4=_desired_yaw_cd;
                    yaw_nowl=_desired_yaw_cd;
    ///////////////////////////////////
                   // deep_water_5=turn_rate;

          turn_rate1 =   turn_rate;
          if(hal.util->hzz_test[0]!=0)
              rover.Log_Write_loit();

    calc_steering_to_heading(_desired_yaw_cd, turn_rate); //转向，转向设定角度 _desired_yaw_cd是目标航向 turn_rate是转弯加速度，0为无限制
    calc_throttle(_desired_speed, true); //油门
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
