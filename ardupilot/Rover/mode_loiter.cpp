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

void ModeLoiter::update()
{
    // get distance (in meters) to destination 获取到目的地的距离（单位：米）
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    const float loiter_radius = rover.g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;

    // if within loiter radius slew desired speed towards zero and use existing desired heading 如果在巡航半径内，将所需速度转向零，并使用现有的所需航向
    if (_distance_to_destination <= loiter_radius) {
        // sailboats should not stop unless motoring 帆船除非开着车，否则不应该停下来
        const float desired_speed_within_radius = rover.g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);

        // if we have a sail but not trying to use it then point into the wind 如果我们有帆，但不想用它，那就指向风
        if (!rover.g2.sailboat.tack_enabled() && rover.g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }
    } else {
        // P controller with hard-coded gain to convert distance to desired speed P控制器，具有硬编码增益，可将距离转换为所需速度
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // calculate bearing to destination 计算到达目的地的方位
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it 如果目的地在车辆后面，则向其倒车
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        //如果偏航误差较大，则降低期望速度
        //45度的误差使速度降低到75%，90度的误差将速度降低到50%
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    }

    // 0 turn rate is no limit 0转弯率没有限制
    float turn_rate = 0.0;

    // make sure sailboats don't try and sail directly into the wind 确保帆船不要试图直接迎风航行
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        if (g2.sailboat.tacking()) {
            // use pivot turn rate for tacks
            turn_rate = g2.wp_nav.get_pivot_rate();
        }
    }

    // run steering and throttle controllers 运行转向和油门控制器
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
