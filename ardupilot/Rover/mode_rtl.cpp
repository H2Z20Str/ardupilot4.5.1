#include "Rover.h"

bool ModeRTL::_enter()
{
    // refuse RTL if home has not been set
    if (!AP::ahrs().home_is_set()) {
        return false;
    }

    // initialise waypoint navigation library
    g2.wp_nav.init(MAX(0, g2.rtl_speed));

    // set target to the closest rally point or home
#if HAL_RALLY_ENABLED
    if (!g2.wp_nav.set_desired_location(g2.rally.calc_best_rally_or_home_location(rover.current_loc, ahrs.get_home().alt))) {
        return false;
    }
#else
    // set destination
    if (!g2.wp_nav.set_desired_location(ahrs.get_home())) {
        return false;
    }
#endif

    send_notification = true;
    _loitering = false;
    return true;
}

extern char batrtlflag;
void ModeRTL::update()
{
    // determine if we should keep navigating 确定我们是否应该继续航行
    if (!g2.wp_nav.reached_destination()) {
        // update navigation controller 更新导航控制器
        navigate_to_waypoint();
    } else {
        // send notification发送通知
        if (send_notification) {
            send_notification = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Reached destination");
        }

        // we have reached the destination 到达目的地了
        // boats loiter, rovers stop 船在游荡，漫游车停了下来
        if (!rover.is_boat()) {
            stop_vehicle();
        } else {
            // if not loitering yet, start loitering 如果还没有闲逛，就开始闲逛吧
            if (!_loitering) {
                _loitering = rover.mode_loiter.enter();
            }
            // update stop or loiter 更新停止或游荡
            if (_loitering) {
                rover.mode_loiter.update();
            } else {
                stop_vehicle();
            }
        }
        if(batrtlflag==1)
            hal.util->tip=4;
        // update distance to destination 更新到目的地的距离
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
    }
}

// get desired location
bool ModeRTL::get_desired_location(Location& destination) const
{
    if (g2.wp_nav.is_destination_valid()) {
        destination = g2.wp_nav.get_oa_destination();
        return true;
    }
    return false;
}

bool ModeRTL::reached_destination() const
{
    return g2.wp_nav.reached_destination();
}

// set desired speed in m/s
bool ModeRTL::set_desired_speed(float speed)
{
    return g2.wp_nav.set_speed_max(speed);
}
