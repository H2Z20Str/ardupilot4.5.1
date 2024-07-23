/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_WPNav_OA.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

extern const AP_HAL::HAL& hal;
extern int wp_sum;
int wp_sum_old=0;
//extern char south_wp_radius;
int32_t time_ms_old=0;
Location hzz_old,hzz_old_next,hzz_origin,hzz_origin_old;
char hezz=0,hezz_flag=0;
bool _oa_active_hzz;
// update navigation
void AR_WPNav_OA::update(float dt)
{
    // exit immediately if no current location, origin or destination
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !is_destination_valid() || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _oa_active = false;
        return;
    }

    // run path planning around obstacles
    bool stop_vehicle = false;

    // backup _origin, _destination and _next_destination when not doing oa 不执行oa时的backup_origin、_destination和_next_destination
//    if(hal.util->hzz_test[7]!=1)
    {
        if (!_oa_active) {
            _origin_oabak = _origin;
            _destination_oabak = _destination;
            _next_destination_oabak = _next_destination;

                //获取旧的位置信息
                hzz_old= _destination;
               // hzz_old_next= _next_destination;//没获取到
                hzz_origin= _origin;
        }
    }

//    if(_destination.lat!=hzz_old.lat){
//     gcs().send_text(MAV_SEVERITY_CRITICAL, "sum=%d,_origin.lat=%d,_origin.lng=%d",wp_sum,_origin.lat,_origin.lng);
//     gcs().send_text(MAV_SEVERITY_CRITICAL, "destination.lat=%d,destination.lng=%d",_destination.lat,_destination.lng);
//     gcs().send_text(MAV_SEVERITY_CRITICAL, "next_destination.lat=%d,next_destination.lng=%d",_next_destination.lat,_next_destination.lng);
//    }
//    hzz_old=_destination;

    AP_OAPathPlanner *oa = AP_OAPathPlanner::get_singleton();
    if (oa != nullptr) {
        Location oa_origin_new, oa_destination_new, oa_next_destination_new;
        AP_OAPathPlanner::OAPathPlannerUsed path_planner_used;
        bool dest_to_next_dest_clear;
      /*  const*/ AP_OAPathPlanner::OA_RetState oa_retstate = oa->mission_avoidance(current_loc,
                                                                                _origin_oabak,
                                                                                _destination_oabak,
                                                                                _next_destination_oabak,
                                                                                oa_origin_new,
                                                                                oa_destination_new,
                                                                                oa_next_destination_new,
                                                                                dest_to_next_dest_clear,
                                                                                path_planner_used);

        switch (oa_retstate) {

        case AP_OAPathPlanner::OA_NOT_REQUIRED:  //不需要物体回避
            if (_oa_active) { //上一次为true，本次结束避障
                gcs().send_text(MAV_SEVERITY_CRITICAL, "绕行成功！");

                 _origin = hzz_origin;//恢复原来的原点
                 _destination = hzz_old; //恢复原来的目标点

                // object avoidance has become inactive so reset target to original destination 目标回避已变为非活动状态，因此将目标重置为原始目的地
                if (!AR_WPNav::set_desired_location(hzz_old)) {//_destination_oabak
                    // this should never happen because we should have an EKF origin and the destination must be valid 这种情况永远不应该发生，因为我们应该有一个EKF来源，并且目的地必须是有效的
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    stop_vehicle = true;
                }
                _oa_active = false;
                // ToDo: handle "if (oa->get_options() & AP_OAPathPlanner::OA_OPTION_WP_RESET)"

            }
            break;

        case AP_OAPathPlanner::OA_PROCESSING: //仍在计算备选路径
        case AP_OAPathPlanner::OA_ERROR:        //计算过程中的错误
            // during processing or in case of error, slow vehicle to a stop
            stop_vehicle = true;
            _oa_active = false;
            break;

        case AP_OAPathPlanner::OA_SUCCESS:  //需要物体回避
            // handling of returned destination depends upon path planner used
            switch (path_planner_used) {

            case AP_OAPathPlanner::OAPathPlannerUsed::None:
            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerVertical:
                // this should never happen.  this means the path planner has returned success but has returned an invalid planner
               //这不应该发生。这意味着路径规划器返回了成功，但返回了无效的规划器
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                _oa_active = false;
                stop_vehicle = true;
                return;

            case AP_OAPathPlanner::OAPathPlannerUsed::Dijkstras:
                // Dijkstra's.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
               //Dijkstra。只有当路径规划器刚刚激活或目标目的地的纬度或经度发生变化时，才需要采取行动
                if (!_oa_active || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    if (AR_WPNav::set_desired_location(oa_destination_new)) {
                        // if new target set successfully, update oa state and destination 如果新目标设置成功，请更新oa状态和目标
                        _oa_active = true;
                        _oa_origin = oa_origin_new;
                        _oa_destination = oa_destination_new;
                        hal.util->tip=1;
                        gcs().send_text(MAV_SEVERITY_CRITICAL, "避障绕行！");
                    } else {
                        // this should never happen 这不应该发生
                        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        stop_vehicle = true;
                    }
                }
                break;

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerHorizontal: {
                // BendyRuler.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
                //BendyRuler。只有当路径规划器刚刚激活或目标目的地的纬度或经度发生变化时，才需要执行操作
                if (!_oa_active || !oa_destination_new.same_latlon_as(_oa_destination)) {

                     if (AR_WPNav::set_desired_location_expect_fast_update(oa_destination_new))
                        {
                        // if new target set successfully, update oa state and destination 如果新目标设置成功，请更新oa状态和目标
                            _oa_active = true;
                            _oa_origin = oa_origin_new;
                            _oa_destination = oa_destination_new;
                            hal.util->tip=1;
                            gcs().send_text(MAV_SEVERITY_CRITICAL, "避障绕行！");
                        } else {
                        // this should never happen
                            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                            stop_vehicle = true;
                        }

                }
            }
            break;

            } // switch (path_planner_used) {
        } // switch (oa_retstate) {



    } // if (oa != nullptr) {

    if(_oa_active==true) hal.util->bizhang_sum++;

    update_oa_distance_and_bearing_to_destination();


    if(wp_sum_old!=wp_sum)//重置
    {
        hal.util->bizhang_sum=0;
        hal.util->deep_sum =0;
        wp_sum_old=wp_sum;
        _reached_destination = false; //重置
    }

        //浅水避障
           if(hal.util->hzz_test[2]==1&&hal.util->deep_sum!=0)
               gcs().send_text(MAV_SEVERITY_CRITICAL, "deep_s=%d",hal.util->deep_sum);
           if(hal.util->deep_sum>=hal.util->OA_deep_sum)//水深
           {
               hal.util->deep_sum=0;
               hal.util->deep_fllag=1;
               hal.util->deep_sleep_flag=0;
               _reached_destination = true;
               hal.util->tip=3;
               gcs().send_text(MAV_SEVERITY_CRITICAL, "浅水避障");

           }
            _oa_active_hzz=_oa_active;

            //前视避障
           if(hal.util->hzz_test[1]==1&&hal.util->bizhang_sum!=0){
                  gcs().send_text(MAV_SEVERITY_CRITICAL, "avoid_s=%d",hal.util->bizhang_sum);
               }


           const int32_t ms_now = AP_HAL::millis();
            if(hal.util->bizhang_sum==hal.util->bizhang_sum_old)//如果计数相等，判断计时
            {
                if(ms_now-time_ms_old>=hal.util->OA_ms)//计数超过OA_ms时间，则清零，重新计数
                {
                    hal.util->bizhang_sum=0;
                }
            }
            else //计数不相等，重新
            {
                time_ms_old=ms_now;//更新计时
                hal.util->bizhang_sum_old=hal.util->bizhang_sum;//更新计数
            }

            if(hal.util->bizhang_sum>(hal.util->OA_sum ))//避障次数大于设定值，则跳点
            {
                _reached_destination = true;

                    _origin = hzz_origin;//恢复原来的原点
                    _destination = hzz_old; //恢复原来的目标点
                    _nav_control_type = NavControllerType::NAV_SCURVE;
                    _oa_active=false;

                hal.util->bizhang_sum=0;
                hal.util->tip=2;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Jumping avoidance");
                gcs().send_text(MAV_SEVERITY_CRITICAL, "无法绕开障碍物，驶入下一条航线");

            }


    if (stop_vehicle) {
        // decelerate to speed to zero and set turn rate to zero 减速至零并将转弯率设置为零
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        return;
    }

    // call parent update
    AR_WPNav::update(dt);
}

// set desired location and (optionally) next_destination
// next_destination should be provided if known to allow smooth cornering
//设置所需位置和（可选）下一个目的地
//如果已知，应提供下一个目的地，以实现平稳转弯
bool AR_WPNav_OA::set_desired_location(const Location& destination, Location next_destination)
{
    const bool ret = AR_WPNav::set_desired_location(destination, next_destination);

    if (ret) {
        // disable object avoidance, it will be re-enabled (if necessary) on next update
        _oa_active = false;
    }

    return ret;
}

// true if vehicle has reached desired location. defaults to true because this is normally used by missions and we do not want the mission to become stuck
//如果车辆已到达所需位置，则为true。默认为true，因为这通常由任务使用，我们不希望任务陷入困境
bool AR_WPNav_OA::reached_destination() const
{
    // object avoidance should always be deactivated before reaching final destination 在到达最终目的地之前，应始终禁用对象回避
//    if (_oa_active) { //取消
//        return false;
//    }

    return AR_WPNav::reached_destination();
}

// get object avoidance adjusted origin. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
////获取调整后的对象回避原点。注意：这并不保证有效（即未选中_orig_and_dest_valid）
const Location &AR_WPNav_OA::get_oa_origin() const
{
    if (_oa_active) {
        return _oa_origin;
    }

    return _origin;
}

//获得调整了目标回避的目的地。注意：这并不保证有效（即未选中_orig_and_dest_valid）
// get object avoidance adjusted destination. Note: this is not guaranteed to be valid (i.e. _orig_and_dest_valid is not checked)
const Location &AR_WPNav_OA::get_oa_destination() const
{
    if (_oa_active) {
        return _oa_destination;
    }

    return AR_WPNav::get_oa_destination();
}

// return the heading (in centi-degrees) to the next waypoint accounting for OA, (used by sailboats)
//将航向（以厘米为单位）返回到下一个计入OA的航路点（帆船使用）
float AR_WPNav_OA::oa_wp_bearing_cd() const
{
    if (_oa_active) {
        return _oa_wp_bearing_cd;
    }

    return AR_WPNav::oa_wp_bearing_cd();
}

// update distance from vehicle's current position to destination 更新车辆当前位置到目的地的距离
void AR_WPNav_OA::update_oa_distance_and_bearing_to_destination()
{
    // update OA adjusted values
    Location current_loc;
    if (_oa_active && AP::ahrs().get_location(current_loc)) {
        _oa_distance_to_destination = current_loc.get_distance(_oa_destination);
        _oa_wp_bearing_cd = current_loc.get_bearing_to(_oa_destination);
    } else {
        _oa_distance_to_destination = AR_WPNav::get_distance_to_destination();
        _oa_wp_bearing_cd = AR_WPNav::wp_bearing_cd();
    }
}
