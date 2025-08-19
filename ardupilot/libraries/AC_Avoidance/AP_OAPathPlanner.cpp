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

#include "AP_OAPathPlanner.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_OABendyRuler.h"
#include "AP_OADijkstra.h"
#include <GCS_MAVLink/GCS.h>
extern const AP_HAL::HAL &hal;

// parameter defaults
const float OA_MARGIN_MAX_DEFAULT = 5;
const int16_t OA_OPTIONS_DEFAULT = 1;

const int16_t OA_UPDATE_MS = 1000;      // path planning updates run at 1hz
const int16_t OA_TIMEOUT_MS = 3000;     // results over 3 seconds old are ignored

const AP_Param::GroupInfo AP_OAPathPlanner::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Object Avoidance Path Planning algorithm to use
    // @Description: Enabled/disable path planning around obstacles
    // @Values: 0:Disabled,1:BendyRuler,2:Dijkstra,3:Dijkstra with BendyRuler
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1,  AP_OAPathPlanner, _type, OA_PATHPLAN_DISABLED, AP_PARAM_FLAG_ENABLE),

    // Note: Do not use Index "2" for any new parameter
    //       It was being used by _LOOKAHEAD which was later moved to AP_OABendyRuler 

    // @Param: MARGIN_MAX
    // @DisplayName: Object Avoidance wide margin distance
    // @Description: Object Avoidance will ignore objects more than this many meters from vehicle
    // @Units: m
    // @Range: 0.1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MARGIN_MAX", 3, AP_OAPathPlanner, _margin_max, OA_MARGIN_MAX_DEFAULT),

    // @Group: DB_
    // @Path: AP_OADatabase.cpp
    AP_SUBGROUPINFO(_oadatabase, "DB_", 4, AP_OAPathPlanner, AP_OADatabase),

    // @Param: OPTIONS
    // @DisplayName: Options while recovering from Object Avoidance
    // @Description: Bitmask which will govern vehicles behaviour while recovering from Obstacle Avoidance (i.e Avoidance is turned off after the path ahead is clear).   
    // @Bitmask{Rover}: 0: Reset the origin of the waypoint to the present location, 1: log Dijkstra points
    // @Bitmask{Copter}: 1:log Dijkstra points, 2:Allow fast waypoints (Dijkastras only)
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 5, AP_OAPathPlanner, _options, OA_OPTIONS_DEFAULT),

    // @Group: BR_
    // @Path: AP_OABendyRuler.cpp
    AP_SUBGROUPPTR(_oabendyruler, "BR_", 6, AP_OAPathPlanner, AP_OABendyRuler),

    AP_GROUPEND
};

/// Constructor
AP_OAPathPlanner::AP_OAPathPlanner()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// perform any required initialisation
void AP_OAPathPlanner::init()
{
    // run background task looking for best alternative destination
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        return;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
            AP_Param::load_object_from_eeprom(_oabendyruler, AP_OABendyRuler::var_info);
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
#if AP_FENCE_ENABLED
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra(_options);
        }
#endif
        break;
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
#if AP_FENCE_ENABLED
        if (_oadijkstra == nullptr) {
            _oadijkstra = new AP_OADijkstra(_options);
        }
#endif
        if (_oabendyruler == nullptr) {
            _oabendyruler = new AP_OABendyRuler();
            AP_Param::load_object_from_eeprom(_oabendyruler, AP_OABendyRuler::var_info);
        }
        break;
    }

    _oadatabase.init();
    start_thread();
}

// pre-arm checks that algorithms have been initialised successfully
bool AP_OAPathPlanner::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // check if initialisation has succeeded
    switch (_type) {
    case OA_PATHPLAN_DISABLED:
        // do nothing
        break;
    case OA_PATHPLAN_BENDYRULER:
        if (_oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "BendyRuler OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DIJKSTRA:
        if (_oadijkstra == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Dijkstra OA requires reboot");
            return false;
        }
        break;
    case OA_PATHPLAN_DJIKSTRA_BENDYRULER:
        if(_oadijkstra == nullptr || _oabendyruler == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "OA requires reboot");
            return false;
        }
        break;
    }
    return true;
}

bool AP_OAPathPlanner::start_thread()
{
    WITH_SEMAPHORE(_rsem);

    if (_thread_created) {
        return true;
    }
    if (_type == OA_PATHPLAN_DISABLED) {
        return false;
    }

    // create the avoidance thread as low priority. It should soak
    // up spare CPU cycles to fill in the avoidance_result structure based
    // on requests in avoidance_request
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_OAPathPlanner::avoidance_thread, void),
                                      "avoidance",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        return false;
    }
    _thread_created = true;
    return true;
}

// helper function to map OABendyType to OAPathPlannerUsed
AP_OAPathPlanner::OAPathPlannerUsed AP_OAPathPlanner::map_bendytype_to_pathplannerused(AP_OABendyRuler::OABendyType bendy_type)
{
    switch (bendy_type) {
    case AP_OABendyRuler::OABendyType::OA_BENDY_HORIZONTAL:
        return OAPathPlannerUsed::BendyRulerHorizontal;

    case AP_OABendyRuler::OABendyType::OA_BENDY_VERTICAL:
        return OAPathPlannerUsed::BendyRulerVertical;

    default:
    case AP_OABendyRuler::OABendyType::OA_BENDY_DISABLED:
        return OAPathPlannerUsed::None;
    }
}

// provides an alternative target location if path planning around obstacles is required
// returns true and updates result_origin, result_destination, result_next_destination with an intermediate path
// result_dest_to_next_dest_clear is set to true if the path from result_destination to result_next_destination is clear (only supported by Dijkstras)
// path_planner_used updated with which path planner produced the result
//如果需要围绕障碍物进行路径规划，则提供替代目标位置
//返回true，并使用中间路径更新result_origin、result_destination和result_next_destination
//如果从result_destination到result_next_destination的路径是明确的（仅Dijkstras支持），则result_dest_to_next_dest_clear设置为true
//path_planner_used已更新，其中路径规划器生成了结果
AP_OAPathPlanner::OA_RetState AP_OAPathPlanner::mission_avoidance(const Location &current_loc,
                                         const Location &origin,
                                         const Location &destination,
                                         const Location &next_destination,
                                         Location &result_origin,
                                         Location &result_destination,
                                         Location &result_next_destination,
                                         bool &result_dest_to_next_dest_clear,
                                         OAPathPlannerUsed &path_planner_used)
{
    // exit immediately if disabled or thread is not running from a failed init 如果已禁用或线程没有从失败的init运行，则立即退出
    if (_type == OA_PATHPLAN_DISABLED || !_thread_created) {
        return OA_NOT_REQUIRED;
    }

    // check if just activated to avoid initial timeout error 检查是否刚刚激活以避免初始超时错误
    const uint32_t now = AP_HAL::millis();
    if (now - _last_update_ms > 200) {
        _activated_ms = now;
    }
    _last_update_ms = now;

    WITH_SEMAPHORE(_rsem);

    // place new request for the thread to work on 为要处理的线程放置新的请求,来自导航代码的回避请求
    avoidance_request.current_loc = current_loc; //当前位置
    avoidance_request.origin = origin;           //当前原点
    avoidance_request.destination = destination; //当前目标点
    avoidance_request.next_destination = next_destination; //下一个目标点
    avoidance_request.ground_speed_vec = AP::ahrs().groundspeed_vector(); //地面速度矢量
    avoidance_request.request_time_ms = now;//时间

    // check result's destination and next_destination matches our request
    // e.g. check this result was using our current inputs and not from an old request
    //检查结果的destination和nextdestination是否符合我们的请求
    //例如，检查这个结果是使用我们当前的输入，而不是来自旧的请求
    const bool destination_matches = destination.same_latlon_as(avoidance_result.destination); //目的地匹配
    const bool next_destination_matches = next_destination.same_latlon_as(avoidance_result.next_destination);//下一个目的地匹配

    // check results have not timed out 检查结果尚未超时
    const bool timed_out = (now - avoidance_result.result_time_ms > OA_TIMEOUT_MS) && (now - _activated_ms > OA_TIMEOUT_MS);

    // return results from background thread's latest checks 返回后台线程的最新检查结果
    if (destination_matches && next_destination_matches && !timed_out) {
        // we have a result from the thread 我们有一个线程的结果，避障处理结果
        result_origin = avoidance_result.origin_new; //避障原点
        result_destination = avoidance_result.destination_new; //避障目标点
        result_next_destination = avoidance_result.next_destination_new; //下一个避障目标点
        result_dest_to_next_dest_clear = avoidance_result.dest_to_next_dest_clear; //结果dest到下一个dest清除
        path_planner_used = avoidance_result.path_planner_used; //使用路径规划器
        return avoidance_result.ret_state;//避障情况
    }

    // if timeout then path planner is taking too long to respond 如果超时，则路径规划器的响应时间过长
    if (timed_out) {
        return OA_ERROR;
    }

    // background thread is working on a new destination 后台线程正在处理一个新的目标
    return OA_PROCESSING;
}

// avoidance thread that continually updates the avoidance_result structure based on avoidance_request
//避免线程，根据avoidance_request不断更新avoidance-result结构
void AP_OAPathPlanner::avoidance_thread()
{
    // require ekf origin to have been set 要求已设置ekf原点
    bool origin_set = false;
    while (!origin_set) {
        hal.scheduler->delay(500);
        Location ekf_origin {};
        {
            WITH_SEMAPHORE(AP::ahrs().get_semaphore()); //允许线程锁定以防止AHRS更新
            origin_set = AP::ahrs().get_origin(ekf_origin);    
        }
    }

    while (true) {

        // if database queue needs attention, service it faster 如果数据库队列需要关注，请更快地为其提供服务
        if (_oadatabase.process_queue()) {
            hal.scheduler->delay(1);
        } else {
            hal.scheduler->delay(20);
        }

        const uint32_t now = AP_HAL::millis();
        if (now - avoidance_latest_ms < OA_UPDATE_MS) {//1hz，1秒内部处理
            continue;//结束本次线程
        }
        avoidance_latest_ms = now;

        _oadatabase.update();

        // values returned by path planners 路径规划器返回的值
        Location origin_new;
        Location destination_new;
        Location next_destination_new;
        bool dest_to_next_dest_clear = false;
        {
            WITH_SEMAPHORE(_rsem);
            if (now - avoidance_request.request_time_ms > OA_TIMEOUT_MS) {
                // this is a very old request, don't process it 这是一个非常旧的请求，不要处理它
                continue;
            }

            // copy request to avoid conflict with main thread 复制请求以避免与主线程冲突
            avoidance_request2 = avoidance_request;

            // store passed in origin, destination and next_destination so we can return it if object avoidance is not required
            //复制请求以避免与主线程冲突
            origin_new = avoidance_request.origin;
            destination_new = avoidance_request.destination;
            next_destination_new = avoidance_request.next_destination;
        }

        // run background task looking for best alternative destination 运行后台任务，寻找最佳替代目标
        OA_RetState res = OA_NOT_REQUIRED;
        OAPathPlannerUsed path_planner_used = OAPathPlannerUsed::None;
        switch (_type) {
        case OA_PATHPLAN_DISABLED:
            continue;
        case OA_PATHPLAN_BENDYRULER: {
            if (_oabendyruler == nullptr) {
                continue;
            }
            _oabendyruler->set_config(_margin_max);

            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, false)) {
                res = OA_SUCCESS;
            }
            path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
            break;
        }

        case OA_PATHPLAN_DIJKSTRA: {
#if AP_FENCE_ENABLED
            if (_oadijkstra == nullptr) {
                continue;
            }
            _oadijkstra->set_fence_margin(_margin_max);
            const AP_OADijkstra::AP_OADijkstra_State dijkstra_state = _oadijkstra->update(avoidance_request2.current_loc,
                                                                                          avoidance_request2.destination,
                                                                                          avoidance_request2.next_destination,
                                                                                          origin_new,
                                                                                          destination_new,
                                                                                          next_destination_new,
                                                                                          dest_to_next_dest_clear);
            switch (dijkstra_state) {
            case AP_OADijkstra::DIJKSTRA_STATE_NOT_REQUIRED:
                res = OA_NOT_REQUIRED;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_ERROR:
                res = OA_ERROR;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_SUCCESS:
                res = OA_SUCCESS;
                break;
            }
            path_planner_used = OAPathPlannerUsed::Dijkstras;
#endif
            break;
        }

        case OA_PATHPLAN_DJIKSTRA_BENDYRULER: {
            if ((_oabendyruler == nullptr) || _oadijkstra == nullptr) {
                continue;
            } 
            _oabendyruler->set_config(_margin_max); //避障距离
            AP_OABendyRuler::OABendyType bendy_type;
            if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, bendy_type, proximity_only)) {
                // detected a obstacle by vehicle's proximity sensor. Switch avoidance to BendyRuler till obstacle is out of the way
                //通过车辆的接近传感器检测到障碍物。将避让切换到BendyRuler，直到障碍物让路
                proximity_only = false;
                res = OA_SUCCESS;
                path_planner_used = map_bendytype_to_pathplannerused(bendy_type);
                break;
            } else {
                // cleared all obstacles, trigger Dijkstra's to calculate path based on current deviated position  
                //清除所有障碍物，触发Dijkstra根据当前偏差位置计算路径
#if AP_FENCE_ENABLED
                if (proximity_only == false) {
                    _oadijkstra->recalculate_path();
                }
#endif
                // only use proximity avoidance now for BendyRuler 现在仅对BendyRuler使用邻近回避
                proximity_only = true;
            }
#if AP_FENCE_ENABLED
            _oadijkstra->set_fence_margin(_margin_max);
            const AP_OADijkstra::AP_OADijkstra_State dijkstra_state = _oadijkstra->update(avoidance_request2.current_loc,
                                                                                          avoidance_request2.destination,
                                                                                          avoidance_request2.next_destination,
                                                                                          origin_new,
                                                                                          destination_new,
                                                                                          next_destination_new,
                                                                                          dest_to_next_dest_clear);
            switch (dijkstra_state) {
            case AP_OADijkstra::DIJKSTRA_STATE_NOT_REQUIRED:
                res = OA_NOT_REQUIRED;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_ERROR:
                res = OA_ERROR;
                break;
            case AP_OADijkstra::DIJKSTRA_STATE_SUCCESS:
                res = OA_SUCCESS;
                break;
            }
            path_planner_used = OAPathPlannerUsed::Dijkstras;
#endif
            break;
        } //case OA_PATHPLAN_DJIKSTRA_BENDYRULER:

        } // switch

        {
            // give the main thread the avoidance result 给出主线的回避结果
            WITH_SEMAPHORE(_rsem);

            // place the destination and next destination used into the result (used by the caller to verify the result matches their request)
            //将目的地和下一个目的地放入结果中（由调用者用于验证结果是否与他们的请求匹配）
            avoidance_result.destination = avoidance_request2.destination;
            avoidance_result.next_destination = avoidance_request2.next_destination;
            avoidance_result.dest_to_next_dest_clear = dest_to_next_dest_clear;

            // fill the result structure with the intermediate path 用中间路径填充结果结构
            avoidance_result.origin_new = (res == OA_SUCCESS) ? origin_new : avoidance_result.origin_new;
            avoidance_result.destination_new = (res == OA_SUCCESS) ? destination_new : avoidance_result.destination;
            avoidance_result.next_destination_new = (res == OA_SUCCESS) ? next_destination_new : avoidance_result.next_destination;

            // create new avoidance result.dest_to_next_dest_clear field.  fill in with results from dijkstras or leave as unknown
            //创建新的回避结果.dest_to_next_dest-clear字段。填写dijkstras的结果或留下未知
            avoidance_result.result_time_ms = AP_HAL::millis();
            avoidance_result.path_planner_used = path_planner_used;
            avoidance_result.ret_state = res;
        }
    }
}

// singleton instance
AP_OAPathPlanner *AP_OAPathPlanner::_singleton;

namespace AP {

AP_OAPathPlanner *ap_oapathplanner()
{
    return AP_OAPathPlanner::get_singleton();
}

}
