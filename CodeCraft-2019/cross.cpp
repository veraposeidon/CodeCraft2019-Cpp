//
// Created by shen on 2019/3/25.
//

#include <c++/algorithm>
#include <cassert>
#include <c++/iostream>
#include "cross.h"

Cross::Cross() {
    crossID = -1;
    roads = vector<int>(0);
    roads_prior_id = vector<int>(0);
    roads_prior_name = vector<string>(0);
    nothing2do = false;
}

Cross::Cross(int cross_id, int road1, int road2, int road3, int road4, unordered_map<string, Road> &road_dict) {
    crossID = cross_id;
    roads = vector<int>{road1, road2, road3, road4};
    roads_prior_id = get_road_priors(); // 获取除-1外的道路ID优先级

    // 根据道路ID获取Name
    roads_prior_name = vector<string>(0);
    for (int i : roads_prior_id) {
        string road_name = find_road_name_to_cross(road_dict, i);
        if (road_name != NO_FIND)   // 去除单向道路
        {
            roads_prior_name.push_back(road_name);
        }
    }
    nothing2do = false;
}

/**
 * 路口调度道路的优先级，按照id升序
 * @return
 */
vector<int> Cross::get_road_priors() {
    vector<int> road_prior = roads; // 注意深拷贝
    sort(road_prior.begin(), road_prior.end()); // 升序
    road_prior.erase(remove(road_prior.begin(), road_prior.end(), -1), road_prior.end());    // 删除 -1 元素
    return road_prior;  // 返回
}

/**
 * 据道路ID和路口名称找道路名称
 * @param road_dict 道路ID
 * @param road_id   道路字典
 * @return          道路名称
 */
string Cross::find_road_name_to_cross(unordered_map<string, Road> &road_dict, int road_id) {
    for (unordered_map<string, Road>::const_iterator node = road_dict.begin(); node != road_dict.end(); node++) {
        string road_name = (*node).first;
        if (road_dict[road_name].roadID == road_id && road_dict[road_name].roadDest == crossID)
            return road_name;
    }
    return NO_FIND;
}

/**
 * 重置路口完成标记
 */
void Cross::reset_end_flag() {
    nothing2do = false;
}

/*/
 * 查询路口是否完成
 */
bool Cross::if_cross_ended() {
    return nothing2do;
}

/**
 * 判断出路口转向
 * @param road_id
 * @param next_road_id
 * @return
 */
string Cross::get_direction(int road_id, int next_road_id) {
    int index_now = 0, index_next = 0;
    for (int i = 0; i < roads.size(); ++i) {
        if (roads[i] == road_id) {
            index_now = i;
        }
        if (roads[i] == next_road_id) {
            index_next = i;
        }
    }

    assert(index_next != index_now);
    int error = index_next - index_now;
    if (error == 2 || error == -2) {
        return "D";
    } else if (error == 3 || error == -1) {
        return "R";
    } else if (error == 1 || error == -3) {
        return "L";
    }

    return NO_FIND;
}

/**
 * 判断有无直行进入目标车道的车辆发生冲突
 * @param roads_map
 * @param target_road__id
 * @return
 */
bool Cross::has_straight_to_conflict(unordered_map<int, order_info> &roads_map, int target_road__id) {
    for (unordered_map<int, order_info>::const_iterator node = roads_map.begin(); node != roads_map.end(); node++) {
        order_info road_ord_info = (*node).second;
        if (road_ord_info.next_road_id == target_road__id && road_ord_info.direction == "D") {
            return true;
        }
    }
    return false;
}

/**
 * 判断有无直行或左转进入目标车道的车辆发生冲突
 * @param roads_map
 * @param target_road__id
 * @return
 */
bool Cross::has_straight_left_to_conflict(unordered_map<int, order_info> &roads_map, int target_road__id) {
    for (unordered_map<int, order_info>::const_iterator node = roads_map.begin(); node != roads_map.end(); node++) {
        order_info road_ord_info = (*node).second;
        if (road_ord_info.next_road_id == target_road__id &&
            (road_ord_info.direction == "D" || road_ord_info.direction == "L")) {
            return true;
        }
    }
    return false;
}

/**
 * 获取道路的出路口第一优先级车辆
 * @param road_name
 * @param road_dict
 * @param car_dict
 * @param road_id
 * @param first_order
 * @return
 */
bool Cross::get_road_first_order_info(string road_name, unordered_map<string, Road> &road_dict,
                                      unordered_map<int, Car> &car_dict, int &road_id, order_info &first_order) {
    while (true) {
        int car_id = road_dict[road_name].get_first_order_car(car_dict);
        // 当前道路没有待出路口车辆
        if (car_id == -1) {
            return false;
        }
            // 当前道路有待出路口车辆
        else {
            Car &car_obj = car_dict[car_id];
            if (NO_ANSWER == car_obj.next_road_name(crossID))    // 是否下一站到家
            {
                // TODO： 此处优化逻辑
                // 车辆回家
                road_dict[road_name].move_car_home(car_obj);
                // 更新车道后方信息
                road_dict[road_name].update_channel(car_obj.carGPS["channel"], car_dict);
                // 除去道路第一优先序车辆记录
                road_dict[road_name].first_order_car_id = -1;
                // 继续更新本条道路的第一优先级
                continue;
            } else {
                // 不回家车辆的下一条路名称
                string next_road_name = car_obj.next_road_name(crossID);

                // 获取道路ID
                int road_now_id = road_dict[road_name].roadID;
                int road_next_id = road_dict[next_road_name].roadID;
                assert(road_now_id != road_next_id);   // 聊胜于无  # 出现相同是因为计划路线出现了掉头，这个是不允许的。要在车辆更新路线时进行否定

                // 获取方向,填入信息
                string direction = get_direction(road_now_id, road_next_id);
                order_info first_order_info(car_obj.carID, road_name, road_next_id, next_road_name, direction);
                road_id = road_now_id;
                first_order = first_order_info;
                return true;
            }
        }
    }
}

/**
 * 获取第一优先级车辆信息
 * @param road_dict
 * @param car_dict
 * @return
 */
unordered_map<int, order_info>
Cross::get_first_order_info(unordered_map<string, Road> &road_dict, unordered_map<int, Car> &car_dict) {
    vector<string> road_prior = roads_prior_name;   // deep copy
    unordered_map<int, order_info> next_roads;

    for (const auto &road_name : road_prior) {
        int road_id = -1;
        order_info first_info;
        bool got = get_road_first_order_info(road_name, road_dict, car_dict, road_id, first_info);
        if (got) {
            next_roads[road_id] = first_info;
        }
    }
    return next_roads;
}

/**
 * 跨路口移动车辆，较为复杂, 需要耐心
 * 可论证： 这些待转车辆前方都没有阻挡的车的。
 * 假设前方的车 已经 end 了，后方的车会更新到end
 * 假设前方的车 在waiting， 后方的车轮不到第一优先级。
 * @param car_obj
 * @param this_road
 * @param next_road
 * @param car_dict
 */
void Cross::move_car_across(Car &car_obj, Road &this_road, Road &next_road, unordered_map<int, Car> &car_dict) {
    // 1. 找到待进入车道和位置
    int next_channel, e_pos;
    bool succeed = next_road.get_checkin_place_cross(next_channel, e_pos, car_dict);
    // 前方道路堵住
    // 前方道路堵住需要探讨（前方道路的车是终结态还是等待态，只要最后有车等待，那就可以等待，如果最后一排的车全为终结，那就终结）
    if (!succeed) // 表示下一条路全满
    {
        // 如果下一条道路最后排有车在等待,则本车也只能等待
        if (next_road.last_row_are_waiting(car_dict)) {
            car_obj.change2waiting_out();  // 后面的车不需要更新
        } else {
            int car_pos = car_obj.carGPS["pos"];
            int new_pos = this_road.roadLength - 1;  // 注意下标
            int car_channel = car_obj.carGPS["channel"];

            // 车已在道路前方，保持不动
            if (car_pos == new_pos) {
                car_obj.change2end();
            }
                // 未在道路前方，移动车辆
            else {
                this_road.move_car_to(car_channel, car_pos, new_pos, car_obj);
            }

            // 重置第一优先级车辆
            this_road.first_order_car_id = -1;
            // 更新后面车道的车辆
            this_road.update_channel(car_channel, car_dict);
        }
        return;
    }

    // 前方道路没堵住
    int car_pos = car_obj.carGPS["pos"];
    int car_channel = car_obj.carGPS["channel"];
    cout << (this_road.roadID) << endl;
    assert(this_road.roadStatus[car_channel][car_pos] == car_obj.carID);    // 聊胜于无的断言

    int remain_dis = (this_road.roadLength - 1) - car_pos;  // 上端剩余距离
    int speed = min(car_obj.carSpeed, next_road.roadSpeedLimit);
    int real_dis = max(speed - remain_dis, 0);

    if (real_dis == 0)   // 表示不支持转入下一道路，现在调度到本车道终点处，不变channel
    {
        int new_pos = this_road.roadLength - 1; // 注意下标
        // 车已在道路前方，保持不动
        if (car_pos == new_pos) {
            car_obj.change2end();
        }
            // 未在道路前方，移动车辆
        else {
            this_road.move_car_to(car_channel, car_pos, new_pos, car_obj);
        }

        // 重置第一优先级车辆
        this_road.first_order_car_id = -1;
        // 更新后面车道的车辆
        this_road.update_channel(car_channel, car_dict);
        return;
    }
        // 有机会调度到下一道路# 三种情况，够长，直接到位；前方有车，endstate，追尾；前方有车，waiting,不动waiting。
    else {
        int new_pos = real_dis - 1; // 注意下标
        // 判断前方有无车辆
        int front_pos, front_id;
        bool has_c = next_road.has_car(next_channel, 0, new_pos + 1, front_pos, front_id);
        // 前方有车
        if (has_c) {
            if (car_dict[front_id].is_car_waiting()) //前车正在等待
            {
                car_obj.change2waiting_out();   //  标记为等待出路口
            }
                // 前车结束
            else {
                int dis = front_pos;
                assert (dis >= 1);  // 要是距离短于1就见鬼了
                new_pos = front_pos - 1;  // 还能前进一段，新位置在前车屁股
                //  新道路上移动车辆
                next_road.move_car_to(next_channel, -1, new_pos, car_obj);
                // 旧道路上抹除位置
                this_road.roadStatus[car_channel][car_pos] = -1; // 将车辆原来位置置空

                // 重置第一优先级车辆
                this_road.first_order_car_id = -1;
                // 更新后方车道
                this_road.update_channel(car_channel, car_dict);
            }
        } else {
            // 新道路上移动车辆
            next_road.move_car_to(next_channel, -1, new_pos, car_obj);
            // 旧道路上抹除位置
            this_road.roadStatus[car_channel][car_pos] = -1;    // 将车辆原来位置置空
            //  重置第一优先级车辆
            this_road.first_order_car_id = -1;
            //  更新后方车道
            this_road.update_channel(car_channel, car_dict);
        }
        return;
    }
}

/**
 * 调度路口多次
 * @param road_dict
 * @param car_dict
 * @param loops_every_cross
 */
void
Cross::update_cross(unordered_map<string, Road> &road_dict, unordered_map<int, Car> &car_dict, int loops_every_cross) {
    for (int i = 0; i < loops_every_cross; i++) {
        // 获取待调度道路和车辆信息
        unordered_map<int, order_info> next_roads = get_first_order_info(road_dict, car_dict);
        // 如果没有待调度车辆，则判断该路口完成
        if (next_roads.empty()) {
            nothing2do = true;
            return;
        }

        // 路口调度信息
        vector<int> roadIDs(0);
        for (auto &road : next_roads) {
            int road_id = road.first;
            roadIDs.push_back(road_id);
        }
        sort(roadIDs.begin(), roadIDs.end());   // 排序

        // 根据优先级，分别判断每个路口是否满足出路口（路口规则）
        for (int roadID : roadIDs) {
            int last_car_id= next_roads[roadID].car_id;   // 待调度车辆
            while (next_roads.find(roadID) != next_roads.end()) //确保路口的每一轮调度都最大化道路的运输能力，除非转弯顺序不允许或者没有待转弯车辆了。
            {
                // 对方向进行判断
                string direct = next_roads[roadID].direction;
                if (direct == "D") {
                    // 直行优先
                    ;
                } else if (direct == "L") {
                    //左转需要判断有无直行到目标道路车辆
                    // 有直行车辆冲突，跳过
                    if (has_straight_to_conflict(next_roads, next_roads[roadID].next_road_id)) {
                        break; //跳出while 调度下一道路)
                    }
                } else if (direct == "R") {
                    // 右转需要哦安短有无直行或左转到目标道路车辆
                    // 有直行或左转车辆冲突，跳过
                    if (has_straight_left_to_conflict(next_roads, next_roads[roadID].next_road_id)) {
                        break;  // 跳出while 调度下一道路
                    }
                }

                // 调度车辆
                Car &car_o = car_dict[next_roads[roadID].car_id] ;

                Road &this_road = road_dict[next_roads[roadID].road_name];
                Road &next_road = road_dict[next_roads[roadID].next_road_name];

                move_car_across(car_o, this_road, next_road, car_dict);

                // 只更新该道路的优先序车辆
                int road_id = -1;
                order_info first_info;
                bool got = get_road_first_order_info(next_roads[roadID].road_name, road_dict, car_dict, road_id,
                                                     first_info);
                // 该道路还有待调度车辆
                if (got) {
                    next_roads[road_id] = first_info;
                    // 判断更新后的第一辆车还是不是之前的
                    int this_car_id = next_roads[roadID].car_id;
                    if (this_car_id == last_car_id) {
                        break; //  还是上一辆，说明没动，跳出，调度下一道路
                    }else{
                        last_car_id = this_car_id;
                    }
                }else{
                    // 清除字典信息
                    next_roads.erase(next_roads.find(roadID));
                    break;
                }
            }
        }

    }
}

