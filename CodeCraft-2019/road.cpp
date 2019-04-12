//
// Created by shen on 2019/3/24.
//
#include "road.h"
#include <cassert>
#include <algorithm>
#include <iostream>

using namespace std;

/**
 * 默认构造函数
 */
Road::Road() {
    roadID = -1;
    roadLength = -1;
    roadSpeedLimit = -1;
    roadChannel = -1;
    roadOrigin = -1;
    roadDest = -1;
    // 第一优先级车辆ID
    first_order_car_id = -1;
    // 所有道路格子赋为 -1
    roadStatus = vector<vector<int> >(0, vector<int>(0, -1));
    prior_cars_preset = vector<int> ();      // 本条路负责上路的优先车辆(预置)
    prior_cars_unpreset = vector<int> ();    // 本条路负责上路的优先车辆(非预置)
    unpriors_cars_preset = vector<int> (); // 本条路负责上路的非优先车辆(预置)
    unpriors_cars_unpreset = vector<int> (); // 本条路负责上路的非优先车辆(非预置)
}


/**
 * 构造函数
 * @param road_id
 * @param length
 * @param speed_limit
 * @param channel
 * @param origin
 * @param dest
 */
Road::Road(const int road_id, const int length, const int speed_limit, const int channel, const int origin,
           const int dest) {
    roadID = road_id;
    roadLength = length;
    roadSpeedLimit = speed_limit;
    roadChannel = channel;
    roadOrigin = origin;
    roadDest = dest;
    // 第一优先级车辆ID
    first_order_car_id = -1;
    // 所有道路格子赋为 -1
    roadStatus = vector<vector<int> > (roadChannel, vector<int>(roadLength, -1));
    prior_cars_preset = vector<int> ();      // 本条路负责上路的优先车辆(预置)
    prior_cars_unpreset = vector<int> ();    // 本条路负责上路的优先车辆(非预置)
    unpriors_cars_preset = vector<int> (); // 本条路负责上路的非优先车辆(预置)
    unpriors_cars_unpreset = vector<int> (); // 本条路负责上路的非优先车辆(非预置)
}

/*
 * 更新道路，时间片内第一次调度
 * 在车道内行驶，由于不可以超车，所以仍是按序行驶
 */
void Road::update_road(unordered_map<int, Car> &car_dict) {
    for (int grid = roadLength - 1; grid >= 0; --grid) {
        for (int channel = 0; channel < roadChannel; channel++) {
            if (roadStatus[channel][grid] != -1) {
                int car_id = roadStatus[channel][grid];
                // 标记所有车辆为待处理状态
                car_dict[car_id].change2waiting();
                // 调度车辆
                update_car(car_dict[car_id], channel, grid, car_dict);
            }
        }
    }
}


/**
 * 调度车辆
 * @param car_obj
 * @param channel
 * @param grid
 * @param car_dict
 */
void Road::update_car(Car &car_obj, int channel, int grid, unordered_map<int, Car> &car_dict) {
    // 断言检测 车辆位置
    int car_channel = car_obj.carGPS.channel;
    int car_pos = car_obj.carGPS.pos;
    assert(car_channel == channel);
    assert(car_pos == grid);

    int speed = min(car_obj.carSpeed, roadSpeedLimit);  // 获取车速
    int len_remain = (roadLength - 1) - car_pos;  // 道路剩余长度

    // 准备出路口
    if (len_remain < speed) {
        int front_pos=-1, front_id=-1;
        bool has_c = has_car(car_channel, car_pos + 1, roadLength, front_pos, front_id);
        // 前方有车
        if (has_c) {
            // 前车正在等待
            if (car_dict[front_id].is_car_waiting()) {
                car_obj.change2waiting_out();   // 标记为等待调度出路口
            }
                // 前车调度结束
            else {
                int dis = front_pos - car_pos;
                assert(dis >= 1);  // 两车相距大于一
                if (dis == 1) {
                    car_obj.change2end();  // 本就在前车屁股，无需调度
                } else {
                    int new_pos = front_pos - 1;  // 还能前进一段，新位置在前车屁股
                    move_car_to(car_channel, car_pos, new_pos, car_obj);  // 移动车辆
                }
            }
        }
            // 前方无车
        else {
            car_obj.change2waiting_out();  // 标记为等待调度出路口
        }
    }
        // 不准备出路口
    else {
        int front_pos=-1, front_id=-1;
        bool has_c = has_car(car_channel, car_pos + 1, car_pos + speed + 1, front_pos, front_id);
        // 前方有车
        if (has_c) {
            // 前车正在等待
            if (car_dict[front_id].is_car_waiting()) {
                car_obj.change2waiting_inside();    // 标记为等待且不出路口
            }
                // 前车结束调度
            else {
                int dis = front_pos - car_pos;
                assert(dis >= 1);  // 两车相距大于一
                if (dis == 1) {
                    car_obj.change2end();  // 本就在前车屁股，无需调度
                } else {
                    int new_pos = front_pos - 1;  // 还能前进一段，新位置在前车屁股
                    move_car_to(car_channel, car_pos, new_pos, car_obj);  // 移动车辆
                }
            }
        }
        // 前方无车
        else {
            int new_pos = car_pos + speed;
            move_car_to(car_channel, car_pos, new_pos, car_obj);  // 移动车辆
        }
    }

}


/**
 * 移动车辆到指定位置
 * @param car_channel
 * @param car_pos
 * @param new_pos
 * @param car_obj
 */
void Road::move_car_to(int car_channel, int car_pos, int new_pos, Car &car_obj) {
    // 注册新位置
    roadStatus[car_channel][new_pos] = car_obj.carID;
    // 抹除旧位置
    if (car_pos != -1)   // 也存在车从家里出发的情况
    {
        roadStatus[car_channel][car_pos] = -1;
    }

    // 更新到车辆信息
    car_obj.mark_new_pos(roadID, car_channel, new_pos, roadOrigin, roadDest);

    // 标记车辆为EndState
    car_obj.change2end();
}

/**
 * 判断车道内某段区域是否有车
 * @param channel   车道
 * @param start     车前方一格
 * @param end       目标占领点前方一格
 * @param position  若存在车，前车位置
 * @param car_id    若存在车，前车ID
 * @return          是否存在车辆
 */
bool Road::has_car(int channel, int start, int end, int &position, int &car_id) {
    // 车已在目标位置
    if (start == end) {
        return false;
    }

    for (int grid = start; grid < end; ++grid) {
        if (roadStatus[channel][grid] == -1)
            continue;
        else {
            position = grid;
            car_id = roadStatus[channel][grid];
            return true;
        }
    }

    return false;
}

/**
 * 启动时 获取进入道路时的空位置
 * 要是没有返回None
 * 必须要找小编号车道优先原则
 * 判断车辆分布即可
 * @return
 */
bool Road::get_checkin_place_start(int &e_channel, int &e_pos) {
    bool all_neg = true;
    for(int channel = 0;channel<roadChannel;channel++)
    {
        for (int pos = 0; pos < roadLength; pos ++)
        {
            if (roadStatus[channel][pos] != -1)
            {
                all_neg = false;
                break;
            }
        }
    }
    if(all_neg)
    {
        // 道路为空：第一车道最前方
        e_channel = 0;
        e_pos = roadLength - 1;
        return true;
    }

    for (int channel = 0; channel < roadChannel; ++channel) {
        if (roadStatus[channel][0] != -1)   // 必须要最后一格有车）
            continue;

        for (int pos = 0; pos < roadLength; ++pos) {
            if (roadStatus[channel][pos] == -1) {
                // 到头那就是最大长度
                if (pos == roadLength - 1) {
                    e_channel = channel;
                    e_pos = pos;
                    return true;
                } else {
                    continue;
                }
            } else {
                e_channel = channel;
                e_pos = pos - 1;  // 返回空位置，而不是有阻挡的位置
                return true;
            }
        }
    }

    return false;
}

/**
 * 车辆入驻道路
 * @param car_obj
 * @return True,成功； False,失败
 */
bool Road::try_on_road(Car &car_obj, int time, bool in_cross, unordered_map<int, Car> &car_dict) {
    // 判断时间
    if (car_obj.carPlanTime > time)
        return false;
    // 1. 找车位
    int e_channel=-1, e_pos=-1;
    bool succeed;
    if (in_cross) {
        succeed = get_checkin_place_cross(e_channel, e_pos, car_dict);
    } else {
        succeed = get_checkin_place_start(e_channel, e_pos);
    }


    if (!succeed)
        return false;   // 上路失败

    // 2. 根据车速和前车位置 判断新位置
    int new_pos = std::min({car_obj.carSpeed - 1, roadSpeedLimit - 1, e_pos});

    // 3. 移动车辆
    move_car_to(e_channel, -1, new_pos, car_obj);

    // 4. 移动完标记为调度完成
    car_obj.change2end();

    // 5. 记录下发车时间
    car_obj.startTime = time;

    return true;    // 上路成功
}

/**
 * 当有车更新到终止态之后，要更新一次当前车道的车辆
 * @param channel_id
 * @param car_dict
 */
void Road::update_channel(int channel_id, unordered_map<int, Car> &car_dict) {
    for (int grid = roadLength - 1; grid >= 0; --grid) {
        if (roadStatus[channel_id][grid] != -1) {
            // 获取车对象
            int car_id = roadStatus[channel_id][grid];
            // 判断车辆状态，END跳过
            if (car_dict[car_id].is_car_waiting()) {
                // 调度车辆
                update_car(car_dict[car_id], channel_id, grid, car_dict);
            }
        }
    }
}

/**
 * 判断道路起始最优一批的几辆车是否存在等待情况
 * @param car_dict
 * @return
 */
bool Road::last_row_are_waiting(unordered_map<int, Car> &car_dict) {
    for (int channel = 0; channel < roadChannel; ++channel) {
        if (car_dict[roadStatus[channel][0]].is_car_waiting())
            return true;
    }

    return false;
}


/**
 * 移动车辆回家
 * @param car_obj
 */
void Road::move_car_home(Car &car_obj, int time) {
    assert(car_obj.is_car_way_home());  //断言
    int car_id = car_obj.carID;
    int car_channel = car_obj.carGPS.channel;
    int car_pos = car_obj.carGPS.pos;
    assert(roadStatus[car_channel][car_pos] == car_id); //直接断言 车辆所处位置正确

    roadStatus[car_channel][car_pos] = -1;  //将车辆所在位置置空
    car_obj.change2success(time);   // 更改车辆状态
}

/**
 * 车越多越堵，数值越大g
 * 简单版本： 只统计个数
 * 复杂版本： 从后到前，权重加大
 * @param dist_k
 * @return
 */
double Road::get_road_weight(double dist_k = 1.0) {
    int count_car = 0;
    int count_dist = 0;
    for (int channel = 0; channel < roadChannel; ++channel) {
        for (int pos = 0; pos < roadLength; ++pos) {
            if (roadStatus[channel][pos] != -1) {
                count_car += 1;     // 车数目
                count_dist += pos;  // 车分布  TODO: 也可以在车道分布上下文章
            }
        }
    }

    // 占车位百分比，越大越堵
    double cars_percent = count_car * 1.0 / (roadLength * roadChannel);
    double dist_percent;
    dist_percent = 2.0 * (count_car * roadLength - count_dist) / (
                (roadLength + 1) * roadLength * roadChannel);

    return cars_percent * (1 - dist_k) + dist_percent * dist_k;
}

/**
 * 获取本条道路的第一优先级车辆（只考虑出路口的车辆）
 * TODO: 复赛： 优先级车辆的变动（当前道路只要优先车辆前方无其他车辆阻挡，优先车辆即可通过）
 * @return
 */
int Road::get_first_order_car(unordered_map<int, Car> &car_dict) {
    if (first_order_car_id != -1)
        return first_order_car_id;

    // 根据优先序列遍历车辆
    // 到此处first_order_car_id = -1
    int normal_first_order_car = -1;    // 非优先车辆的第一优先级
    int prior_first_order_car = -1;     // 优先车辆的第一优先级
    for (int pos = roadLength - 1; pos >= 0; --pos) {
        for (int channel = 0; channel < roadChannel; ++channel) {
            if(prior_first_order_car !=-1){
                break;  // 已经有优先车辆就可以结束循环了
            }

            if (roadStatus[channel][pos] != -1) {
                // 获取车对象
                int car_id = roadStatus[channel][pos];
                // 是否等待出路口
                if (car_dict[car_id].is_car_waiting_out()) {
                    // 1. 出路口的先决条件仍是只为出路口车辆服务
                    if(!car_dict[car_id].carPriority)   // 非优先车辆
                    {
                        if(normal_first_order_car == -1)    // 只标记第一辆
                            normal_first_order_car = car_id;
                    }else{  // 优先车辆
                        // 判断该优先车前方有无阻挡
                        int position, carid;
                        if(has_car(channel, pos+1, roadLength, position, carid)){
                            continue;   // 表示有阻挡
                        }

                        // 没有阻挡
                        if(prior_first_order_car == -1)
                            prior_first_order_car = car_id; // 第一辆优先车辆
                    }
                }
            }
        }
    }

    // 比较有无优先级车辆了
    if(prior_first_order_car != -1){
        first_order_car_id = prior_first_order_car;
        return first_order_car_id;
    }else if(normal_first_order_car != -1){
        first_order_car_id = normal_first_order_car;
        return first_order_car_id;
    }else{
        return -1;
    }
}

/**
 * 获取进入道路时的空位置
 * 必须要找小编号车道优先原则
 * @param e_channel
 * @param e_pos
 * @return
 */
bool Road::get_checkin_place_cross(int &e_channel, int &e_pos, unordered_map<int, Car> &car_dict) {
    bool all_neg = true;
    for(int channel = 0;channel<roadChannel;channel++)
    {
        for (int pos = 0; pos < roadLength; pos ++)
        {
            if (roadStatus[channel][pos] != -1)
            {
                all_neg = false;
                break;
            }
        }
    }
    if(all_neg)
    {
        // 道路为空：第一车道最前方
        e_channel = 0;
        e_pos = roadLength - 1;
        return true;
    }

    for (int channel = 0; channel < roadChannel; ++channel) {
        // 最后一格有车，判断其状态
        if (roadStatus[channel][0] != -1) {
            // 最后一个调度结束就找下条道
            if (car_dict[roadStatus[channel][0]].is_car_end_state())
                continue;
            else
                return false;   // 否则按堵住处理，需要前车移动了才能调度后车，让后车先等待
        }

        for (int pos = 0; pos < roadLength; ++pos) {
            if (roadStatus[channel][pos] == -1) {
                if (pos == roadLength - 1) {
                    e_channel = channel;
                    e_pos = pos;
                    return true;
                } else {
                    continue;
                }
            } else {
                e_channel = channel;
                e_pos = pos - 1;  // 返回空位置，而不是有阻挡的位置
                return true;
            }
        }
    }

    return false;
}

/**
 * 对优先级的车辆进行上路处理，需要考虑预置车辆
 * // FIXME: 因为解决不了预置车辆和优先车辆的优先级冲突，因此决定要么预置上路要么优先上路
 * // incross表示是否处于路口调度阶段
 */
int Road::start_priors(unordered_map<int, Car> &car_dict, int time, bool cars_overload, bool in_cross) {
    // 判空处理
    if(prior_cars_preset.empty() && prior_cars_unpreset.empty())
        return 0;

    // 1. 满足条件的预置车辆和非预置车辆
    vector<int> satisfied_preset_cars_first(0); // 由于出发时间小于当前时间，强行第一顺序
    vector<int> satisfied_preset_cars(0);   // 符合当前时间出发，可以跟其他车辆混合出发,并按ID排序

    // 预置车辆的
    for(int car_id : prior_cars_preset){
        if (car_dict[car_id].carPlanTime < time)
            satisfied_preset_cars_first.push_back(car_id);
        else if (car_dict[car_id].carPlanTime == time)
            satisfied_preset_cars.push_back(car_id);
        else
            break;  // 因为是按出发时间排序的,不满足就表示时间超了
    }
    // 非预置车辆的
    int nums_propasal = min(CARS_ON_SINGLE_ROAD, (int) prior_cars_unpreset.size());    // 推荐数量
    // 必须在不拥堵的情况下加非预置车辆
    for (int i = 0; i < nums_propasal && !cars_overload; i++) {
        satisfied_preset_cars.push_back(prior_cars_unpreset[i]);
    }
    // 进行ID排序
    sort(satisfied_preset_cars.begin(), satisfied_preset_cars.end());



    // 2. 决定上路的车辆和排序
    vector<int> try_cars;
    try_cars = satisfied_preset_cars_first;
    try_cars.insert(try_cars.end(), satisfied_preset_cars.begin(), satisfied_preset_cars.end());

    // 3. 对待上路列表进行处理
    int count = 0;
    for(int car_id : try_cars){
        Car &car_obj = car_dict[car_id];
        if (try_on_road(car_obj, time, in_cross, car_dict)) {
            // 上路成功，抹去上路清单位置
            prior_cars_preset.erase(remove(prior_cars_preset.begin(), prior_cars_preset.end(), car_id),
                                    prior_cars_preset.end());
            prior_cars_unpreset.erase(remove(prior_cars_unpreset.begin(), prior_cars_unpreset.end(), car_id),
                                      prior_cars_unpreset.end());
            count++;
        }
    }
    return count;
}


/**
 * 处理非优先车辆
 * @param car_dict
 * @param time
 * @return
 */
int Road::start_un_priors(unordered_map<int, Car> &car_dict, int time, bool cars_overload, bool in_cross) {
    // 判空处理
    if(unpriors_cars_preset.empty() && unpriors_cars_unpreset.empty())
        return 0;

    // 1. 满足条件的预置车辆和非预置车辆
    vector<int> satisfied_preset_cars_first(0); // 由于出发时间小于当前时间，强行第一顺序
    vector<int> satisfied_preset_cars(0);   // 符合当前时间出发，可以跟其他车辆混合出发,并按ID排序

    // 预置车辆的
    for(int car_id : unpriors_cars_preset){
        if (car_dict[car_id].carPlanTime < time)
            satisfied_preset_cars_first.push_back(car_id);
        else if (car_dict[car_id].carPlanTime == time)
            satisfied_preset_cars.push_back(car_id);
        else
            break;  // 因为是按出发时间排序的,不满足就表示时间超了
    }
    // 非预置车辆的
    int nums_propasal = min(CARS_ON_SINGLE_ROAD, (int) unpriors_cars_unpreset.size());    // 推荐数量
    // 必须在不拥堵的情况下加非预置车辆
    for (int i = 0; i < nums_propasal && !cars_overload; i++) {
        satisfied_preset_cars.push_back(unpriors_cars_unpreset[i]);
    }
    // 进行ID排序
    sort(satisfied_preset_cars.begin(), satisfied_preset_cars.end());



    // 2. 决定上路的车辆和排序
    vector<int> try_cars;
    try_cars = satisfied_preset_cars_first;
    try_cars.insert(try_cars.end(), satisfied_preset_cars.begin(), satisfied_preset_cars.end());

    // 3. 对待上路列表进行处理
    int count = 0;
    for(int car_id : try_cars){
        Car &car_obj = car_dict[car_id];
        // FIXME: 注意区分第一次上路和路口内调度上路，判断堵车的方式不一样
        if (try_on_road(car_obj, time, in_cross, car_dict)) {
            // 上路成功，抹去上路清单位置
            unpriors_cars_preset.erase(remove(unpriors_cars_preset.begin(), unpriors_cars_preset.end(), car_id),
                                       unpriors_cars_preset.end());
            unpriors_cars_unpreset.erase(remove(unpriors_cars_unpreset.begin(), unpriors_cars_unpreset.end(), car_id),
                                         unpriors_cars_unpreset.end());
            count++;
        }
    }
    return count;
}



