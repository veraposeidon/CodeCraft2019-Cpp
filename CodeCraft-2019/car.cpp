//
// Created by shen on 2019/3/24.
//

#include <c++/algorithm>
#include "dijsktra.h"
#include "car.h"


Car::Car() {
    carID = -1;
    carFrom = -1;
    carTo = -1;
    carSpeed = -1;
    carPlanTime = -1;
    carStatus = WAITING_HOME;
    startTime = -1;

    // 初始化GPS
    carGPS = GPS();
    // 策略
    strategy = vector<int>(0);
    // 路线
    passed_by = vector<int>(0);
}

Car::Car(int car_id, int origin, int destination, int speed, int plan_time) {
    carID = car_id;
    carFrom = origin;
    carTo = destination;
    carSpeed = speed;
    carPlanTime = plan_time;
    carStatus = WAITING_HOME;
    startTime = 0;

    // 初始化GPS
    carGPS = GPS();
    // 策略
    strategy = vector<int>(0);
    // 路线
    passed_by = vector<int>(0);
}

/// 判断是否到达目的地
/// \return
bool Car::is_ended() {
    return carStatus == SUCCEED;
}

/// 更新车辆的GPS记录
/// \param road_id
/// \param channel
/// \param pos
/// \param this_cross
/// \param next_cross
void Car::mark_new_pos(int road_id, int channel, int pos, int this_cross, int next_cross) {
    // 标记位置
    carGPS.roadID = road_id;
    carGPS.channel = channel;
    carGPS.pos = pos;
    carGPS.now = this_cross;
    carGPS.next = next_cross;

    // 标记状态,车辆调度结束
    carStatus = ON_ROAD_STATE_END;

    // 记录经过路段
    if ((passed_by.empty()) || ((!passed_by.empty()) && (road_id != passed_by.back()))) {
        passed_by.push_back(road_id);
    }
}

/// 尝试启动，找最佳路径并返回下一路段名称
/// \param graph
/// \param time
/// \param suceed
/// \return
string Car::try_start(Graph &graph, int time) {

    // 如果已经在路上就不需要再启动了
    if (carStatus != WAITING_HOME) {
        return NO_ANSWER;
    }
    // 或者没到时间
    if (time < carPlanTime) {
        return NO_ANSWER;
    }

    // 1. 起点，终点和路径
    int start = carFrom;
    int end = carTo;
    // 最佳路径
    strategy = graph.short_path_finding(start, end);
    int now_cross = strategy[0];
    int next_cross = strategy[1];
    // 2. 下段路名称
    string name = to_string(now_cross) + "_" + to_string(next_cross);
    // 3. 时间
    startTime = time;
    return name;
}

/***
 * 更改状态为等待处理
 */
void Car::change2waiting() {
    carStatus = ON_ROAD_STATE_WAITING;
}

/***
 * 更改状态为处理完成
 */
void Car::change2end() {
    carStatus = ON_ROAD_STATE_END;
}

/***
 * 更改状态为到达终点
 */
void Car::change2success() {
    carStatus = SUCCEED;
}

/***
 * 更改状态为出路口等待调度
 */
void Car::change2waiting_out() {
    carStatus = ON_ROAD_STATE_WAITING_OUT;
}

/***
 * 更改为不出路口等待调度状态
 */
void Car::change2waiting_inside() {
    carStatus = ON_ROAD_STATE_WAITING_INSIDE;

}

/**
 * 判断是否在路上
 */
bool Car::is_car_on_road() {
    return is_car_waiting() || is_car_end_state();
}

/**
 * 判断是否在路上等待调度
 */
bool Car::is_car_waiting() {
    return carStatus == ON_ROAD_STATE_WAITING || carStatus == ON_ROAD_STATE_WAITING_INSIDE ||
           carStatus == ON_ROAD_STATE_WAITING_OUT;
}

/**
 * 判断车辆是否等待调度出路口
 * @return
 */
bool Car::is_car_waiting_out() {
    return carStatus == ON_ROAD_STATE_WAITING_OUT;
}

/**
 * 判断车辆是否等待在家
 * @return
 */
bool Car::is_car_waiting_home() {
    return carStatus == WAITING_HOME;
}

/**
 * 判断车辆是否调度结束
 * @return
 */
bool Car::is_car_end_state() {
    return carStatus == ON_ROAD_STATE_END;
}

/**
 * 判断车辆前方是否终点即可
 * @return
 */
bool Car::is_car_way_home() {
    int next_cross = carGPS.next;
    return next_cross == carTo;
}

/**
 * 判断下一条路,需要判断是否到终点
 * @param cross_id
 * @return
 */
string Car::next_road_name(int cross_id) {
    if (cross_id == carTo)
        return NO_ANSWER;

    auto index_iter = find(strategy.begin(), strategy.end(), cross_id);
    index_iter++;
    int next_cross = *(index_iter);

    string road_name = to_string(cross_id) + "_" + to_string(next_cross);

    return road_name;
}

/**
 * 更新策略的时候一定要注意不走回头路
 * @param graph
 */
void Car::update_new_strategy(Graph &graph) {
    if (is_car_way_home())    // 回家路上没有用
        return;

    int this_cross = carGPS.now;
    int next_cross = carGPS.next;

    strategy = graph.short_path_finding(next_cross, carTo); // 下个路口到家的路

    // 判断走没有所在的路，要是有，就重新更新下Graph,重新找最优路径
    if (this_cross == strategy[1]) {
        // 深拷贝效率低，原有权重替换即可
        double origin_weight = graph.weights[make_tuple(next_cross, this_cross)];
        // 更换权重
        graph.update_weight(next_cross, this_cross, 1000);
        // 重新规划路线
        strategy = graph.short_path_finding(next_cross, carTo); // 下个路口到家的路
        // 换回原有权重
        graph.update_weight(next_cross, this_cross, origin_weight);
    }
}





