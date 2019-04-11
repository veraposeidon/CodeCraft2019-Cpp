//
// Created by shen on 2019/3/24.
//

#ifndef CODECRAFT_2019_ROAD_H
#define CODECRAFT_2019_ROAD_H

#include "car.h"
#include <vector>

using namespace std;

// 单条道路上路车辆数目
#define CARS_ON_SINGLE_ROAD (1)

class Road {
public:
    int roadID;             // 道路编号
    int roadLength;         // 道路长度
    int roadSpeedLimit;     // 道路限速
    int roadChannel;        // 车道数目
    int roadOrigin;         // 道路起点
    int roadDest;           // 道路终点
    vector<vector<int> > roadStatus;    // 道路详情
    int first_order_car_id; // 出路口第一优先级车辆
    vector<int> prior_cars_preset;      // 本条路负责上路的优先车辆(预置)
    vector<int> prior_cars_unpreset;    // 本条路负责上路的优先车辆(非预置)
    vector<int> unpriors_cars_preset; // 本条路负责上路的非优先车辆(预置)
    vector<int> unpriors_cars_unpreset; // 本条路负责上路的非优先车辆(非预置)
    // 默认构造函数
    Road();
    // 构造函数
    Road(int road_id, int length, int speed_limit, int channel, int origin,
         int dest);

    // 更新道路，时间片内第一次调度
    void update_road(unordered_map<int, Car> &car_dict);    // 注意map以自定义类作为值，需要默认构造函数。

    // 调度车辆(第一轮调度，待出路口只标记为等待)
    void update_car(Car &car_obj, int channel, int grid, unordered_map<int, Car> &car_dict);

    // 移动车辆到指定位置
    void move_car_to(int car_channel, int car_pos, int new_pos, Car &car_obj);

    // 判断车道内某段区域是否有车
    bool has_car(int channel, int start, int end, int &position, int &car_id);

    // 启动时 获取进入道路时的空位置
    bool get_checkin_place_start(int &e_channel, int &e_pos);

    // 调度路口是，获取进入道路时的空位置
    bool get_checkin_place_cross(int &e_channel, int &e_pos, unordered_map<int, Car> &car_dict);

    // 车辆入驻道路
    bool try_on_road(Car &car_obj, int time);

    // 当有车更新到终止态之后，要更新一次当前车道的车辆
    void update_channel(int channel_id, unordered_map<int, Car> &car_dict);

    // 判断道路起始最优一批的几辆车是否存在等待情况
    bool last_row_are_waiting(unordered_map<int, Car> &car_dict);

    // 移动车辆回家
    void move_car_home(Car &car_obj, int time);

    // 获取道路拥堵权重
    double get_road_weight(double dist_k);

    // 获取本条道路的第一优先级车辆
    int get_first_order_car(unordered_map<int, Car> &car_dict);

    // 对优先级的车辆进行上路处理
    int start_priors(unordered_map<int, Car> &car_dict, int time, bool cars_overed);

    // 对非优先车辆进行上路处理
    int start_un_priors(unordered_map<int, Car> &car_dict, int time, bool cars_overed);
};


#endif //CODECRAFT_2019_ROAD_H
