#include <utility>

//
// Created by shen on 2019/3/25.
//

#ifndef CODECRAFT_2019_TRAFFICMANAGER_H
#define CODECRAFT_2019_TRAFFICMANAGER_H

// 场上车辆数目
//CARS_ON_ROAD = 2500  // 大地图2500辆
#define CARS_ON_ROAD  (200)

// 一次上路车辆 基数     动态上路
#define CAR_GET_START_BASE (300)

// 路口全部调度多少次重新更新车辆路线
#define LOOPS_TO_UPDATE (4)

// 路口调度多少次直接判为死锁
#define LOOPS_TO_DEAD_CLOCK (100)

// 路口占比权重
#define ROAD_WEIGHTS_CALC (3.0)

// 单时间片一个路口循环次数
#define CROSS_LOOP_TIMES (1)


#include "dijsktra.h"
#include "cross.h"

struct schedule_result {
    int startTiem;
    vector<int> passedBy;

    schedule_result() {
        startTiem = 0;
        passedBy = vector<int>();
    }

    schedule_result(int ttime, vector<int> passd) {
        startTiem = ttime;
        passedBy = std::move(passd);
    }
};


class trafficManager {
public:
    topology_type topology; // 拓扑信息
    unordered_map<int, Cross> crossDict;    // 路口对象
    unordered_map<int, Car> carDict;    // 车辆对象
    unordered_map<string, Road> roadDict;   // 道路对象
    Graph graph;    // 图模型
    int TIME;   // 调度系统时间
    int TIME_STEP;    // 调度系统时间单位

    unordered_map<int, schedule_result> result; // 调度结果
    vector<int> launch_order;   // 车辆启动顺序
    vector<int> crossList;  // 路口遍历顺序

    // 构造函数
    trafficManager(topology_type &topo, unordered_map<int, Cross> &cross_dict, unordered_map<int, Car> &car_dict,
                   unordered_map<string, Road> &road_dict);

    // 初始化上路顺序
    void get_start_list(vector<int> &order);

    // 判断道路上是否有车等待调度
    bool any_car_waiting(vector<int> &carOnRoadList);

    // 遍历车辆，获取状态, 在发车列表上进行遍历
    int update_cars(vector<int> &carAtHomeList, vector<int> &carOnRoadList);

    // 是否所有车辆演算结束
    bool is_task_completed();

    // 更新整个地图的权重，可以融合诸多规则。
    Graph get_new_map();

    // 处理结果
    unordered_map<int, schedule_result> get_result();

    // 推演
    void inference();
};


#endif //CODECRAFT_2019_TRAFFICMANAGER_H
