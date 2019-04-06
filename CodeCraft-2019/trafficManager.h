//
// Created by shen on 2019/3/25.
//

#ifndef CODECRAFT_2019_TRAFFICMANAGER_H
#define CODECRAFT_2019_TRAFFICMANAGER_H

// 场上车辆数目
//#define CARS_ON_ROAD  {9000,8500, 8000, 5000, 3000, 2000}    // 大参数跑成绩，小参数保命
#define CARS_ON_ROAD  {500}    // 大参数跑成绩，小参数保命

// 一次上路车辆 基数     动态上路
#define CAR_GET_START_BASE (300)

// 路口全部调度多少次重新更新车辆路线
#define LOOPS_TO_UPDATE (3)

// 路口调度多少次直接判为死锁
#define LOOPS_TO_DEAD_CLOCK (50)

// 路口占比权重
#define ROAD_WEIGHTS_CALC (6)

// 单时间片一个路口循环次数
#define CROSS_LOOP_TIMES (1)

// 随机种子
#define RANDOM_SEED (42)

// 分布系数在拥堵中占的比重
#define DIST_PERCENT (0.5)

// 路口权重调用比例 基数 越小影响越大
#define CROSS_BASE (15)

// 动态调度 抽样频率
// 通过降低抽样频率来加快运行速度
#define UPDATE_FREQUENCE (2)

// 上路方式
//#define START_RANDOM    // 随机上路
//#define START_BY_TIME // 按照预计时间上路
#define START_BY_TIME_AND_PLACE // 按照预计时间和区位分布上路
//#define START_BY_DIRECTION // 按照预计时间和区位分布上路


#include "dijsktra.h"
#include "cross.h"

struct schedule_result {
    int startTime;
    vector<int> passedBy;

    schedule_result() {
        startTime = 0;
        passedBy = vector<int>();
    }

    schedule_result(int time, vector<int> passby) {
        startTime = time;
        passedBy = std::move(passby);
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
    size_t how_many_cars_on_road;

    unordered_map<int, schedule_result> result; // 调度结果
    vector<int> launch_order;   // 车辆待启动序列
    vector<int> launch_order_preset_priors;   // 预置优先车辆
    vector<int> launch_order_preset_notpriors;   // 预置非优先车辆
    vector<int> launch_order_notpreset_priors;   // 非预置优先车辆
    vector<int> launch_order_notpreset_notpriors;   // 非预置非优先车辆

    vector<int> crossList;  // 路口遍历顺序

    // 构造函数
    trafficManager(topology_type &topology_, unordered_map<int, Cross> &cross_dict, unordered_map<int, Car> &car_dict,
                   unordered_map<string, Road> &road_dict, int on_road_cars);

    // 初始化上路顺序
    void get_start_list(vector<int> &order);

    // 判断道路上是否有车等待调度
    bool any_car_waiting(vector<int> &carOnRoadList);

    // 遍历车辆，获取状态, 在发车列表上进行遍历
    int update_cars(vector<int> &carAtHomeList, vector<int> &carOnRoadList);

    // 遍历车辆，更新优先车辆和非优先车辆列表
    void update_prior_cars(vector<int> &carAtHomeList, vector<int> &carNotPriorAtHomeList,
                           unordered_map<int, vector<pair<int, int>>> &carPriorAtHome);

    // 是否所有车辆演算结束
    bool is_task_completed();

    // 更新整个地图的权重，可以融合诸多规则。
    Graph get_new_map();

    // 处理结果
    unordered_map<int, schedule_result> get_result();

    // 推演
    bool inference();

    // 总调度时间
    long long total_schedule_time();

    // 找到堵死路口
    void find_dead_clock();

    // 计算系数因子
    double calc_factor_a(int &first_car_plan_time);

    // 初始化每条道路的优先车辆
    void update_road_prior_cars();

};


#endif // CODECRAFT_2019_TRAFFICMANAGER_H
