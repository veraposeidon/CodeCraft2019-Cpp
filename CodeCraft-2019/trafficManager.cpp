//
// Created by shen on 2019/3/25.
//

#include <ostream>
#include <iostream>
#include <algorithm>
#include <cassert>
#include "trafficManager.h"

/**
 * 调度中心，构造函数
 * @param topo
 * @param cross_dict
 * @param car_dict
 * @param road_dict
 */
trafficManager::trafficManager(topology_type &topo, unordered_map<int, Cross> &cross_dict,
                               unordered_map<int, Car> &car_dict, unordered_map<string, Road> &road_dict,
                               size_t on_road_cars) {
    topology = topo;
    crossDict = cross_dict;
    carDict = car_dict;
    roadDict = road_dict;
    how_many_cars_on_road = on_road_cars;

    TIME = 0;
    TIME_STEP = 1;
    result = unordered_map<int, schedule_result>();
    launch_order = vector<int>();


    crossList = vector<int>(0);
    for (auto &cross : crossDict) {
        int cross_id = cross.first;
        crossList.push_back(cross_id);
    }
    sort(crossList.begin(), crossList.end());   // 升序排布
    get_start_list(launch_order);
    graph = Graph(crossList);

}

/**
 * 初始化上路顺序
 * TODO: 组合放在此处
 * @param order
 */
void trafficManager::get_start_list(vector<int> &order) {
#ifdef START_RANDOM
    // 随机引擎
    auto random_ = std::default_random_engine(RANDOM_SEED);
    //  车辆列表
    for (unordered_map<int, Car>::const_iterator item = carDict.begin(); item != carDict.end(); item++) {
        int carID = (*item).first;
        order.push_back(carID);
    }
    // 随机打乱
    shuffle(order.begin(), order.end(), random_);
    return ;
#endif

#ifdef START_BY_TIME
    // 建立pair容器
    vector<pair<int, int>> id_time;
    for (unordered_map<int, Car>::const_iterator item = carDict.begin(); item != carDict.end(); item++) {
        int carID = (*item).first;
        int time = carDict[carID].carPlanTime;
        id_time.emplace_back(carID, time);
    }
    sort(id_time.begin(), id_time.end(),[=](pair<int,int>&a, pair<int,int>&b){return a.second < b.second;});
    for(auto &item : id_time)
    {
        order.push_back(item.first);
    }
    return;
#endif

#ifdef START_BY_TIME_AND_PLACE
    // 先获取地方分布
    unordered_map<int, vector<pair<int, int>>> area_dist;
    for (unordered_map<int, Car>::const_iterator item = carDict.begin(); item != carDict.end(); item++) {
        int carID = (*item).first;
        int cross_id = carDict[carID].carFrom;
        int time = carDict[carID].carPlanTime;
        area_dist[cross_id].push_back(make_pair(carID, time));
    }

    // 对每个出发点的车按时间排序
    for (auto &id_time : area_dist) {
        int cross_id = id_time.first;
        sort(area_dist[cross_id].begin(), area_dist[cross_id].end(),
             [=](pair<int, int> &a, pair<int, int> &b) { return a.second < b.second; });
    }

    // 轮流抽出发点
    while (order.size() < carDict.size()) {
        for (auto &id_time : area_dist) {
            int cross_id = id_time.first;
            if (!area_dist[cross_id].empty()) {
                order.push_back(area_dist[cross_id][0].first);
                area_dist[cross_id].erase(area_dist[cross_id].begin());
            }
        }
    }
    assert(order.size() == carDict.size());
#endif
#ifdef START_BY_DIRECTION   // 先东西相向而行 // 再跑其他车辆
    int eastUp = 20;
    int westBegin = 50;

    // 先获取地方分布
    unordered_map<int, vector<pair<int, int>>> area_dist;
    for (unordered_map<int, Car>::const_iterator item = carDict.begin(); item != carDict.end(); item++) {
        int carID = (*item).first;
        int cross_id = carDict[carID].carFrom;
        int time = carDict[carID].carPlanTime;
        area_dist[cross_id].push_back(make_pair(carID, time));
    }

    // 对每个出发点的车按时间排序
    for(auto &id_time : area_dist)
    {
        int cross_id = id_time.first;
        sort(area_dist[cross_id].begin(),area_dist[cross_id].end(),[=](pair<int,int>&a, pair<int,int>&b){return a.second < b.second;});
    }

    // 编号与距离
    vector<pair<int, int> > east_start(0);
    vector<pair<int, int> > west_start(0);
    vector<pair<int, int> > other_start(0);

    for (int i = 1; i < eastUp; ++i) {
        vector<pair<int, int>> this_point = area_dist[i];
        for (int j = 0; j < this_point.size(); ++j) {
            int car_id = this_point[j].first;
            int dis = carDict[car_id].carTo - carDict[car_id].carFrom;
            east_start.push_back(make_pair(car_id, dis));
        }
    }
    sort(east_start.begin(),east_start.end(),[=](pair<int,int>&a, pair<int,int>&b){return a.second > b.second;});
    cout << east_start.size() << endl;
    cout << westBegin << ", " <<  crossList.size() << endl;
    for (int i = westBegin; i <= crossList.size(); ++i) {
        vector<pair<int, int>> this_point = area_dist[i];
        for (int j = 0; j < this_point.size(); ++j) {
            int car_id = this_point[j].first;
            int dis = carDict[car_id].carTo - carDict[car_id].carFrom;
            west_start.push_back(make_pair(car_id, dis));
        }
    }
    sort(west_start.begin(),west_start.end(),[=](pair<int,int>&a, pair<int,int>&b){return a.second > b.second;});
    cout << west_start.size() << endl;

    for (int i = eastUp; i < westBegin; ++i) {
        vector<pair<int, int>> this_point = area_dist[i];
        for (int j = 0; j < this_point.size(); ++j) {
            int car_id = this_point[j].first;
            int dis = carDict[car_id].carTo - carDict[car_id].carFrom;
            other_start.push_back(make_pair(car_id, dis));
        }
    }
    sort(other_start.begin(),other_start.end(),[=](pair<int,int>&a, pair<int,int>&b){return a.second > b.second;});
    cout << other_start.size() << endl;
    while (!east_start.empty() || !west_start.empty())
    {
        if(!east_start.empty())
        {
            order.push_back(east_start[0].first);
            east_start.erase(east_start.begin());
        }
        if(!west_start.empty())
        {
            order.push_back(west_start[0].first);
            west_start.erase(west_start.begin());
        }
    }
    while (!other_start.empty() )
    {
        order.push_back(other_start[0].first);
        other_start.erase(other_start.begin());
    }

    assert(order.size() == carDict.size());
    return ;
#endif
}

/**
 * 判断道路上是否有车等待调度
 * 仅判断在路上的车辆即可
 * @param carOnRoadList
 * @return
 */
bool trafficManager::any_car_waiting(vector<int> &carOnRoadList) {
    for (int car_id : carOnRoadList) {
        if (carDict[car_id].is_car_waiting())
            return true;
    }
    return false;
}

/**
 * 遍历车辆，获取状态
 * @param carAtHomeList
 * @param carOnRoadList
 * @return 时间片内入库的车数
 * 还有一个任务是统计所有车辆总调度时间，即车辆到达时间-预计出发时间
 */
int trafficManager::update_cars(vector<int> &carAtHomeList, vector<int> &carOnRoadList) {
    int carSucceedNum = 0;

    auto iter = launch_order.begin();
    while (iter != launch_order.end()) {
        int car_id = *iter;
        Car &car_obj = carDict[car_id];

        if (car_obj.is_car_waiting_home()) {
            carAtHomeList.push_back(car_id);
            ++iter;
        } else if (car_obj.is_car_on_road()) {
            carOnRoadList.push_back(car_id);
            ++iter;
        } else if (car_obj.is_ended()) {
            carSucceedNum += 1;

            iter = launch_order.erase(iter);    // 发车列表中去除这个ID
        }
    }
    return carSucceedNum;
}

/**
 * 是否所有车辆演算结束
 * @return
 */
bool trafficManager::is_task_completed() {
    for (unordered_map<int, Car>::const_iterator item = carDict.begin(); item != carDict.end(); item++) {
        int car_id = (*item).first;
        if (!carDict[car_id].is_ended())
            return false;
    }
    return true;
}

/**
 * 重点，更新整个地图的权重，可以融合诸多规则。
 * @param road_dict
 * @return
 */
Graph trafficManager::get_new_map() {
    // 1. 根据道路和路口更新权重
    for (auto &start : topology) {
        int road_begin = start.first;
        for (auto item = topology[road_begin].begin(); item != topology[road_begin].end(); item++) {
            topo ends = (*item);
            int road_end = ends.end;
            string road_name = to_string(road_begin) + "_" + to_string(road_end);
            double road_weight = roadDict[road_name].get_road_weight(DIST_PERCENT);

            // 道路权重附加
            (*item).weight = (*item).length * (1.0 + road_weight * ROAD_WEIGHTS_CALC);
//            (*item).weight = roadDict[road_name].roadLength * roadDict[road_name].roadChannel *1.0 / roadDict[road_name].roadSpeedLimit * (1.0 + road_weight * ROAD_WEIGHTS_CALC);

            // 路口权重附加路口权重只在加在了通往该路口的边
            // 从该路口出发的边没有加上
            int cross_end_call = crossDict[road_end].call_times;
            int cross_start_call = crossDict[road_begin].call_times;
            int cross_call = max(cross_start_call, cross_end_call);
            (*item).weight = (*item).weight * (1.0 + cross_call * 1.0 / CROSS_BASE);
        }
    }

    Graph graph = create_graph(topology, crossList);
    return graph;
}

/**
 * 处理结果并返回
 * @return
 */
unordered_map<int, schedule_result> trafficManager::get_result() {
    for (unordered_map<int, Car>::const_iterator car_item = carDict.begin(); car_item != carDict.end(); car_item++) {
        int car_id = (*car_item).first;
        Car car_obj = carDict[car_id];
        schedule_result singleResult(car_obj.startTime, car_obj.passed_by);
        result[car_id] = singleResult;
    }

    return result;
}

/**
 * 推演
 */
bool trafficManager::inference() {
    // 初始化时间
    TIME = 0;
    // 初始化有向图
    graph = get_new_map();
    // 初始化列表
    vector<int> carAtHomeList(0), carOnRoadList(0);
    update_cars(carAtHomeList, carOnRoadList);
    // 进入调度任务，直至完成
    while (!is_task_completed()) {
        // 1. 更新时间片
        TIME += TIME_STEP;

        // 2. 更新所有车道（调度一轮即可）。
        // 2.1 获取道路ID列表 调度顺序无关
        for (auto &road : roadDict) {
            string road_name = road.first;
            roadDict[road_name].update_road(carDict);
        }

        // 3. 更新所有路口
        // 重置路口标记
        for (int cross_id : crossList) {
            Cross &cross = crossDict[cross_id];
            cross.reset_end_flag();
        }

        // 这个While 是刚需，必须要完成道路所有车辆的调度才能进行下一个时间片
        int cross_loop_alert = 0;
        while (any_car_waiting(carOnRoadList)) {
            // 调度一轮所有路口
            for (int cross_id : crossList) {
                Cross &cross = crossDict[cross_id];
                if (!cross.if_cross_ended()) {
                    cross.update_cross(roadDict, carDict, CROSS_LOOP_TIMES, TIME);    // 更新路口
                }
            }


            cross_loop_alert += 1;

            if (cross_loop_alert > LOOPS_TO_DEAD_CLOCK) {
                cout
                        << "路口循环调度次数太多进行警告从头来过*******************************************************警告线**************************************"
                        << endl;
                cout
                        << "路口循环调度次数太多进行警告从头来过*******************************************************警告线**************************************"
                        << endl;

                // 找到堵死的路口
                find_dead_clock();

                // assert(false);  // 不直接断言了，保险起见，返回信息重新换参数推演
                return false;
            }

            if (cross_loop_alert >= LOOPS_TO_UPDATE) {
                // TODO: 怂恿堵着的车辆换路线
                // TODO: 高速路开高速车
                // TODO: 定时更新策略

                // 更新 有向图 权重
                graph = get_new_map();

                // 更新 路上车辆 路线
                for (size_t i = 0; i < carOnRoadList.size(); i += UPDATE_FREQUENCE) {
                    int car_id = carOnRoadList[i];
                    Car &car_o = carDict[car_id];
                    car_o.update_new_strategy(graph);
                }
            }
        }
        cout << ("TIME: " + to_string(TIME) + ", LOOPs " + to_string(cross_loop_alert)) << endl;

        // 4. 处理准备上路的车辆
        carAtHomeList.clear();
        carOnRoadList.clear();
        int carsOnEnd = update_cars(carAtHomeList, carOnRoadList);
        size_t lenOnRoad = carOnRoadList.size();
        size_t lenAtHome = carAtHomeList.size();

        // TODO: 动态更改地图车辆容量
        // TODO: 动态上路数目
        if (lenOnRoad < how_many_cars_on_road) {
            size_t how_many = 0;
            // 所谓半动态
            how_many = min(how_many_cars_on_road - lenOnRoad, lenAtHome);

//            if (lenAtHome < how_many_cars_on_road) {
//                how_many = min(lenAtHome, size_t(how_many_cars_on_road / 4));
//            } else {
//                how_many = min(how_many_cars_on_road - lenOnRoad, lenAtHome);
//            }
//            how_many =  min(CARS_ON_ROAD - lenOnRoad, lenAtHome);

            int count_start = 0;
            vector<int> carordered(carAtHomeList.begin(), carAtHomeList.begin() + how_many);
            sort(carordered.begin(), carordered.end());  //小号优先

            for (unsigned int i = 0; i < how_many; ++i) {
                int car_id = carordered[i];
                Car &car_obj = carDict[car_id];

                // 判断道路挤不挤，挤就不上路
                if (crossDict[car_obj.carFrom].call_times > 10) {
                    continue;
                }

                string road_name = car_obj.try_start(graph, TIME);
                if (road_name != NO_ANSWER) {
                    Road &road_obj = roadDict[road_name];
                    if (road_obj.try_on_road(car_obj))   // 尝试上路
                    {
                        count_start += 1;
                        carOnRoadList.push_back(car_obj.carID);
                        lenOnRoad += 1;
                        lenAtHome -= 1;
                    }
                }
            }
            cout << to_string(count_start) + "," + to_string(lenAtHome) + "," + to_string(lenOnRoad) + "," +
                    to_string(carsOnEnd) << endl;
        }
    }
    cout << "Tasks Completed! " << endl;
    cout << "system schedule time is: " + to_string(TIME) << endl;
    cout << "all cars total schedule time: " + to_string(total_schedule_time()) << endl;
    return true;
}

int trafficManager::total_schedule_time() {
    int total = 0;
    for (auto &car : carDict) {
        int car_id = car.first;
        int time = carDict[car_id].arriveTime - carDict[car_id].carPlanTime;
        total += time;
    }
    return total;
}



void trafficManager::find_dead_clock() {
    int max_calltimes = 0;
    int max_cross_id = 0;
    for (auto &cross : crossDict) {
        int cross_id = cross.first;
        int calltimes = crossDict[cross_id].call_times;

        if (calltimes >= max_calltimes) {
            max_cross_id = cross_id;
        }
    }

    cout << "Dead Clock: " << max_cross_id << endl;
}


