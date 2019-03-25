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
                               unordered_map<int, Car> &car_dict, unordered_map<string, Road> &road_dict) {
    topology = topo;
    crossDict = cross_dict;
    carDict = car_dict;
    roadDict = road_dict;
    graph = Graph();
    TIME = 0;
    TIME_STEP = 1;
    result = unordered_map<int, schedule_result>();
    launch_order = vector<int>();
    get_start_list(launch_order);

    crossList = vector<int>(0);
    for (auto &cross : crossDict)
    {
        int cross_id = cross.first;
        crossList.push_back(cross_id);
    }
    sort(crossList.begin(), crossList.end());   // 升序排布
}

/**
 * 初始化上路顺序
 * TODO: 组合放在此处
 * @param order
 */
void trafficManager::get_start_list(vector<int> &order) {
    for (unordered_map<int, Car>::const_iterator item = carDict.begin(); item != carDict.end(); item++) {
        int carID = (*item).first;
        order.push_back(carID);
    }
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
 */
int trafficManager::update_cars(vector<int> &carAtHomeList, vector<int> &carOnRoadList) {
    int carSucceedNum = 0;

    auto iter = launch_order.begin();
    while (iter != launch_order.end()) {
        int car_id = *iter;
        Car &car_obj = carDict[car_id];

        if (car_obj.is_car_waiting_home()) {
            carAtHomeList.push_back(car_id);
        } else if (car_obj.is_car_on_road()) {
            carOnRoadList.push_back(car_id);
        } else if (car_obj.is_ended()) {
            carSucceedNum += 1;
            iter = launch_order.erase(iter);    // 发车列表中去除这个ID
        }

        if (launch_order.empty())
        {
            break;
        }

        ++iter;
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
        if (! carDict[car_id].is_ended())
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
    for (auto &start : topology) {
        int road_begin = start.first;
        for (auto item = topology[road_begin].begin(); item != topology[road_begin].end(); item++) {
            unordered_map<string, int> ends = (*item);
            int road_end = ends["end"];
            string road_name = to_string(road_begin) + "_" + to_string(road_end);
            double road_weight = roadDict[road_name].get_road_weight(1.0);
            // 注意是整形的
            (*item)["weight"] = (int) ((*item)["length"] * 1.0 * (1.0 + road_weight * ROAD_WEIGHTS_CALC));
        }
    }
    Graph graph = create_graph(topology);
    return graph;
}

/**
 * 处理结果并返回
 * @return
 */
unordered_map<int, schedule_result> trafficManager::get_result() {
    for (unordered_map<int, Car>::const_iterator car_item = carDict.begin(); car_item != carDict.end(); car_item++)
    {
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
void trafficManager::inference() {
    // 初始化时间
    TIME = 0;
    // 初始化有向图
    graph = get_new_map();
    // 初始化列表
    vector<int> carAtHomeList(0),carOnRoadList(0);
    update_cars(carAtHomeList, carOnRoadList);
    // 进入调度任务，直至完成
    while(!is_task_completed())
    {
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
        for(int cross_id : crossList)
        {
            Cross &cross = crossDict[cross_id];
            cross.reset_end_flag();
        }

        // 这个While 是刚需，必须要完成道路所有车辆的调度才能进行下一个时间片
        int cross_loop_alert = 0;
        while (any_car_waiting(carOnRoadList))
        {
            // 调度一轮所有路口
            for(int cross_id : crossList)
            {
                Cross &cross = crossDict[cross_id];
                if (!cross.if_cross_ended())
                {
                    cross.update_cross(roadDict, carDict, CROSS_LOOP_TIMES);    // 更新路口
                }
            }


            cross_loop_alert += 1;

            if (cross_loop_alert > LOOPS_TO_DEAD_CLOCK)
            {
                cout << "路口循环调度次数太多进行警告" << endl;
                assert(false);
            }

            if(cross_loop_alert >= LOOPS_TO_UPDATE)
            {
                // TODO: 怂恿堵着的车辆换路线
                // TODO: 高速路开高速车
                // TODO: 定时更新策略

                // 更新 有向图 权重
                graph = get_new_map();
                // 更新 路上车辆 路线
                for (int car_id : carOnRoadList)
                {
                    Car car_o = carDict[car_id];
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
        if(lenOnRoad < CARS_ON_ROAD)
        {
            size_t how_many = min(CARS_ON_ROAD - lenOnRoad, lenAtHome);

            int count_start = 0;
            for (unsigned int i = 0; i < how_many; ++i) {
                int car_id = carAtHomeList[i];
                Car &car_obj = carDict[car_id];
                string road_name = car_obj.try_start(graph, TIME);
                if(road_name != NO_ANSWER)
                {
                    Road &road_obj = roadDict[road_name];
                    if(road_obj.try_on_road(car_obj))   // 尝试上路
                    {
                        count_start += 1;
                        carOnRoadList.push_back(car_obj.carID);
                        lenOnRoad += 1;
                        lenAtHome -= 1;
                    }
                }
            }
            cout << to_string(count_start) + "," + to_string(lenAtHome) + ","+ to_string(lenOnRoad) + ","+ to_string(carsOnEnd) << endl;
        }
    }
    cout << "Tasks Completed! and Schedule Time is: " + to_string(TIME) << endl;
}


