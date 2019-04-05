//
// Created by shen on 2019/3/25.
//

#include <ostream>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <set>
#include "trafficManager.h"

# define INT_M 0x3f3f3f3f

/**
 * 调度中心，构造函数
 * @param topo
 * @param cross_dict
 * @param car_dict
 * @param road_dict
 */
trafficManager::trafficManager(unordered_map<int, Cross> &cross_dict,
                               unordered_map<int, Car> &car_dict, unordered_map<string, Road> &road_dict) {
    crossDict = cross_dict;
    carDict = car_dict;
    roadDict = road_dict;

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
    // 统计每个出发点的车数目
    // 统计权重和历史权重
    unordered_map<int, pair<double, double>> area_ratio;
    size_t min_cars = INT_M; // 最少的车数出发点
    for (auto &id_time : area_dist) {
        int cross_id = id_time.first;
        sort(area_dist[cross_id].begin(), area_dist[cross_id].end(),
             [=](pair<int, int> &a, pair<int, int> &b) { return a.second < b.second; });

        // 最少数目
        if (area_dist[cross_id].size() < min_cars) {
            min_cars = area_dist[cross_id].size();
        }
    }

    // 统计比例
    for (auto &id_time : area_dist) {
        int cross_id = id_time.first;
        double ratio = area_dist[cross_id].size() * 1.0 / min_cars;
        area_ratio[cross_id] = make_pair(ratio, 0.0);
    }

    // 轮流抽出发点   //
    // 1:1轮流采会导致堆积，应该按照比例进行采样   // 这样不会最后堆在同一个出发点
    while (order.size() < carDict.size()) {
        for (auto &id_time : area_dist) {
            int cross_id = id_time.first;
            if (!area_dist[cross_id].empty()) {
                order.push_back(area_dist[cross_id][0].first);
                area_dist[cross_id].erase(area_dist[cross_id].begin());
            }

            // 减掉一次机会
            area_ratio[cross_id].second += area_ratio[cross_id].first - 1.0;

            // 如果累计大于一了，那就再添加一辆车
            if (area_ratio[cross_id].second >= 1.0) {
                if (!area_dist[cross_id].empty()) {
                    order.push_back(area_dist[cross_id][0].first);
                    area_dist[cross_id].erase(area_dist[cross_id].begin());
                }
                // 减掉一次机会
                area_ratio[cross_id].second -= 1.0;
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
 */
int trafficManager::update_cars(vector<int> &carAtHomeList, vector<int> &carOnRoadList) {
    int carSucceedNum = 0;

    auto iter = launch_order.begin();
    while (iter != launch_order.end()) {
        int car_id = *iter;
        Car &car_obj = carDict[car_id];

        if (car_obj.is_car_waiting_home()) {    // 等待出发
            carAtHomeList.push_back(car_id);    // 等待出发的车里存在优先车辆和非优先车辆
            ++iter;
        } else if (car_obj.is_car_on_road()) {  // 已经在路上
            carOnRoadList.push_back(car_id);
            ++iter;
        } else if (car_obj.is_ended()) {    // 已经到达目的地
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

            // 路口权重（附加在与路口相连的道路上）
            int cross_end_call = crossDict[road_end].call_times;
            int cross_start_call = crossDict[road_begin].call_times;
//            int cross_call = max(cross_start_call, cross_end_call);   // 取较大值
            int cross_call = cross_start_call + cross_end_call; // 效果相同，但更慢
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
        // 复赛：预置车辆跳过，不做统计
        if (car_obj.carPreset) {
            continue;
        }
        schedule_result singleResult(car_obj.startTime, car_obj.passed_by);
        result[car_id] = singleResult;
    }

    return result;
}

/**
 * 推演
 */
bool trafficManager::inference() {
    TIME = 0;   // 初始化时间
    graph = get_new_map();  // 初始化有向图
    // 初始化列表
    vector<int> carAtHomeList(0), carOnRoadList(0);
    update_cars(carAtHomeList, carOnRoadList);
    vector<int> carNotPriorAtHomeList(0);   // 非优先待上路车辆
    unordered_map<int, vector<pair<int, int>>> carPriorAtHome;    // 优先待上路车辆
    update_prior_cars(carAtHomeList, carNotPriorAtHomeList, carPriorAtHome);
    size_t lenOnRoad = carOnRoadList.size();    // 路上车辆
    size_t lenAtHome = carNotPriorAtHomeList.size();    // 待出发车辆

    // 进入调度任务，直至完成
    while (!is_task_completed()) {
        // 1. 更新时间片
        TIME += TIME_STEP;

        // 2. 更新所有车道（调度一轮即可）。
        // 2.1 获取道路ID列表 调度顺序无关。同时道路内的车辆调度不涉及优先级
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

        // 路口调度计数，用于判断堵死
        int cross_loop_alert = 0;
        int cars_on_road_in_cross = 0;  // 在路口调度中上路的优先车辆数目
        // 这个While 是刚需，必须要完成道路所有车辆的调度才能进行下一个时间片
        size_t how_many_in_cross = (how_many_cars_on_road - lenOnRoad) / crossList.size() + 1; // 分配的辆数
        while (any_car_waiting(carOnRoadList)) {
            for (int cross_id : crossList) {
                Cross &cross = crossDict[cross_id]; // 路口
                // TODO: 该路口的待调度的优先车辆(后期注意控制车辆数目)
                vector<pair<int, int>> &priority_cars = carPriorAtHome[cross_id];
                size_t cars_num = priority_cars.size();
                int cross_cars = how_many_in_cross;
                if (!cross.if_cross_ended()) {
                    cross.update_cross(roadDict, carDict, CROSS_LOOP_TIMES, TIME, priority_cars, graph, cross_cars);    // 更新一次路口
                }
                cars_on_road_in_cross += cars_num - priority_cars.size();   // 累加
            }

            cross_loop_alert += 1;
            if (cross_loop_alert > LOOPS_TO_DEAD_CLOCK) {
                cout << "**************死锁****************" << endl;
                find_dead_clock();  // 找到堵死的路口
                // assert(false);  // 不直接断言了，保险起见，返回信息重新换参数推演
                return false;
            }

            if (cross_loop_alert >= LOOPS_TO_UPDATE) {
                // TODO: 怂恿堵着的车辆换路线 // TODO: 高速路开高速车 // TODO: 定时更新策略
                graph = get_new_map();  // 更新 有向图 权重
                // 更新 路上车辆 路线
                for (size_t i = 0; i < carOnRoadList.size(); i += UPDATE_FREQUENCE) {
                    int car_id = carOnRoadList[i];
                    Car &car_o = carDict[car_id];
                    car_o.update_new_strategy(graph);
                }
            }
        }
        cout << ("TIME: " + to_string(TIME) + ", LOOPs " + to_string(cross_loop_alert)) << endl;

        // 4. 更新车辆列表
        carAtHomeList.clear();
        carOnRoadList.clear();
        int carsOnEnd = update_cars(carAtHomeList, carOnRoadList);
        carNotPriorAtHomeList.clear();   // 非优先待上路车辆
        carPriorAtHome.clear();    // 优先待上路车辆
        update_prior_cars(carAtHomeList, carNotPriorAtHomeList, carPriorAtHome);
        lenOnRoad = carOnRoadList.size();    // 路上车辆
        lenAtHome = carNotPriorAtHomeList.size();    // 待出发车辆
        int count_start = 0;    // 上路车数
        if (lenOnRoad < how_many_cars_on_road) {
            // 上路数目
            size_t how_many = 0;
            how_many = min(how_many_cars_on_road - lenOnRoad, lenAtHome);
            // TODO: 只负责非优先（优先车辆应该已经在路口调度中压榨到了极致）
            vector<int> car_ordered(carNotPriorAtHomeList.begin(), carNotPriorAtHomeList.begin() + how_many);
            sort(car_ordered.begin(), car_ordered.end());  // 小号优先

            for (unsigned int i = 0; i < how_many; ++i) {
                int car_id = car_ordered[i];
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
        }
        cout << to_string(cars_on_road_in_cross) + "," + to_string(count_start) + "," + to_string(lenAtHome) + "," +
                to_string(lenOnRoad) + "," +
                to_string(carsOnEnd) << endl;
    }

    cout << "Tasks Completed! " << endl;
    cout << "system schedule time is: " + to_string(TIME) << endl;
    cout << "all cars total schedule time: " + to_string(total_schedule_time()) << endl;
    int car_prior_plan_time;
    int factor_a = int (calc_factor_a(car_prior_plan_time));
    int prior_schedule_time = TIME -car_prior_plan_time;
    int T = factor_a * prior_schedule_time + TIME;
    cout << "new schedule time: " + to_string(T) << endl;
    return true;
}

/**
 * 所有车辆调度时间
 * @return
 */
long long trafficManager::total_schedule_time() {
    long long total = 0;
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

/**
 * 更新待上路的优先车辆和非优先车辆
 * @param carAtHomeList
 * @param carNotPriorAtHomeList
 * @param carPriorAtHome
 */
void trafficManager::update_prior_cars(vector<int> &carAtHomeList, vector<int> &carNotPriorAtHomeList,
                                       unordered_map<int, vector<pair<int, int>>> &carPriorAtHome) {
    for (int &car_id:carAtHomeList) {
        if (!carDict[car_id].carPriority) {   // 非优先车辆
            carNotPriorAtHomeList.push_back(car_id);
        } else { // 优先车辆，按出发地点保存id和出发时间，
            int cross_id = carDict[car_id].carFrom;
            int time = carDict[car_id].carPlanTime;
            carPriorAtHome[cross_id].push_back(make_pair(car_id, time)); // 塞入优先车辆ID
        }
    }
    // 每个路口的优先车辆按照出发时间升序排列
    for (auto &cross:carPriorAtHome) {
        int cross_id = cross.first;
//        sort(carPriorAtHome[cross_id].begin(), carPriorAtHome[cross_id].end()); // 按ID排序
//        sort(carPriorAtHome[cross_id].begin(), carPriorAtHome[cross_id].end(),[=](pair<int,int>&a, pair<int,int>&b){return a.second < b.second;});    // 按时间排序
        sort(carPriorAtHome[cross_id].begin(), carPriorAtHome[cross_id].end(),
             [=](pair<int, int> &a, pair<int, int> &b) { return a.first < b.first; });    // 按ID排序
    }
}

/**
 * 计算系数因子a
 * @return
 */
double trafficManager::calc_factor_a(int &first_car_plan_time) {
    int cars_total = 0;
    int cars_prior_total = 0;
    int maxspeed = 0;
    int minspeed = 0x3f3f3f3f;
    int maxspeed_prior = 0;
    int minspeed_prior = 0x3f3f3f3f;
    int timelast = 0;
    int timefirst = 0x3f3f3f3f;
    int timelast_prior = 0;
    int timefirst_prior = 0x3f3f3f3f;
    set<int> starts;
    set<int> starts_prior;
    set<int> ends;
    set<int> ends_prior;
    int first_prior_plan_time = 0x3f3f3f3f; // 最早计划出发时间最早的优先车辆计划出发时间

    for(auto &item :carDict){
        int car_id = item.first;
        Car &car = carDict[car_id];

        cars_total += 1;

        if(car.carSpeed > maxspeed)
            maxspeed = car.carSpeed;
        if(car.carSpeed < minspeed)
            minspeed = car.carSpeed;
        if(car.startTime > timelast)
            timelast = car.startTime;
        if(car.startTime < timefirst)
            timefirst = car.startTime;
        starts.insert(car.carFrom);
        ends.insert(car.carTo);

        // 针对优先级车辆
        if(car.carPriority){

            cars_prior_total +=1;

            if(car.carSpeed > maxspeed_prior)
                maxspeed_prior = car.carSpeed;
            if(car.carSpeed < minspeed_prior)
                minspeed_prior = car.carSpeed;
            if(car.startTime > timelast_prior)
                timelast_prior = car.startTime;
            if(car.startTime < timefirst_prior)
                timefirst_prior = car.startTime;
            starts_prior.insert(car.carFrom);
            ends_prior.insert(car.carTo);

            if(car.carPlanTime < first_prior_plan_time)
                first_prior_plan_time = car.carPlanTime;
        }
    }

    double a = cars_total * 1.0 / cars_prior_total * 0.05 +
            (maxspeed * 1.0 / minspeed) / (maxspeed_prior*1.0/minspeed_prior) * 0.2375 +
            (timelast * 1.0 / timefirst) / (timelast_prior * 1.0 / timefirst_prior) * 0.2375 +
            (starts.size() * 1.0 / starts_prior.size()) * 0.2375 +
            (ends.size() * 1.0 / ends_prior.size()) * 0.2375;
    first_car_plan_time = first_prior_plan_time;
    return a;
}


