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
 * @param topology
 * @param cross_dict
 * @param car_dict
 * @param road_dict
 */
trafficManager::trafficManager(topology_type &topology_, unordered_map<int, Cross> &cross_dict,
                               unordered_map<int, Car> &car_dict, unordered_map<string, Road> &road_dict,
                               int on_road_cars) {
    topology = topology_;
    crossDict = cross_dict;
    carDict = car_dict;
    roadDict = road_dict;
    how_many_cars_on_road = (size_t) on_road_cars;

    TIME = 0;
    TIME_STEP = 1;
    result = unordered_map<int, schedule_result>();
    launch_order = vector<int>();

    // 路口升序排布列表
    crossList = vector<int>(0);
    for (auto &cross : crossDict) {
        int cross_id = cross.first;
        crossList.push_back(cross_id);
    }
    sort(crossList.begin(), crossList.end());

    // 根据总体分布生成序列
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
    carAtHomeList.clear();
    carOnRoadList.clear();

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
    initialize_road_prior_cars_and_normal_cars();                        // 初始化每条道路的优先车辆集合

    // 初始化列表
    vector<int> carAtHomeList(0), carOnRoadList(0);
    update_cars(carAtHomeList, carOnRoadList);
    size_t lenOnRoad = carOnRoadList.size();    // 路上车辆
    size_t lenAtHome = carAtHomeList.size();    // 待出发车辆
    bool cars_overed = false;
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

        // 2.2 TODO: 每条路上路优先车辆
        // FIXME: 目前只在车道标定后丢车，不在路口调度后丢车，后期添加在路口调度时塞车
        // 判断控制场上车数
        if (lenOnRoad > how_many_cars_on_road)
            cars_overed = true;
        else
            cars_overed = false;

        int priors_count = 0;   // 当前时间片优先车辆的上路数目
        for (auto &road : roadDict) {
            string road_name = road.first;
            // 路口如果挤就不上路了
            bool local_overed = false;
            if (crossDict[roadDict[road_name].roadOrigin].call_times >= BANED_CAR_ON ||
                crossDict[roadDict[road_name].roadDest].call_times >= BANED_CAR_ON)
                local_overed = true;

            priors_count += roadDict[road_name].start_priors(carDict, TIME, cars_overed || local_overed);
        }

        // 2.3 由于2.2进行了上路处理，因此需要更新在路的车辆
        update_cars(carAtHomeList, carOnRoadList);

        // 3. 更新所有路口
        // 重置路口标记
        for (int cross_id : crossList) {
            Cross &cross = crossDict[cross_id];
            cross.reset_end_flag();
        }

        // 路口调度计数，用于判断堵死
        int cross_loop_alert = 0;
        // 直到上路的车辆都标记位调度完成
        while (any_car_waiting(carOnRoadList)) {
            for (int cross_id : crossList) {
                Cross &cross = crossDict[cross_id]; // 路口
                if (!cross.if_cross_ended()) {
                    cross.update_cross(roadDict, carDict, CROSS_LOOP_TIMES, TIME, graph);    // 更新一次路口
                }
            }

            cross_loop_alert += 1;
            if (cross_loop_alert > LOOPS_TO_DEAD_CLOCK) {
                cout << "**************Dead Clock****************" << endl;
                find_dead_clock();  // 找到堵死的路口
                // assert(false);  // 不直接断言了，保险起见，返回信息重新换参数推演
                return false;
            }
        }

        // 更换路线 放在while外面会加速，减少无意义的更新路线
        if (cross_loop_alert >= LOOPS_TO_UPDATE) {
            // FIXME: 暂时无缘，先跑出复赛地图再说  TODO: 怂恿堵着的车辆换路线 // TODO: 高速路开高速车 // TODO: 定时更新策略
            graph = get_new_map();  // 更新 有向图 权重
            // 更新 路上车辆 路线
            for (size_t i = 0; i < carOnRoadList.size(); i += UPDATE_FREQUENCE) {
                int car_id = carOnRoadList[i];
                Car &car_o = carDict[car_id];
                car_o.update_new_strategy(graph);
            }
        }

        cout << ("TIME: " + to_string(TIME) + ", LOOPs " + to_string(cross_loop_alert)) << endl;

        // 4. 更新车辆列表
        int carsOnEnd = update_cars(carAtHomeList, carOnRoadList);  // 计算路口调度后回家的车辆
        lenOnRoad = carOnRoadList.size();    // 路上车辆
        lenAtHome = carAtHomeList.size();    // 待出发车辆

        if (lenOnRoad > how_many_cars_on_road)
            cars_overed = true;
        else
            cars_overed = false;

        int count_start = priors_count;    // 上路车数
        for (auto &road : roadDict) {
            string road_name = road.first;
            // 路口如果挤就不上路了
            bool local_overed = false;
            if (crossDict[roadDict[road_name].roadOrigin].call_times >= BANED_CAR_ON ||
                crossDict[roadDict[road_name].roadDest].call_times >= BANED_CAR_ON)
                local_overed = true;

            count_start += roadDict[road_name].start_un_priors(carDict, TIME, cars_overed || local_overed);  // 每条路上路
        }

        update_cars(carAtHomeList, carOnRoadList);
        lenOnRoad = carOnRoadList.size();    // 路上车辆
        lenAtHome = carAtHomeList.size();    // 待出发车辆
        cout << "priors: " + to_string(priors_count) + ", "
                + "totalStart: " + to_string(count_start) + ", "
                + "ON_ROAD: " + to_string(lenOnRoad) + ", "
                + "WAITING_HOME: " + to_string(lenAtHome) + ", "
                + "ARRIVED: " + to_string(carsOnEnd) << endl;
    }

    // 5. 计算调度时间
    cout << "Task Completed! " << endl;
    cout << "End Time: " + to_string(TIME) << endl;


    int prior_time; // 优先车辆调度时间
    long long total_all, total_pri;  // 车辆和优先车辆总调度时间
    total_schedule_time(total_all, total_pri, prior_time);
    double factor_a, factor_b;  // 两个系数
    calc_factor_a(factor_a, factor_b);
    int T_e = (int) (factor_a * prior_time + TIME);
    long long T_esum = (int) (factor_b * total_pri + total_all);
    cout << "schedule time: " + to_string(T_e) << endl;
    cout << "schedule time total: " + to_string(T_esum) << endl;
    return true;
}

/**
 * 所有车辆调度时间
 * @return
 */
void trafficManager::total_schedule_time(long long &total_all, long long &total_pri, int &first_car_plan_time) {
    total_all = 0;
    total_pri = 0;
    int first_prior_car_time = 0x3f3f3f3f; // 最早计划出发时间最早的优先车辆计划出发时间
    int last_prior_car_time = 0;
    for (auto &car : carDict) {
        int car_id = car.first;
        int time = carDict[car_id].arriveTime - carDict[car_id].carPlanTime;
        total_all += time;
        if (carDict[car_id].carPriority) {
            total_pri += time;
            if (carDict[car_id].startTime < first_prior_car_time) {
                first_prior_car_time = carDict[car_id].carPlanTime;
            }

            if (carDict[car_id].arriveTime > last_prior_car_time)
                last_prior_car_time = carDict[car_id].arriveTime;
        }
    }
    first_car_plan_time = last_prior_car_time - first_prior_car_time;
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
 * 计算系数因子a
 * @return
 */
void trafficManager::calc_factor_a(double &a, double &b) {
    int cars_total = 0;         // 车辆总数
    int cars_prior_total = 0;   // 优先车辆数
    int maxspeed = 0;           // 所有车辆最高速
    int minspeed = 0x3f3f3f3f;  // 所有车辆最低速
    int maxspeed_prior = 0;     // 优先车辆最高车速
    int minspeed_prior = 0x3f3f3f3f;    // 优先车辆最低车速
    int timelast = 0;           // 所以车辆最晚出发时间
    int timefirst = 0x3f3f3f3f; // 所有车辆最早出发时间
    int timelast_prior = 0;     // 优先车辆最晚出发时间
    int timefirst_prior = 0x3f3f3f3f;   // 优先车辆最早出发时间
    set<int> starts;            // 所有车辆出发地分布
    set<int> starts_prior;      // 优先车辆出发地分布
    set<int> ends;              // 所有车辆终止点分布
    set<int> ends_prior;        // 优先车辆终止点分布
    for (auto &item :carDict) {
        int car_id = item.first;
        Car &car = carDict[car_id];
        cars_total += 1;
        if (car.carSpeed > maxspeed)
            maxspeed = car.carSpeed;
        if (car.carSpeed < minspeed)
            minspeed = car.carSpeed;
        if (car.carPlanTime > timelast)
            timelast = car.carPlanTime;
        if (car.carPlanTime < timefirst)
            timefirst = car.carPlanTime;
        starts.insert(car.carFrom);
        ends.insert(car.carTo);

        // 针对优先级车辆
        if (car.carPriority) {
            cars_prior_total += 1;
            if (car.carSpeed > maxspeed_prior)
                maxspeed_prior = car.carSpeed;
            if (car.carSpeed < minspeed_prior)
                minspeed_prior = car.carSpeed;
            if (car.carPlanTime > timelast_prior)
                timelast_prior = car.carPlanTime;
            if (car.carPlanTime < timefirst_prior)
                timefirst_prior = car.carPlanTime;
            starts_prior.insert(car.carFrom);
            ends_prior.insert(car.carTo);
        }
    }

    assert(size_t (cars_total) == carDict.size());
    a = cars_total * 1.0 / cars_prior_total * 0.05 +
               (maxspeed * 1.0 / minspeed) / (maxspeed_prior * 1.0 / minspeed_prior) * 0.2375 +
               (timelast * 1.0 / timefirst) / (timelast_prior * 1.0 / timefirst_prior) * 0.2375 +
               (starts.size() * 1.0 / starts_prior.size()) * 0.2375 +
               (ends.size() * 1.0 / ends_prior.size()) * 0.2375;
    b = cars_total * 1.0 / cars_prior_total * 0.8 +
        (maxspeed * 1.0 / minspeed) / (maxspeed_prior * 1.0 / minspeed_prior) * 0.05 +
        (timelast * 1.0 / timefirst) / (timelast_prior * 1.0 / timefirst_prior) * 0.05 +
        (starts.size() * 1.0 / starts_prior.size()) * 0.05 +
        (ends.size() * 1.0 / ends_prior.size()) * 0.05;
}

/**
 * 初始化每条道路的优先车辆
 */
void trafficManager::initialize_road_prior_cars_and_normal_cars() {
    // 初始化车辆数目
    for (auto &car : carDict) {
        int car_id = car.first;
        string road_name = carDict[car_id].on_road_name(graph);
        if (carDict[car_id].carPriority) {
            if(carDict[car_id].carPreset){
                // 优先车辆 (预置)
                roadDict[road_name].prior_cars_preset.push_back(car_id);
            }else{
                // 优先车辆 (非预置)
                roadDict[road_name].prior_cars_unpreset.push_back(car_id);
            }
        }else{
            // 非优先车辆
            if(carDict[car_id].carPreset){
                // 优先车辆 (预置)
                roadDict[road_name].unpriors_cars_preset.push_back(car_id);
            }else{
                // 优先车辆 (非预置)
                roadDict[road_name].unpriors_cars_unpreset.push_back(car_id);
            }
        }
    }

    size_t count = 0;
    // 对车辆进行排序
    for (auto &road_item : roadDict){
        string road_name = road_item.first;
        Road &road = roadDict[road_name];

        // 对优先预置车辆上路顺序进行调整
        // 预置车辆由于固定了时间，所以上路的顺序有两个决定，第一是出发时间，第二才是车辆ID
        // 对预置车辆上路顺序进行调整
        sort(road.prior_cars_preset.begin(), road.prior_cars_preset.end(), [=](int &a, int &b) {
            if(carDict.at(a).carPlanTime < carDict.at(b).carPlanTime)
                return true;
            else if(carDict.at(a).carPlanTime > carDict.at(b).carPlanTime)
                return false;
            else{
                return a < b;   // 按ID排序
            }});
        count += road.prior_cars_preset.size();
        // 对优先非预置车辆上路顺序进行调整
        // FIXME : 非预置的优先车辆由于并没有固定出发时间，因此什么时候出发完全是生成的，因此上路的顺序只需要符合ID小优先即可，也应当符合能上就上原则，不然很难处理好第一遍调度和路口调度完之后的上路问题
        sort(road.prior_cars_unpreset.begin(), road.prior_cars_unpreset.end(),
             [=](int &a, int &b) { return a < b; });   // 对ID进行排序
        count += road.prior_cars_unpreset.size();
        // 非优先预置
        sort(road.unpriors_cars_preset.begin(), road.unpriors_cars_preset.end(), [=](int &a, int &b) {
            if(carDict.at(a).carPlanTime < carDict.at(b).carPlanTime)
                return true;
            else if(carDict.at(a).carPlanTime > carDict.at(b).carPlanTime)
                return false;
            else{
                return a < b;   // 按ID排序
            }});
        count += road.unpriors_cars_preset.size();
        // 非优先非预置
        sort(road.unpriors_cars_unpreset.begin(), road.unpriors_cars_unpreset.end(),
             [=](int &a, int &b) { return a < b; });
        count += road.unpriors_cars_unpreset.size();
    }
    assert(count == carDict.size());
}


