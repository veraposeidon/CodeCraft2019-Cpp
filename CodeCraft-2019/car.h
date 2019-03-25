//
// Created by shen on 2019/3/24.
//

#ifndef CODECRAFT_2019_CAR_H
#define CODECRAFT_2019_CAR_H

#include "dijsktra.h"
#include <unordered_map>
#include <vector>
#include <string>

using namespace std;
#define NO_ANSWER ("NONE")

enum CarStatus {
    WAITING_HOME=122,                   // 在家准备出发
    ON_ROAD_STATE_END=123,              // 在路上，调度完毕
    ON_ROAD_STATE_WAITING=124,          // 等待调度
    ON_ROAD_STATE_WAITING_OUT=125,      // 出路口等待调度
    ON_ROAD_STATE_WAITING_INSIDE=126,   // 不出路口等待调度
    SUCCEED=127                         // 成功抵达终点
};


struct GPS {
    int roadID;
    int channel;
    int pos;
    int now;
    int next;
    GPS(int r_i, int ch, int po, int no, int ne)
    {
        roadID = r_i;
        channel = ch;
        pos = po;
        now = no;
        next = ne;
    }
    GPS()
    {
        roadID = -1;
        channel = -1;
        pos = -1;
        now = -1;
        next = -1;
    }

};


class Car {
public:
    int carID;              // 车辆编号
    int carFrom;            // 车辆起点
    int carTo;              // 车辆终点
    int carSpeed;           // 车辆速度
    int carPlanTime;        // 预计发车时间
    CarStatus carStatus;    // 车辆状态
    int startTime;          // 实际发车车间
    GPS carGPS;        // 定位
    vector<int> strategy;   // 规划路径
    vector<int> passed_by;  // 记录路过的路段，该结果为最终结果

    // 构造函数
    Car();

    Car(int car_id, int origin, int destination, int speed, int plan_time);

    // 判断是否结束
    bool is_ended();

    // 更新车辆的GPS记录
    void mark_new_pos(int road_id, int channel, int pos, int this_cross, int next_cross);

    // 尝试启动，找最佳路径并返回下一路段名称
    string try_start(Graph &graph, int time);

    // 更改状态为等待处理
    void change2waiting();

    // 更改状态为处理完成
    void change2end();

    // 更改状态为到达终点
    void change2success();

    // 更改状态为出路口等待调度
    void change2waiting_out();

    // 更改为不出路口等待调度状态
    void change2waiting_inside();

    // 判断是否在路上
    bool is_car_on_road();

    // 判断是否在路上等待调度
    bool is_car_waiting();

    // 判断车辆是否等待调度出路口
    bool is_car_waiting_out();

    // 判断车辆是否等待在家
    bool is_car_waiting_home();

    // 判断车辆是否调度结束
    bool is_car_end_state();

    // 判断车辆前方是否终点即可
    bool is_car_way_home();

    // 判断下一条路,需要判断是否到终点
    string next_road_name(int cross_id);

    // 更新策略的时候一定要注意不走回头路
    void update_new_strategy(Graph &graph);

};


#endif //CODECRAFT_2019_CAR_H
