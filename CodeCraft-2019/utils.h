#include <utility>

//
// Created by shen on 2019/3/24.
//

#ifndef CODECRAFT_2019_UTILS_H
#define CODECRAFT_2019_UTILS_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <unordered_map>
using namespace std;

// 预置车辆信息结构体
struct presetCar {
    int car_id;
    int real_start_time;
    vector<int> routes;

    presetCar() {
        car_id = -1;
        real_start_time = -1;
        routes = vector<int>();
    }

    presetCar(int id, int time, vector<int> rout_) {
        car_id = id;
        real_start_time = time;
        routes = std::move(rout_);
    }
};

unordered_map<int,unordered_map<string, int>> read_road(string road_path);

unordered_map<int, unordered_map<string, int>> read_cross(string cross_path);

unordered_map<int, unordered_map<string, int>> read_car(string car_path);

unordered_map<int, presetCar> read_presetCars(string presets_path);

unordered_map<int, presetCar> read_answer(string answers_path);


#endif //CODECRAFT_2019_UTILS_H
