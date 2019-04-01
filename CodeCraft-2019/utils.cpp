//
// Created by shen on 2019/3/24.
//

#include "utils.h"


string strip(const string &str, char ch = ' ') {
    //除去str两端的ch字符
    size_t i = 0;
    while (str[i] == ch)// 头部ch字符个数是 i
        i++;
    size_t j = str.size() - 1;

    while (str[j] == ch) //
        j--;
    return str.substr(i, j + 1 - i);
}


vector<string> split(const string &str, const string &ch = " ") {
    vector<string> ret;
    size_t pos = 0;
    size_t start = 0;
    while ((pos = (str.find(ch, start))) != string::npos) {
        if (pos > start)
            ret.push_back(str.substr(start, pos - start));
        start = pos + ch.size();
    }
    if (str.size() > start)
        ret.push_back(str.substr(start));
    return ret;
}


/// 道路信息读取
/// \param road_path
/// \return
unordered_map<int, unordered_map<string, int>> read_road(string road_path) {
    // 初始化map
    unordered_map<int, unordered_map<string, int>> road_dict;

    // 打开文件
    ifstream road_file(road_path);
    string line;
    if (road_file.is_open()) {
        while (getline(road_file, line))    // 读取每一行
        {
            if (line.rfind('#') == 0)
                continue;
            // 去除首尾括号
            string data;
            data = strip(string(line), '(');
            data = strip(string(data), ')');

            vector<string> datalist = split(data, ",");
            unordered_map<string, int> road;
            road["id"] = stoi(datalist[0]);
            road["length"] = stoi(datalist[1]);
            road["speed"] = stoi(datalist[2]);
            road["channel"] = stoi(datalist[3]);
            road["from"] = stoi(datalist[4]);
            road["to"] = stoi(datalist[5]);
            road["isDuplex"] = stoi(datalist[6]);

            road_dict[road["id"]] = road;
        }
        road_file.close();
    }

    return road_dict;
}


/// 路口信息读取
/// \param cross_path
/// \return
unordered_map<int, unordered_map<string, int>> read_cross(string cross_path) {
    // 初始化map
    unordered_map<int, unordered_map<string, int>> cross_dict;

    // 打开文件
    ifstream cross_file(cross_path);
    string line;
    if (cross_file.is_open()) {
        while (getline(cross_file, line))    // 读取每一行
        {
            if (line.rfind('#') == 0)
                continue;
            // 去除首尾括号
            string data;
            data = strip(string(line), '(');
            data = strip(string(data), ')');

            vector<string> datalist = split(data, ",");
            unordered_map<string, int> cross;
            cross["id"] = stoi(datalist[0]);
            cross["road1"] = stoi(datalist[1]);
            cross["road2"] = stoi(datalist[2]);
            cross["road3"] = stoi(datalist[3]);
            cross["road4"] = stoi(datalist[4]);

            cross_dict[cross["id"]] = cross;
        }
        cross_file.close();
    }

    return cross_dict;
}


/// 车辆信息读取
/// \param cross_path
/// \return
unordered_map<int, unordered_map<string, int>> read_car(string car_path) {
    // 初始化map
    unordered_map<int, unordered_map<string, int>> car_dict;

    // 打开文件
    ifstream car_file(car_path);
    string line;
    if (car_file.is_open()) {
        while (getline(car_file, line))    // 读取每一行
        {
            if (line.rfind('#') == 0)
                continue;
            // 去除首尾括号
            string data;
            data = strip(string(line), '(');
            data = strip(string(data), ')');

            vector<string> datalist = split(data, ",");

            unordered_map<string, int> car;
            car["id"] = stoi(datalist[0]);
            car["from"] = stoi(datalist[1]);
            car["to"] = stoi(datalist[2]);
            car["speed"] = stoi(datalist[3]);
            car["planTime"] = stoi(datalist[4]);
            car["priority"] = stoi(datalist[5]);    // 是否优先
            car["preset"] = stoi(datalist[6]);      // 是否预置

            car_dict[car["id"]] = car;
        }
        car_file.close();
    }

    return car_dict;
}


/**
 * 读取预置车辆信息
 * @param presets_path
 * @return
 */
unordered_map<int, presetCar> read_presetCars(string presets_path) {
    // 初始化map
    unordered_map<int, presetCar> presetCars_dict;
    // 打开文件
    ifstream presetCar_file(presets_path);
    string line;
    if (presetCar_file.is_open()) {
        while (getline(presetCar_file, line))    // 读取每一行
        {
            if (line.rfind('#') == 0)
                continue;

            // 去除首尾括号
            string data;
            data = strip(string(line), '(');
            data = strip(string(data), ')');

            vector<string> datalist = split(data, ",");

            int car_id = stoi(datalist[0]); // 预置车辆编号
            int time = stoi(datalist[1]); // 预置车辆出发时间
            vector<int> routes;     // 预置车辆路径
            for (size_t i = 2; i < datalist.size(); ++i) {
                routes.push_back(stoi(datalist[i]));
            }
            // 预置车结构体
            presetCar car = presetCar(car_id, time, routes);
            // 添加到字典
            presetCars_dict[car.car_id] = car;
        }
        presetCar_file.close();
    }
    return presetCars_dict;
}
