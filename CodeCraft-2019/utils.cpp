//
// Created by shen on 2019/3/24.
//

#include "utils.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>


string strip(const string &str, char ch = ' ') {
    //除去str两端的ch字符
    int i = 0;
    while (str[i] == ch)// 头部ch字符个数是 i
        i++;
    auto j = static_cast<int>(str.size() - 1);
    while (str[j] == ch) //
        j--;
    return str.substr(static_cast<unsigned long long int>(i), static_cast<unsigned long long int>(j + 1 - i));
}


vector<string> split(const string &str, const string &ch = " ") {
    vector<string> ret;
    int pos = 0;
    int start = 0;
    while ((pos = static_cast<int>(str.find(ch, static_cast<unsigned long long int>(start)))) != string::npos) {
        if (pos > start)
            ret.push_back(str.substr(static_cast<unsigned long long int>(start),
                                     static_cast<unsigned long long int>(pos - start)));
        start = static_cast<int>(pos + ch.size());
    }
    if (str.size() > start)
        ret.push_back(str.substr(static_cast<unsigned long long int>(start)));
    return ret;
}


/// 道路信息读取
/// \param road_path
/// \return
map<int, map<string, int>> read_road(string road_path) {
    // 初始化map
    map<int, map<string, int>> road_dict;

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

            vector<string> datalist = split(data, ", ");
            map<string, int> road;
            road.insert(pair<string, int>("id", stoi(datalist[0])));
            road.insert(pair<string, int>("length", stoi(datalist[1])));
            road.insert(pair<string, int>("speed", stoi(datalist[2])));
            road.insert(pair<string, int>("channel", stoi(datalist[3])));
            road.insert(pair<string, int>("from", stoi(datalist[4])));
            road.insert(pair<string, int>("to", stoi(datalist[5])));
            road.insert(pair<string, int>("isDuplex", stoi(datalist[6])));

            road_dict.insert(pair<int, map<string, int>>(road["id"], road));

        }
        road_file.close();
    }

    return road_dict;
}


/// 路口信息读取
/// \param cross_path
/// \return
map<int, map<string, int>> read_cross(string cross_path) {
    // 初始化map
    map<int, map<string, int>> cross_dict;

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

            vector<string> datalist = split(data, ", ");
            map<string, int> cross;
            cross.insert(pair<string, int>("id", stoi(datalist[0])));
            cross.insert(pair<string, int>("road1", stoi(datalist[1])));
            cross.insert(pair<string, int>("road2", stoi(datalist[2])));
            cross.insert(pair<string, int>("road3", stoi(datalist[3])));
            cross.insert(pair<string, int>("road4", stoi(datalist[4])));

            cross_dict.insert(pair<int, map<string, int>>(cross["id"], cross));

        }
        cross_file.close();
    }

    return cross_dict;
}



/// 车辆信息读取
/// \param cross_path
/// \return
map<int, map<string, int>> read_car(string car_path) {
    // 初始化map
    map<int, map<string, int>> car_dict;

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

            vector<string> datalist = split(data, ", ");
            map<string, int> car;
            car.insert(pair<string, int>("id", stoi(datalist[0])));
            car.insert(pair<string, int>("from", stoi(datalist[1])));
            car.insert(pair<string, int>("to", stoi(datalist[2])));
            car.insert(pair<string, int>("speed", stoi(datalist[3])));
            car.insert(pair<string, int>("planTime", stoi(datalist[4])));

            car_dict.insert(pair<int, map<string, int>>(car["id"], car));

        }
        car_file.close();
    }

    return car_dict;
}