//
// Created by shen on 2019/3/24.
//

#ifndef CODECRAFT_2019_UTILS_H
#define CODECRAFT_2019_UTILS_H

#include <iostream>
#include <fstream>
#include <string>

#include <map>
using namespace std;

map<int,map<string, int>> read_road(string road_path);
map<int,map<string, int>> read_cross(string road_path);
map<int,map<string, int>> read_car(string road_path);


#endif //CODECRAFT_2019_UTILS_H
