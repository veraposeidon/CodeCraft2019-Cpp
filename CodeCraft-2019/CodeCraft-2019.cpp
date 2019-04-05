#include <iostream>
#include <ctime>
#include <sstream>
#include "utils.h"
#include "dijsktra.h"
#include "car.h"
#include "road.h"
#include "cross.h"
#include "trafficManager.h"



int main(int argc, char *argv[]) {
    clock_t start, end;
    start = clock();        //程序开始计时

    std::cout << "Begin" << std::endl;

    if (argc < 6) {
        std::cout << "please input args: carPath, roadPath, crossPath, presetAnswerPath, answerPath" << std::endl;
        exit(1);
    }

    std::string carPath(argv[1]);
    std::string roadPath(argv[2]);
    std::string crossPath(argv[3]);
    std::string presetAnswerPath(argv[4]);
    std::string answerPath(argv[5]);

    std::cout << "carPath is " << carPath << std::endl;
    std::cout << "roadPath is " << roadPath << std::endl;
    std::cout << "crossPath is " << crossPath << std::endl;
    std::cout << "presetAnswerPath is " << presetAnswerPath << std::endl;
    std::cout << "answerPath is " << answerPath << std::endl;

    // 第一步：读入文件
    auto car_dict = read_car(carPath);
    auto road_dict = read_road(roadPath);
    auto cross_dict = read_cross(crossPath);
    auto preset_car_dict = read_presetCars(presetAnswerPath);
    auto answer_dict = read_answer(answerPath);


    // 2. 生成车辆对象
    unordered_map<int, Car> cars;
    for (unordered_map<int, unordered_map<string, int>>::const_iterator item = car_dict.begin();
         item != car_dict.end(); item++) {
        int carID = (*item).first;
        bool priority = (car_dict[carID]["priority"] == 1);
        bool preset = (car_dict[carID]["preset"] == 1);
        Car car_ = Car(
                car_dict[carID]["id"],
                car_dict[carID]["from"],
                car_dict[carID]["to"],
                car_dict[carID]["speed"],
                car_dict[carID]["planTime"],
                priority,
                preset
        );
        cars[carID] = car_;
    }
    car_dict.clear();    // 清空。没有清干净

    // 3. 生成道路对象
    unordered_map<string, Road> roads;
    for (unordered_map<int, unordered_map<string, int>>::const_iterator item = road_dict.begin();
         item != road_dict.end(); item++) {
        int roadID = (*item).first;
        Road road1_ = Road(
                road_dict[roadID]["id"],
                road_dict[roadID]["length"],
                road_dict[roadID]["speed"],
                road_dict[roadID]["channel"],
                road_dict[roadID]["from"],
                road_dict[roadID]["to"]);
        string road_name1 = to_string(road1_.roadOrigin) + "_" + to_string(road1_.roadDest);
        roads[road_name1] = road1_;

        // 双向道路反向
        if (road_dict[roadID]["isDuplex"] == 1) {
            Road road2_ = Road(
                    road_dict[roadID]["id"],
                    road_dict[roadID]["length"],
                    road_dict[roadID]["speed"],
                    road_dict[roadID]["channel"],
                    road_dict[roadID]["to"], //注意次序
                    road_dict[roadID]["from"]);
            string road_name2 = to_string(road2_.roadOrigin) + "_" + to_string(road2_.roadDest);
            roads[road_name2] = road2_;
        }
    }
    road_dict.clear();      // 清空。没有清干净

    // 4. 生成路口对象
    unordered_map<int, Cross> crosses;
    for (unordered_map<int, unordered_map<string, int>>::const_iterator item = cross_dict.begin();
         item != cross_dict.end(); item++) {
        int crossID = (*item).first;
        Cross cross_ = Cross(
                cross_dict[crossID]["id"],
                cross_dict[crossID]["road1"],
                cross_dict[crossID]["road2"],
                cross_dict[crossID]["road3"],
                cross_dict[crossID]["road4"],
                roads);
        crosses[crossID] = cross_;
    }
    cross_dict.clear();

    // 5. 拷贝预置车辆信息
    for(auto &preset_car: answer_dict)
    {
        int car_id = preset_car.first;
        cars[car_id].set_preset_route(preset_car.second.real_start_time, preset_car.second.routes);
    }


    // 6. 调度中心

    trafficManager manager = trafficManager(crosses, cars, roads);
    manager.inference();


    // 程序结束用时
    end = clock();
    double end_time = (double) (end - start) / CLOCKS_PER_SEC;
    cout << "program run time:" << end_time << endl;


    return 0;
}