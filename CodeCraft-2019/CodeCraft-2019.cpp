#include "iostream"
#include "utils.h"
#include "dijsktra.h"

#include<ctime>



int main(int argc, char *argv[])
{
    clock_t start,end;
    start=clock();		//程序开始计时

    std::cout << "Begin" << std::endl;
	
	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
	
	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);
	
	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;
	
	// 第一步：读入文件
	auto car_dict = read_car(carPath);
    auto road_dict = read_road(roadPath);
    auto cross_dict = read_cross(crossPath);

    // 第二步：处理数据
    // 1. 生成拓扑字典
    auto topologyDict = create_topology(road_dict);
//	Graph graph;


	// TODO:process
	// TODO:write output file


    //程序结束用时
    end=clock();
    double end_time=(double)(end-start)/CLOCKS_PER_SEC;

    cout<<"Total time:"<<end_time<<endl;


	return 0;
}