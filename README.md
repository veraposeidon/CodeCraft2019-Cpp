# 2019华为软挑 赛后源码整理

2019华为软件精英挑战赛代码Repo
- 江山赛区队伍：今天是2019年3月29日农历二月廿三多云
- 初赛赛区第三
- 复赛赛区十四

## 代码结构

- CodeCraft-2019	
  - CMakeLists.txt
  - CodeCraft-2019.cpp：主程序 Main函数入口
  - trafficManager.cpp / trafficManager.h：模拟器（根据判题逻辑实时更新车辆/道路/路口状态）和调度器（车辆上路策略和更新路径策略）
  - car.cpp / car.h：车辆类（记录车辆对象静态信息+更新车辆动态信息）
  - road.cpp / road.h：道路类（记录道路对象静态信息+更新道路动态信息+道路内移动车辆）
  - cross.cpp / cross.h：路口类（记录路口对象静态信息+跨路口调度车辆）
  - dijsktra.cpp / dijsktra.h：图模型构建+最短路径搜索
  - utils.cpp / utils.h：赛题文件读取

## 最短路径搜索

**Dijkstra’s Shortest Path Algorithm**

先使用邻接矩阵表征，时间复杂度 O(v^2)；后来使用 邻接表+优先队列，时间复杂度O(ElogV)。

Ps：第一次感受到降低时间复杂度的重要性

参考链接：

[Dijkstra’s Shortest Path Algorithm using priority_queue of STL](<https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/>)

[Single-Source Shortest Path：Dijkstra's Algorithm](http://alrightchiu.github.io/SecondRound/single-source-shortest-pathdijkstras-algorithm.html)

## 发车策略+动态换路+路权函数

跟大神的差距就差在这了。一个人没下功夫琢磨（忙，忙，忙![img](3223F32C.png)）。

### 发车策略

很low的策略。

* 初赛（按车辆发车）
  * 车辆按照起点分布分组，按照出发时间排序。每个地点依次抽一辆车放入出发序列。
  * 设置场上运行车辆数目上限。

* 复赛（按道路发车）
  * 初始化所有车辆的路径，将车辆分配给该车起始道路。
  * 每条道路对管辖的车辆进行排序（分优先序列和非优先序列）。
  * 设置场上运行车辆数目上限。
  * 遍历道路，在上限内每条道路依次发车（1-2辆）。

### 动态换路

很low的策略。

路口对象对时间片内调度次数进行统计，越多则表示该路口车辆情况越不乐观。

时间片内判断最大的路口调度次数，超过阈值，则场上车辆进行抽样，更新路径（当前位置至目的地）（更新一半或更少是防止出现扎堆现象）。

### 路权函数

很low的策略。

两个因素：

* 道路：统计道路内车辆数目，计算占比，得到权重。
* 路口：统计时间片内路口调度次数，作为权重，更新到连接该路口的道路。

以上两因素，更新整张地图的拓扑，从而更新抽象图（用于最短路径搜索）。




