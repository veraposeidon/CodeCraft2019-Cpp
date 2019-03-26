//
// Created by shen on 2019/3/24.
//

#ifndef CODECRAFT_2019_DIJSKTRA_H
#define CODECRAFT_2019_DIJSKTRA_H

#include <unordered_map>
#include <vector>
#include <string>
using  namespace std;

typedef unordered_map<int, vector<unordered_map<string, int>>> topology_type;

// 创建拓扑图
topology_type create_topology(const unordered_map<int, unordered_map<string, int>> &road_dict);




// 定义一个tuple做key值
// 致谢： http://coliru.stacked-crooked.com/a/dede823b40af4662
typedef std::tuple<size_t , size_t> key_weight;

struct key_hash : public std::unary_function<key_weight, std::size_t>
{
    std::size_t operator()(const key_weight& k) const
    {
        return static_cast<size_t>(std::get<0>(k) * 10000000 + std::get<1>(k));     // TODO: 注意啊，保持独立，编号为ID1-ID2。
    }
};

struct key_equal : public std::binary_function<key_weight, key_weight, bool>
{
    bool operator()(const key_weight& v0, const key_weight& v1) const
    {
        return (
                std::get<0>(v0) == std::get<0>(v1) &&
                std::get<1>(v0) == std::get<1>(v1)
        );
    }
};


typedef std::unordered_map<const key_weight, double ,key_hash,key_equal> map_weight;

// 用于定义邻接表 // 权重放前边
typedef pair<int,int> iPair;

class Graph{
public:
    unordered_map<int, vector<int>> edges;  // 边
    map_weight weights;     // 权重

    vector<vector<iPair > > adj;     // 邻接表
    size_t vertexNum;          // 顶点数量
    vector<int > cross_list;
    unordered_map <int, int> checkMap;  // 顶点对应标号

    Graph();
    Graph(vector<int> crossList);

    // 添加边
    void add_edge(int from_node, int to_note, double weight);

    // 邻接表添加权重
    void addEdge(int u, int v, int wt);

    // 更新权重
    void update_weight(int from_node, int to_node, double weight);

    // 最短路径搜索(普通方式)
    vector<int> short_path_finding(int from_node, int to_node);

    // 最短路径搜索（binary heap）
    vector<int> shortestPath_binary(int src, int target);
    // Binary Heap
};

// 创建邻接表
Graph create_graph(topology_type &topology_dict, vector<int> cross_list);

#endif //CODECRAFT_2019_DIJSKTRA_H
