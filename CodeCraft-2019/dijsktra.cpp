//
// Created by shen on 2019/3/24.
//


#include "dijsktra.h"
#include <set>
#include <iostream>
#include <cassert>
#include <algorithm>

using namespace std;
#define WEIGHT_MAX 10000
#define SEARCH_END (-122)

topology_type create_topology(const unordered_map<int, unordered_map<string, int> > &road_dict) {
    topology_type topology;

//    for (auto iter = road_dict.begin(); iter!=road_dict.end(); ++iter)
//    {
//
//    }
    // 测试
    for (const auto &road : road_dict) {
        unordered_map<string, int> item = road.second;

        auto start = item["from"];
        auto end = item["to"];
        auto length = item["length"];
        auto road_id = item["id"];
        auto channel = item["channel"];
        unordered_map<string, int> way = {
                {"start",   start},
                {"end",     end},
                {"length",  length},
                {"road_id", road_id},
                {"channel", channel}
        };
        // 正向
        topology[start].push_back(way);

        // 反向
        if (item["isDuplex"] == 1) {
            unordered_map<string, int> way = {
                    {"start",   end},
                    {"end",     start},
                    {"length",  length},
                    {"road_id", road_id},
                    {"channel", channel}
            };

            topology[end].push_back(way);
        }
    }


    return topology;
}

/**
 * 从拓扑数据生成有向图（邻接表）
 * @param topology_dict
 * @return
 */
Graph create_graph(topology_type &topology_dict) {
    Graph graph = Graph();
    for (topology_type::const_iterator item = topology_dict.begin(); item != topology_dict.end(); item++) {
        int road_begin = (*item).first;
        for (auto &road : topology_dict[road_begin]) {
            unordered_map<string, int> ends = road;
            graph.add_edge(ends["start"], ends["end"], ends["weight"]);
        }
    }

    return graph;
}


Graph::Graph() {
    edges = unordered_map<int, vector<int> >();
    weights = map_weight();

//    heapq = unordered_map<int, tuple<double,int>>();
//    unordered_map<tuple<int,int>, double > weights;
}

/// 添加边
/// \param from_node
/// \param to_node
/// \param weight
void Graph::add_edge(const int from_node, const int to_node, const double weight) {
    edges[from_node].push_back(to_node);
    weights[std::make_tuple(from_node, to_node)] = weight;
}

/// 更新权重
/// \param from_node
/// \param to_node
/// \param weight
void Graph::update_weight(const int from_node, const int to_node, const double weight) {
    weights[std::make_tuple(from_node, to_node)] = weight;
}

/// 最短路径搜索
/// \param from_node
/// \param to_node
/// \return
///   dijsktra 最短路径搜索
///   直接实现。时间复杂度：O(n²)
///    致谢： http://benalexkeen.com/implementing-djikstras-shortest-path-algorithm-with-python/
vector<int> Graph::short_path_finding(const int from_node, const int to_node) {

    // shortest_paths是字典，索引为节点，值为tuple(上一个节点，权重)
    unordered_map<int, pair<int, double> > shortest_paths = {{from_node, make_pair(SEARCH_END, 0.0)}};
    int current_node = from_node;
    set<int> visited;
    while (current_node != to_node) {
        visited.insert(current_node);
        vector<int> destinations = edges[current_node];
        double weight_to_current_node = shortest_paths[current_node].second;

//        std::cout << weight_to_current_node;
        for (vector<int>::const_iterator next_node = destinations.begin();
             next_node != destinations.end(); next_node++) {
            double weight = weights[std::make_tuple(current_node, *next_node)] + weight_to_current_node;

            auto got = shortest_paths.find(*next_node);
            if (got == shortest_paths.end())    //不存在
            {
                shortest_paths[*next_node] = make_pair(current_node, weight);
            } else {
                double current_shortest_weight = shortest_paths[*next_node].second;
                if (current_shortest_weight > weight) {
                    shortest_paths[*next_node] = make_pair(current_node, weight);
                }
            }
        }

        unordered_map<int, pair<int, double> > next_destinations;
        for (unordered_map<int, pair<int, double> >::const_iterator node = shortest_paths.begin();
             node != shortest_paths.end(); node++) {
            int node_N = (*node).first;
            auto got = visited.find(node_N);
            if (got == visited.end()) {
                next_destinations[node_N] = shortest_paths[node_N];
            }
        }

        if (next_destinations.empty()) {
            std::cout << "Route Not Possible" << endl;
            assert(!next_destinations.empty());
        }


        // 遍历找最小可能会稍快。
        current_node = (*next_destinations.begin()).first;
        for (unordered_map<int, pair<int, double> >::const_iterator node = next_destinations.begin();
             node != next_destinations.end(); node++) {
            int node_N = (*node).first;
            if (next_destinations[node_N].second < next_destinations[current_node].second) {
                current_node = node_N;
            }
        }
    }

    vector<int> path(0);
    while (current_node != SEARCH_END) {
        path.push_back(current_node);
        int next_node = shortest_paths[current_node].first;
        current_node = next_node;
    }

    reverse(path.begin(), path.end());

    return path;
}
