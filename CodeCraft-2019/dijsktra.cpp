#include <utility>

//
// Created by shen on 2019/3/24.
//


#include "dijsktra.h"
#include <set>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <queue>

using namespace std;

#define SEARCH_END (-122)
# define INF 0x3f3f3f3f

topology_type create_topology(const unordered_map<int, unordered_map<string, int> > &road_dict) {
    // 初始化
    topology_type topology;

    // 测试
    for (const auto &road : road_dict) {
        unordered_map<string, int> item = road.second;

        auto start = item["from"];
        auto end = item["to"];
        auto length = item["length"];
        auto road_id = item["id"];
        auto channel = item["channel"];
        topo way(start, end, length, road_id, channel);

        // 正向
        topology[start].push_back(way);

        // 反向
        if (item["isDuplex"] == 1) {
            topo way2(end, start, length, road_id, channel);
            topology[end].push_back(way2);
        }
    }


    return topology;
}

/**
 * 从拓扑数据生成有向图（邻接表）
 * @param topology_dict
 * @return
 */
Graph create_graph(topology_type &topology_dict, vector<int> cross_list) {
    Graph graph = Graph(std::move(cross_list));
    for (topology_type::const_iterator item = topology_dict.begin(); item != topology_dict.end(); item++) {
        int road_begin = (*item).first;
        for (auto &road : topology_dict[road_begin]) {
            topo ends = road;
            graph.add_edge(ends.start, ends.end, ends.weight);
        }
    }

    return graph;
}


Graph::Graph(vector<int> crossList) {
    edges = unordered_map<int, vector<int> >();
    weights = map_weight();

    cross_list = std::move(crossList); // 留作备份
    vertexNum = cross_list.size();  // 顶点数目

    // 邻接表大小
    adj = vector<vector<iPair > >(vertexNum);

    // 创建对照表
    for (size_t i = 0; i < cross_list.size(); ++i) {
        checkMap[cross_list[i]] = i;
    }

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
    // 添加到邻接表
    int from_node_index = checkMap[from_node];
    int to_node_index = checkMap[to_node];
    addEdge(from_node_index, to_node_index, weight);
}

/// 更新权重
/// \param from_node
/// \param to_node
/// \param weight
void Graph::update_weight(const int from_node, const int to_node, const double weight) {
    weights[std::make_tuple(from_node, to_node)] = weight;

    // 查找邻接表并修改
    int from_node_index = checkMap[from_node];
    int to_node_index = checkMap[to_node];

    for (auto &iter : adj[from_node_index]) {
        if(iter.first == to_node_index)
        {
            iter.second = weight;
        }
    }
}

/// 最短路径搜索
/// \param from_node
/// \param to_node
/// \return
///   dijsktra 最短路径搜索
///   直接实现。时间复杂度：O(n²)
///    致谢： http://benalexkeen.com/implementing-djikstras-shortest-path-algorithm-with-python/
vector<int> Graph::short_path_finding(const int from_node, const int to_node) {

    return shortestPath_binary(from_node,to_node);
    // 屏蔽上面那句然后进行正常的dijsktra
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

/**
 * 邻接表添加边
 * @param adj
 * @param u
 * @param v
 * @param wt
 */
void Graph::addEdge(int u, int v, double wt) {
    adj[u].push_back(make_pair(v, wt));
}

/**
 * 大恩大德
 * binary_heap 最短路径搜索
 * http://alrightchiu.github.io/SecondRound/single-source-shortest-pathdijkstras-algorithm.html
 * https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
 * @param src
 * @param target
 * @return
 */
typedef pair<double, int> ority_pair;
vector<int> Graph::shortestPath_binary(int src, int target) {
    vector<int > result(0);

    int src_index = checkMap[src];
    int target_index = checkMap[target];

    priority_queue<ority_pair, vector<ority_pair>, greater<ority_pair> > pq;
    vector<double> dist(vertexNum, INF);
    vector<int> path(0);
    // Insert source itself in priority queue and initialize
    // its distance as 0.
    pq.push(make_pair(0, src_index));
    dist[src_index] = 0;

    // predecessor 记录
    vector<int > pred(vertexNum, -1);

    while (!pq.empty()) {
        // The first vertex in pair is the minimum distance
        // vertex, extract it from priority queue.
        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = pq.top().second;

        pq.pop();
        // Get all adjacent of u.
        for (auto x : adj[u]) {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = x.first;
            double weight = x.second;
            // If there is shorted path to v through u.
            if (dist[v] > dist[u] + weight) {
                // Updating distance of v
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));

                pred[v] = u;
            }
        }
    }

    // 置换成真实顶点
    int index = target_index;
    while (-1 != index)
    {
        result.push_back(cross_list[index]);
        index = pred[index];
    }
    reverse(result.begin(), result.end());
    return result;
}

Graph::Graph() {
    edges = unordered_map<int, vector<int> >();
    weights = map_weight();

    cross_list = vector<int>(); // 留作备份
    vertexNum = cross_list.size();  // 顶点数目

    // 邻接表大小
    adj = vector<vector<iPair > >();

//    // 创建对照表
//    for (size_t i = 0; i < cross_list.size(); ++i) {
//        checkMap[cross_list[i]] = i;
//    }
}
