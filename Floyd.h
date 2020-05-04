#pragma once

#include"Graph.h"

class Floyd
{
private:
    size_t _vexNum;    // 图的顶点个数 number of vertices  
    size_t _edgeNum;  // 图的边数 number of edges
    std::vector<std::vector<double>> _distTo;   // 记录各顶点对的最短距离(weight) 
    std::vector<std::vector<size_t>> _pathTo;  //  记录各顶点对的最短路径信息（过渡顶点信息）

public:
    //构造函数
    Floyd(const Graph& graph);

    //析构函数
    ~Floyd();

    // Generate Path
    void generatePath(const Graph& graph);

    // 返回到各个顶点的最短距离(weight)
    std::vector<std::vector<double>> distTo() const { return _distTo; };

    //  返回到各个顶点的最短路径的信息
    std::vector<std::vector<size_t>> pathTo() const { return _pathTo; };

    //打印最短路径
    void printPath();
};

