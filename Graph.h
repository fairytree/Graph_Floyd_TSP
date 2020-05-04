#pragma once

#include<iostream>
#include<string>
#include<vector>

class Graph
{
private:
	size_t _vexNum;    // 图的顶点个数 number of vertices  
	size_t _edgeNum;   // 图的边数 number of edges
	size_t _graphType;   // 图的类型，1代表有向图，2代表无向图, types of graph, directedgraph is 1, undirectedgraph is 2.
	std::vector<std::vector<double>> _adjMatrix;  //  邻接矩阵, 记录图中顶点和边的关系, adjacency matrix

public:
	//构造函数
	Graph();

	//析构函数
	~Graph();

	//检验输入边数和顶点数的值是否有效
	bool checkVexnum(size_t vexnum, size_t edge);

	// 判断每次输入的的边的信息是否合法
	bool checkEdgeValue(size_t start, size_t end, double weight);

	// 返回顶点个数
	size_t vexNum() const { return _vexNum; };

	// 返回边的条数
	size_t edgeNum() const { return _edgeNum; };

	// 返回邻接矩阵
	std::vector<std::vector<double>> adjMatrix() const { return _adjMatrix; };

	//打印邻接矩阵
	void print();
};


