#pragma once

#include"Graph.h"
#include"Floyd.h"
#include <bitset>

class TSP
{
private:
	size_t _totalRoomNumber; // 顶点的总数
	unsigned int _startNode;   //起始顶点
	unsigned int _allRoomVisited;  //结束时，所有房间（顶点）被访问过的状态
	double _minPathCost;  //最短路径cost
	std::vector<unsigned int> _path;  //最短路径

public:
	// 构造函数
	TSP(unsigned int startNode, const Graph& graph);

	// 析构函数
	~TSP();

	// 生成遍历所有顶点需要途经的具体路径信息
	void generatePath(const Floyd& floyd);

	// 递归求解最短路径
	double tspRecursive(unsigned int robotPosition, unsigned int& roomVisitedState,
		std::vector<std::vector<double>>& cost,
		std::vector<std::vector<unsigned int>>& pathTo,
		const Floyd& floyd);

	// 打印遍历所有顶点的最短路径信息
	void printPath();
};


