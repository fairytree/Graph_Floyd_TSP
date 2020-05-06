#include "TSP.h"

//Time complexity - O(2^n * n^2)
//Space complexity - O(n * 2^n)

//构造函数
TSP::TSP(unsigned int startNode, const Graph& graph)
{
	// 房间（顶点）总数为图的顶点总数
	_totalRoomNumber = graph.vexNum();

	// 用bits来表示“每一个顶点是否被访问过的状态”
	// 例如一共有6个房间（顶点），初始状态为000001（右边第一个数字代表第1个房间，机器人从第1个房间开始，故默认第1个房间已被访问)
	// 顶点访问状态的值为111000时表示，第1、2、3个房间尚未访问过，第4、5、6个房间已被访问过
	// 当所有房间全部访问完毕的时候，最终的值为111111
	_allRoomVisited = (1 << _totalRoomNumber) - 1;

	// "startNode-1"是因为用户输入顶点的编号是从1开始的
	_startNode = startNode - 1;

	// 初始化最短路径成本为极大值
	_minPathCost = DBL_MAX;
}

//析构函数
TSP::~TSP()
{
}


//生成遍历所有顶点需要途经的具体路径信息
void TSP::generatePath(const Floyd& floyd)
{
	// cost 记录机器人从当前房间位置及房间访问状态"进入下一房间及状态所需要的成本(cost)
	std::vector<std::vector<float>> cost(_totalRoomNumber, std::vector<float>((1 << _totalRoomNumber), 0));

	// PathTo 记录遍历所有房间（顶点）的最短访问顺序
	std::vector<std::vector<unsigned int>> pathTo(_totalRoomNumber, std::vector<unsigned int>((1 << _totalRoomNumber), -1));

	// 默认机器人的起始位置（顶点）已被访问过
	unsigned int roomVisitedState = 1 << _startNode;

	// _minPathCost 记录遍历所有顶点的最小成本之和
	_minPathCost = tspRecursive(_startNode, roomVisitedState, cost, pathTo, floyd);

	// 生成遍历所有顶点需要途经的具体路径信息
	// currentPosition表示机器人目前所在位置，nexPosition表示机器人下一个位置
	unsigned int currentPosition = _startNode;
	unsigned int nextPosition = currentPosition;
	while (true) {
		//根据Floyd的各顶点间最短路径，循环输出机器人访问下一个顶点需要经过的过渡顶点
		size_t bridge = floyd.pathTo()[currentPosition][nextPosition];
		while (bridge != nextPosition) {
			_path.push_back(bridge);
			bridge = floyd.pathTo()[bridge][nextPosition];
		}
		_path.push_back(nextPosition);

		//根据TSP生成的访问顶点的顺序，继续访问下一个顶点，直到全部访问完毕
		unsigned int nextRoomToVisit = pathTo[nextPosition][roomVisitedState];
		if (nextRoomToVisit == -1) {
			break;
		}

		// nextVisitedState表示当下一个房间被访问后，房间被访问过的状态
		unsigned int nextVisitedState = roomVisitedState | (1 << nextRoomToVisit);
		roomVisitedState = nextVisitedState;
		currentPosition = nextPosition;
		nextPosition = nextRoomToVisit;
	}

	// 当所有房间都访问完毕后，生成回到起点的具体路径
	size_t bridge = floyd.pathTo()[nextPosition][_startNode];
	while (bridge != _startNode) {
		_path.push_back(bridge);
		bridge = floyd.pathTo()[bridge][_startNode];
	}
	_path.push_back(_startNode);
}


// 递归求解最短路径, 即求解从当前机器人位置及房间访问状态，到终点（所有房间被全部访问）的最小成本及路径
float TSP::tspRecursive(unsigned int robotPosition, unsigned int& roomVisitedState,
	std::vector<std::vector<float>>& cost,
	std::vector<std::vector<unsigned int>>& pathTo,
	const Floyd& floyd)
{
	// 当所有房间全部被访问过后，返回从当前位置回到起点的成本（cost).
	if (roomVisitedState == _allRoomVisited) {
		return floyd.distTo()[robotPosition][_startNode];
	}

	// 如果成本已经计算过，直接使用
	if (cost[robotPosition][roomVisitedState] != NULL) {
		return cost[robotPosition][roomVisitedState];
	}

	// minCost表示从当前机器人位置及房间访问状态，到终点（所有房间被全部访问）的最小成本
	float minCost = DBL_MAX;
	int nextRoomToVisit = -1;
	// 机器人的下一个访问位置可以是任一房间（顶点)
	for (int nextRoomIndex = 0; nextRoomIndex < _totalRoomNumber; nextRoomIndex++) {
		// 跳过已被访问过的房间（顶点）
		if ((roomVisitedState & (1 << nextRoomIndex)) != 0) {
			continue;
		}

		// nextVisitedState表示当下一个房间被访问后，房间被访问过的新状态
		unsigned int nextVisitedState = roomVisitedState | (1 << nextRoomIndex);
		float newCost = floyd.distTo()[robotPosition][nextRoomIndex] + tspRecursive(nextRoomIndex, nextVisitedState, cost, pathTo, floyd);

		//当访问某一个房间(顶点)的总成本小于其他选择的时候，将该房间设定为机器人访问的下一位置
		if (newCost < minCost) {
			minCost = newCost;
			nextRoomToVisit = nextRoomIndex;
		}
	}

	pathTo[robotPosition][roomVisitedState] = nextRoomToVisit;
	return cost[robotPosition][roomVisitedState] = minCost;
}

// 打印遍历所有顶点的最短路径信息
void TSP::printPath()
{
	std::cout << "Travelling_Salesman_Problem Solution: " << std::endl;
	std::cout << "Path: ";
	for (auto n : _path) {
		std::cout << n + 1 << "-->";
	}

	std::cout << "\nPath cost: " << _minPathCost << std::endl;
}
