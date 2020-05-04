#include "Graph.h"

//构造函数
Graph::Graph()
{
	//输入图的类型，并判断值是否合法
	std::cout << "输入图的种类：1代表有向图，2代表无向图" << std::endl;
	std::cin >> _graphType;
	while (_graphType != 1 && _graphType != 2) {
		std::cout << "输入的图的种类编号不合法，请重新输入：1代表有向图，2代表无向图" << std::endl;
		std::cin >> _graphType;
	}

	//输入顶点数和边数，并检验值是否有效
	std::cout << "输入图的顶点个数和边的条数：" << std::endl;
	std::cin >> _vexNum >> _edgeNum;
	while (!checkVexnum(_vexNum, _edgeNum)) {
		std::cout << "输入的数值不合法，请重新输入" << std::endl;
		std::cin >> _vexNum >> _edgeNum;
	}

	//创建邻接矩阵并赋初值
	_adjMatrix.assign(_vexNum, std::vector<double>(_vexNum, UINT_MAX));

	// 输入边的信息(起点、终点以及权重)，并判断每次输入的值是否合法
	std::cout << "请输入每条边的起点和终点（顶点编号从1开始）以及其权重" << std::endl;
	for (size_t count = 0; count < _edgeNum; ++count) {
		size_t start;
		size_t end;
		double weight;
		std::cin >> start >> end >> weight;

		while (!this->checkEdgeValue(start, end, weight)) {
			std::cout << "输入的边的信息不合法，请重新输入" << std::endl;
			std::cin >> start >> end >> weight;
		}

		//对邻接矩阵赋值
		_adjMatrix[start - 1][end - 1] = weight;
		//对于无向图，认为反向权重与正向权重相等
		if (_graphType == 2) {
			_adjMatrix[end - 1][start - 1] = weight;
		}
	}

	// 如果没有手动输入自循环权重，则设置任意顶点到自己的距离为0
	for (size_t i = 0; i < _vexNum; ++i) {
		if (_adjMatrix[i][i] == UINT_MAX) {
			_adjMatrix[i][i] = 0;
			std::cout << _adjMatrix[i][i];
		}
	}
}


//析构函数
Graph::~Graph()
{
}


//检验输入边数和顶点数的值是否有效
bool Graph::checkVexnum(size_t vexNum, size_t edgeNum)
{
	// graphType 1代表有向图，2代表无向图
	if (_graphType == 1) {
		if (vexNum <= 0 || edgeNum <= 0 || edgeNum > (vexNum * (vexNum - 1))) {
			return false;
		}
	}
	else {
		if (vexNum <= 0 || edgeNum <= 0 || edgeNum > (vexNum * (vexNum - 1) / 2)) {
			return false;
		}
	}
	return true;
}

// 判断每次输入的的边的信息是否合法
bool Graph::checkEdgeValue(size_t start, size_t end, double weight) {
	if (start < 1 || end < 1 || start > _vexNum || end > _vexNum) {
		return false;
	}
	return true;
}

// 打印邻接矩阵
void Graph::print() {
	std::cout << "图的邻接矩阵为：" << std::endl;
	for (size_t count_row = 0; count_row < _vexNum; ++count_row) {
		for (size_t count_col = 0; count_col < _vexNum; ++count_col) {
			if (_adjMatrix[count_row][count_col] == UINT_MAX) {
				std::cout << "∞" << " ";
			}
			else {
				std::cout << _adjMatrix[count_row][count_col] << " ";
			}
		}
		std::cout << std::endl;
	}
}


