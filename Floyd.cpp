#include"Floyd.h"


//构造函数
Floyd::Floyd(const Graph& graph)
	:_vexNum(graph.vexNum()), _edgeNum(graph.edgeNum()), _distTo(graph.adjMatrix())
{
	_pathTo.assign(_vexNum, std::vector<size_t>(_vexNum, UINT_MAX));

	//矩阵_pathTo的初值为各个边的终点顶点
	for (size_t row = 0; row < _vexNum; ++row) {
		for (size_t col = 0; col < _vexNum; ++col) {
			if (graph.adjMatrix()[row][col] != UINT_MAX) {
				_pathTo[row][col] = col;
			}
		}
	}
}


//析构函数
Floyd::~Floyd()
{
}

void Floyd::generatePath(const Graph& graph)
{
	//三重循环，用于计算每个顶点对的最短路径
	for (size_t bridge = 0; bridge < _vexNum; ++bridge) {
		for (size_t row = 0; row < _vexNum; ++row) {
			for (size_t col = 0; col < _vexNum; ++col) {
				double new_distTo = (_distTo[row][bridge] == UINT_MAX || _distTo[bridge][col] == UINT_MAX) ? UINT_MAX : (_distTo[row][bridge] + _distTo[bridge][col]);
				if (_distTo[row][col] > new_distTo) {
					_distTo[row][col] = new_distTo;
					_pathTo[row][col] = _pathTo[row][bridge];
				}
			}
		}
	}
}

//打印各顶点对的最短路径信息
void Floyd::printPath() {
	std::cout << "Floyd各个顶点对的最短路径：" << std::endl;
	for (size_t row = 0; row < _vexNum; ++row) {
		for (size_t col = 0; col < _vexNum; ++col) {
			std::cout << "v" << std::to_string(row + 1) << "---"
				<< "v" << std::to_string(col + 1) << " weight: "
				<< _distTo[row][col] << "; pathTo: "
				<< " v" << std::to_string(row + 1);
			size_t bridge = _pathTo[row][col];
			//循环输出途径的每条路径。
			while (bridge != col) {
				std::cout << "-->" << "v" << std::to_string(bridge + 1);
				bridge = _pathTo[bridge][col];
			}
			std::cout << "-->" << "v" << std::to_string(col + 1) << std::endl;
		}
	}
}


