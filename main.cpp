#include"Floyd.h"
//#include"Floyd_function.h"   //Floyd Algorithm函数
#include "Graph.h"
#include"TSP.h"

int main()
{
	// 创建图，并打印；
	Graph graph;
	graph.print();

	// 用Floyd algorithm(Class)计算任意两个顶点之间的最短距离及路径，并打印；
	Floyd floyd(graph);
	floyd.generatePath(graph);
	floyd.printPath();

	// 可选择用Floyd algorithm(function)计算任意两个顶点之间的最短距离及路径，并打印；
	//floyd(graph);

	// 用Travelling_Salesman_Problem(TSP) Algorithm计算遍历所有顶点的最短路径信息，并打印
	// 1代表从第一个顶点开始，可从任一顶点开始
	TSP tspSolver(1, graph);  
	tspSolver.generatePath(floyd);
	tspSolver.printPath();

	return 0;
}


//  无向图测试例子
	//  输入图的种类：1代表有向图，2代表无向图
	//	2
	//	输入图的顶点个数和边的条数：
	//	11 12
	//	请输入每条边的起点和终点（顶点编号从1开始）以及其权重
	//	1 5 10
	//	3 5 10
	//	4 5 10
	//	5 2 1
	//	2 6 10
	//	6 7 10
	//	2 8 10
	//	5 8 1
	//	3 6 1
	//	8 9 10
	//	9 10 10
	//	10 11 10
	//  TSP path:  1-->5-->2-->6-->7-->6-->3-->5-->4-->5-->8-->9-->10-->11-->10-->9-->8-->5-->1
	//  TSP cost:  144



	//  有向图测试例子
	//  输入图的种类：1代表有向图，2代表无向图
	//	1
	//	输入图的顶点个数和边的条数：
	//	8 18
	//	请输入每条边的起点和终点（顶点编号从1开始）以及其权重
	//	1 5 10
	//	5 1 5
	//	3 5 10
	//	5 3 1
	//	4 5 10
	//	5 4 10
	//	5 2 1
	//	2 5 5
	//	2 6 10
	//	6 2 1
	//	6 7 10
	//	7 6 10
	//	2 8 10
	//	8 2 10
	//	5 8 1
	//	8 5 1
	//	3 6 1
	//	6 3 10
	//  TSP path:  1-->5-->3-->6-->7-->6-->2-->5-->4-->5-->8-->5-->1
	//  TSP cost:  65
