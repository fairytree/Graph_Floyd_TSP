#pragma once

#include"Graph.h"
#include"Floyd.h"
#include <bitset>
#include <deque>

class TSP
{
private:
	size_t _totalRoomNumber; // ���������
	unsigned int _startNode;   //��ʼ����
	unsigned int _allRoomVisited;  //����ʱ�����з��䣨���㣩�����ʹ���״̬
	float _minPathCost;  //���·��cost
	std::deque<unsigned int> _path;  //���·��

public:
	// ���캯��
	TSP(unsigned int startNode, const Graph& graph);

	// ��������
	~TSP();

	// ���ɱ������ж�����Ҫ;���ľ���·����Ϣ
	void generatePath(const Floyd& floyd);

	// �ݹ�������·��
	float tspRecursive(unsigned int robotPosition, unsigned int& roomVisitedState,
		std::deque<std::deque<float>>& cost,
		std::deque<std::deque<unsigned int>>& pathTo,
		const Floyd& floyd);

	// ��ӡ�������ж�������·����Ϣ
	void printPath();
};


