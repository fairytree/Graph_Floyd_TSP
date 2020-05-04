#include "Graph.h"

//���캯��
Graph::Graph()
{
	//����ͼ�����ͣ����ж�ֵ�Ƿ�Ϸ�
	std::cout << "����ͼ�����ࣺ1��������ͼ��2��������ͼ" << std::endl;
	std::cin >> _graphType;
	while (_graphType != 1 && _graphType != 2) {
		std::cout << "�����ͼ�������Ų��Ϸ������������룺1��������ͼ��2��������ͼ" << std::endl;
		std::cin >> _graphType;
	}

	//���붥�����ͱ�����������ֵ�Ƿ���Ч
	std::cout << "����ͼ�Ķ�������ͱߵ�������" << std::endl;
	std::cin >> _vexNum >> _edgeNum;
	while (!checkVexnum(_vexNum, _edgeNum)) {
		std::cout << "�������ֵ���Ϸ�������������" << std::endl;
		std::cin >> _vexNum >> _edgeNum;
	}

	//�����ڽӾ��󲢸���ֵ
	_adjMatrix.assign(_vexNum, std::vector<double>(_vexNum, UINT_MAX));

	// ����ߵ���Ϣ(��㡢�յ��Լ�Ȩ��)�����ж�ÿ�������ֵ�Ƿ�Ϸ�
	std::cout << "������ÿ���ߵ������յ㣨�����Ŵ�1��ʼ���Լ���Ȩ��" << std::endl;
	for (size_t count = 0; count < _edgeNum; ++count) {
		size_t start;
		size_t end;
		double weight;
		std::cin >> start >> end >> weight;

		while (!this->checkEdgeValue(start, end, weight)) {
			std::cout << "����ıߵ���Ϣ���Ϸ�������������" << std::endl;
			std::cin >> start >> end >> weight;
		}

		//���ڽӾ���ֵ
		_adjMatrix[start - 1][end - 1] = weight;
		//��������ͼ����Ϊ����Ȩ��������Ȩ�����
		if (_graphType == 2) {
			_adjMatrix[end - 1][start - 1] = weight;
		}
	}

	// ���û���ֶ�������ѭ��Ȩ�أ����������ⶥ�㵽�Լ��ľ���Ϊ0
	for (size_t i = 0; i < _vexNum; ++i) {
		if (_adjMatrix[i][i] == UINT_MAX) {
			_adjMatrix[i][i] = 0;
			std::cout << _adjMatrix[i][i];
		}
	}
}


//��������
Graph::~Graph()
{
}


//������������Ͷ�������ֵ�Ƿ���Ч
bool Graph::checkVexnum(size_t vexNum, size_t edgeNum)
{
	// graphType 1��������ͼ��2��������ͼ
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

// �ж�ÿ������ĵıߵ���Ϣ�Ƿ�Ϸ�
bool Graph::checkEdgeValue(size_t start, size_t end, double weight) {
	if (start < 1 || end < 1 || start > _vexNum || end > _vexNum) {
		return false;
	}
	return true;
}

// ��ӡ�ڽӾ���
void Graph::print() {
	std::cout << "ͼ���ڽӾ���Ϊ��" << std::endl;
	for (size_t count_row = 0; count_row < _vexNum; ++count_row) {
		for (size_t count_col = 0; count_col < _vexNum; ++count_col) {
			if (_adjMatrix[count_row][count_col] == UINT_MAX) {
				std::cout << "��" << " ";
			}
			else {
				std::cout << _adjMatrix[count_row][count_col] << " ";
			}
		}
		std::cout << std::endl;
	}
}

