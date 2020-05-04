#pragma once

#include<iostream>
#include<string>
#include<vector>

class Graph
{
private:
	size_t _vexNum;    // ͼ�Ķ������ number of vertices  
	size_t _edgeNum;   // ͼ�ı��� number of edges
	size_t _graphType;   // ͼ�����ͣ�1��������ͼ��2��������ͼ, types of graph, directedgraph is 1, undirectedgraph is 2.
	std::vector<std::vector<double>> _adjMatrix;  //  �ڽӾ���, ��¼ͼ�ж���ͱߵĹ�ϵ, adjacency matrix

public:
	//���캯��
	Graph();

	//��������
	~Graph();

	//������������Ͷ�������ֵ�Ƿ���Ч
	bool checkVexnum(size_t vexnum, size_t edge);

	// �ж�ÿ������ĵıߵ���Ϣ�Ƿ�Ϸ�
	bool checkEdgeValue(size_t start, size_t end, double weight);

	// ���ض������
	size_t vexNum() const { return _vexNum; };

	// ���رߵ�����
	size_t edgeNum() const { return _edgeNum; };

	// �����ڽӾ���
	std::vector<std::vector<double>> adjMatrix() const { return _adjMatrix; };

	//��ӡ�ڽӾ���
	void print();
};

