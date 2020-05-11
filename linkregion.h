#pragma once

#include<iostream>
#include <cstdlib>
#include<vector>
#include<string>
#include "opencv.hpp"
//#include "../../wallpaper/wallpaper.h"
//#include "../../AreaDivision/AreaDivision.h"
//#include "../clean/CleanRobot.h"
//#include "../../mapdivision/MapDivision.h"
//#include "../../mapdivision/MapDivisionDialog.h"

using namespace std;

// newly added
typedef struct
{
    double X;
    double Y;
}mPoint;

// newly added
typedef struct
{
    int LinkRegion;
    int LinkRegion_1;
    double LinkEnterAngle;
    double LinkOutAngle;
    double LinkEnterAngle_1;
    double LinkOutAngle_1;
    double originAngle;
    bool isRealDoor;
    cv::Point tl;
    cv::Point br;
    cv::Point centrePoint;
    mPoint worldTL;
    mPoint worldBR;
    mPoint worldCentrePoint;
    void clear()
    {
        LinkRegion = -1;
        LinkRegion_1 = -1;
        LinkEnterAngle = -1;
        LinkOutAngle = -1;
        LinkEnterAngle_1 = -1;
        LinkOutAngle_1 = -1;
        originAngle = -1;
        centrePoint = { -1,-1 };
        tl = { -1,-1 };
        br = { -1,-1 };
        worldTL = { 0,0 };
        worldBR = { 0,0 };
        worldCentrePoint = { 0,0 };
        isRealDoor = false;
    }
}DoorInfo;

typedef struct
{
    double X;
    double Y;
    double Angle;
}PoseInfo;
//Ԥ��õ�����Ϣ
typedef struct
{
    int Income_Room_index;
    int Into_Room_index;
    PoseInfo Pose;
}DoorInfo_Link;

typedef struct
{
    vector<DoorInfo_Link> Door;
}DoorInfoAllMap;

//��һ���ɿ��ǰ���Ҳ��Ϊ�ڵ㣬��������ͼ

class LinkRegion {
private:
    int RoomNum;   //ͼ�Ķ������
    int DoorNum;     //ͼ�ı���
    int** arc;   //�ڽӾ�������������֮��ĳ�ʼ������Ϣ��Ĭ���κ��������ڷ���ľ���Ϊ1�������ڷ������Ϊ�����
    int** dis;   //��¼�����������·������Ϣ�� ��¼������������֮�������м�����
    int** path;  //��¼�������·������Ϣ�� ��¼������������֮�����·���ķ���ı��
    double** dispath;             //��¼������������ż�����Ϣ�� ��¼������������֮�����̾��루����Ϊ������֮���ֱ�߾��룬��ֻͨ��һ���ţ������Ϊ0��
    std::vector<PoseInfo>** ARC;  //���ٽ�����ARC��¼������������֮����ŵ�������Ϣ�� Pose��¼�ŵ����꣬��ʱû���ض����ô�����DISһ��
    std::vector<PoseInfo>** DIS;  //�����·������ ��¼������������֮�������ͨ�����ŵ�������Ϣ
public:
    //���캯��
    LinkRegion(int RoomNum, int DoorNum);
    //��������
    ~LinkRegion();
    // �ж�����ÿ������ĵıߵ���Ϣ�Ƿ�Ϸ�
    //�����1��ʼ���
    //bool check_edge_value(int start, int end, int weight);
    //Angel type transformation
    double Degree2Rad(double Angle_Degree);
    double Rad2Degree(double Angle_Rad);
    //����Ϣת������ȡ��Ч��
    std::vector<DoorInfo_Link> DoorInfoTrans(std::vector<MapDivision::DoorInfo> Doorsinfo_original);
    //��̬��ת
    PoseInfo AntiPose(PoseInfo pose);
    //����ͼ
    void createGraph(vector<DoorInfo_Link>);
    //��·������
    void update_DIS_path(vector<PoseInfo> Dispath1, vector<PoseInfo> Dispath2, vector<PoseInfo>* update);
    //��·�����
    vector<PoseInfo> CombinePath(vector<PoseInfo> Dispath1, vector<PoseInfo> Dispath2);
    //�ż�����
    double CalPathDis(vector<PoseInfo> Dispath);
    //��ӡ�ڽӾ���
    void print();
    //��ӡ�����ڽӾ���
    void print_DisAfter();
    //��ӡ����·������
    void print_PathAfter();
    //�����·��
    void Floyd();
    //��ӡ���·��
    void print_path();
    //���������� ���ݴ��۾���������������ѡȡ����·������
    //vector<int> SortRegion(int StartRoom_index, vector<bool> isWork);
    ////��������������
    //vector<vector<S_PoseF_1>> LinkPath(vector<vector<WorkPoint>> RegionWorkPoints_Total, vector<int> SortResult);
    ////����ȡ�Ĺؼ�·�����ص�����䣬�ϲ��������·���㼯
    //vector<WorkPoint> PathPointOutput(vector<vector<WorkPoint>> RegionWorkPoints_Total, vector<vector<S_PoseF_1>> LinkPoints, vector<int> SortResult, double MinDisBetweenPs);
};



