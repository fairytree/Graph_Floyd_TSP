#pragma once

#include<iostream>
#include <cstdlib>
#include<vector>
#include<string>
#include "../../wallpaper/wallpaper.h"
#include "../../AreaDivision/AreaDivision.h"
#include "../clean/CleanRobot.h"
//#include "../../mapdivision/MapDivision.h"
//#include "../../mapdivision/MapDivisionDialog.h"

using namespace std;

typedef struct
{
    double X;
    double Y;
    double Angle;
}PoseInfo;
//预设得到门信息
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

//下一步可考虑把门也作为节点，做成有向图

class LinkRegion {
private:
    int RoomNum;   //图的顶点个数
    int DoorNum;     //图的边数
    int** arc;   //邻接矩阵，任意两个门之间的初始连接信息，默认任何两个相邻房间的距离为1，不相邻房间距离为无穷大
    int** dis;   //记录各个顶点最短路径的信息， 记录任意两个房间之间最少有几扇门
    int** path;  //记录各个最短路径的信息， 记录任意两个房间之间最短路径的房间的编号
    double** dispath;             //记录各个顶点最短门间距的信息， 记录任意两个房间之间的最短距离（距离为两扇门之间的直线距离，若只通过一扇门，则距离为0）
    std::vector<PoseInfo>** ARC;  //新临近矩阵，ARC记录任意两个房间之间的门的坐标信息， Pose记录门的坐标，暂时没有特定的用处，和DIS一样
    std::vector<PoseInfo>** DIS;  //新最短路径矩阵， 记录任意两个房间之间的最少通过的门的坐标信息
public:
    //构造函数
    LinkRegion(int RoomNum, int DoorNum);
    //析构函数
    ~LinkRegion();
    // 判断我们每次输入的的边的信息是否合法
    //顶点从1开始编号
    //bool check_edge_value(int start, int end, int weight);
    //Angel type transformation
    double Degree2Rad(double Angle_Degree);
    double Rad2Degree(double Angle_Rad);
    //门信息转换，提取有效门
    std::vector<DoorInfo_Link> DoorInfoTrans(std::vector<MapDivision::DoorInfo> Doorsinfo_original);
    //姿态反转
    PoseInfo AntiPose(PoseInfo pose);
    //创建图
    void createGraph(vector<DoorInfo_Link>);
    //门路径更新
    void update_DIS_path(vector<PoseInfo> Dispath1, vector<PoseInfo> Dispath2, vector<PoseInfo>* update);
    //门路径结合
    vector<PoseInfo> CombinePath(vector<PoseInfo> Dispath1, vector<PoseInfo> Dispath2);
    //门间距计算
    double CalPathDis(vector<PoseInfo> Dispath);
    //打印邻接矩阵
    void print();
    //打印最终邻接矩阵
    void print_DisAfter();
    //打印最终路径矩阵
    void print_PathAfter();
    //求最短路径
    void Floyd();
    //打印最短路径
    void print_path();
    //将区域排序 根据代价矩阵将区域重新排序，选取最优路径方案
    //vector<int> SortRegion(int StartRoom_index, vector<bool> isWork);
    ////将区域点连接输出
    //vector<vector<S_PoseF_1>> LinkPath(vector<vector<WorkPoint>> RegionWorkPoints_Total, vector<int> SortResult);
    ////将提取的关键路径点插回到区域间，合并输出所有路径点集
    //vector<WorkPoint> PathPointOutput(vector<vector<WorkPoint>> RegionWorkPoints_Total, vector<vector<S_PoseF_1>> LinkPoints, vector<int> SortResult, double MinDisBetweenPs);
};



