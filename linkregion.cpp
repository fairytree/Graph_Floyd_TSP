#include"linkregion.h"
//构造函数
LinkRegion::LinkRegion(int RoomNum, int DoorNum) {
    //初始化顶点数和边数
    this->RoomNum = RoomNum;
    this->DoorNum = DoorNum;
    //为邻接矩阵开辟空间和赋初值
    arc = new int* [this->RoomNum];
    dis = new int* [this->RoomNum];
    path = new int* [this->RoomNum];
    dispath = new double* [this->RoomNum];
    ARC = new vector<PoseInfo> * [this->RoomNum];
    DIS = new vector<PoseInfo> * [this->RoomNum];
    vector<PoseInfo> DoorInit(this->DoorNum + 1);
    for (int i = 0; i < this->RoomNum; i++) {
        arc[i] = new int[this->RoomNum];
        dis[i] = new int[this->RoomNum];
        path[i] = new int[this->RoomNum];
        dispath[i] = new double[this->RoomNum];
        ARC[i] = new vector<PoseInfo>[this->RoomNum];
        DIS[i] = new vector<PoseInfo>[this->RoomNum];
        for (int k = 0; k < this->RoomNum; k++) {
            //邻接矩阵初始化为无穷大
            arc[i][k] = INT_MAX;
            dispath[i][k] = 1E8;
            ARC[i][k] = DoorInit;
        }
    }
}

//析构函数
LinkRegion::~LinkRegion() {

    for (int i = 0; i < this->RoomNum; i++) {
        delete this->arc[i];
        delete this->dis[i];
        delete this->path[i];
        delete this->dispath[i];
        delete this->ARC[i];
        delete this->DIS[i];
    }
    delete dis;
    delete arc;
    delete path;
    delete dispath;
    delete ARC;
    delete DIS;
}

double LinkRegion::Degree2Rad(double Angle_Degree)
{
    return (Angle_Degree / 180 * 3.1415926535898);
}

double LinkRegion::Rad2Degree(double Angle_Rad)
{
    return (Angle_Rad * 180 / 3.1415926535898);
}

std::vector<DoorInfo_Link> LinkRegion::DoorInfoTrans(std::vector<MapDivision::DoorInfo> Doorsinfo_original)
{
    std::vector<DoorInfo_Link> returndata;
    DoorInfo_Link temptdoor;
    for (int i = 0; i < Doorsinfo_original.size(); i++)
    {
        if (true == Doorsinfo_original[i].isRealDoor && -1 != Doorsinfo_original[i].LinkRegion && -1 != Doorsinfo_original[i].LinkRegion_1)
        {
            temptdoor.Income_Room_index = Doorsinfo_original[i].LinkRegion;
            temptdoor.Into_Room_index = Doorsinfo_original[i].LinkRegion_1;
            //Unit transform to mm, mm, rad
            temptdoor.Pose.X = Doorsinfo_original[i].worldCentrePoint.X * 1000;
            temptdoor.Pose.Y = Doorsinfo_original[i].worldCentrePoint.Y * 1000;
            temptdoor.Pose.Angle = Degree2Rad(Doorsinfo_original[i].LinkOutAngle);
            returndata.push_back(temptdoor);
        }
    }
    return returndata;
}

PoseInfo LinkRegion::AntiPose(PoseInfo pose)
{
    PoseInfo returnpose;
    returnpose.X = pose.X;
    returnpose.Y = pose.Y;
    returnpose.Angle = atan2(sin(pose.Angle + 3.1415926535898), cos(pose.Angle + 3.1415926535898));
    return returnpose;
}

void LinkRegion::createGraph(vector<DoorInfo_Link> Doors)
{
    //cout << "输入每条边的起点和终点（顶点编号从1开始）以及其过门位姿" << endl;
    int start;
    int end;
    PoseInfo DoorPose;
    int count = 0;
    while (count != Doors.size())
    {
        //cin >> start >> end >> DoorPose;
        ////首先判断边的信息是否合法
        //while (!this->check_edge_value(start, end, weight)) {
        //    cout << "输入的边的信息不合法，请重新输入" << endl;
        //    cin >> start >> end >> weight;
        //}
        start = Doors[count].Income_Room_index;
        end = Doors[count].Into_Room_index;
        if (-1 != start && -1 != end)
        {
            PoseInfo PoseAnti = AntiPose(Doors[count].Pose);    //反向过门Pose
            //对邻接矩阵对应上的点赋值
            arc[start][end] = 1;
            arc[end][start] = 1;   //无向图添加上这行代码,反向权值相同，矩阵对称
            dispath[start][end] = 0;
            dispath[end][start] = 0;   //无向图矩阵对称
            //对邻接矩阵对应上的点赋值
            ARC[start][end].clear();
            ARC[start][end].push_back(Doors[count].Pose); // ARC记录两个相连房间的门的坐标信息， Pose记录门的坐标
            ARC[end][start].clear();
            ARC[end][start].push_back(PoseAnti);
        }
        ++count;
    }
}

void LinkRegion::update_DIS_path(vector<PoseInfo> Dispath1, vector<PoseInfo> Dispath2, vector<PoseInfo>* update)
{
    update->clear();
    for (int ii = 0; ii < Dispath1.size(); ii++)
    {
        update->push_back(Dispath1[ii]);
    }
    for (int ii = 0; ii < Dispath2.size(); ii++)
    {
        update->push_back(Dispath2[ii]);
    }
}

vector<PoseInfo> LinkRegion::CombinePath(vector<PoseInfo> Dispath1, vector<PoseInfo> Dispath2)
{
    vector<PoseInfo> returndata;
    for (int ii = 0; ii < Dispath1.size(); ii++)
    {
        returndata.push_back(Dispath1[ii]);
    }
    for (int ii = 0; ii < Dispath2.size(); ii++)
    {
        returndata.push_back(Dispath2[ii]);
    }
    return returndata;
}

double LinkRegion::CalPathDis(vector<PoseInfo> Dispath)
{
    double returndata = 0;
    /*if (Dispath.size() == 1)
    {
        returndata = 0;
    }
    else
    {*/
    for (int ii = 0; ii < (Dispath.size() - 1); ii++)
    {
        returndata += hypot((Dispath[ii + 1].Y - (Dispath[ii].Y)), (Dispath[ii + 1].X - Dispath[ii].X));
    }
    //}
    return returndata;
}

void LinkRegion::Floyd()
{
    int row = 0;
    int col = 0;
    for (row = 0; row < this->RoomNum; row++) {
        for (col = 0; col < this->RoomNum; col++) {
            //把矩阵D初始化为邻接矩阵的值
            this->dis[row][col] = this->arc[row][col];
            //把矩阵D初始化为邻接矩阵的值
            this->DIS[row][col] = this->ARC[row][col];
            //矩阵P的初值则为各个边的终点顶点的下标
            this->path[row][col] = col;
        }
    }

    //三重循环，用于计算每个点对的最短路径
    int temp = 0;
    int select = 0;
    double selectdis = 0.0;
    for (temp = 0; temp < this->RoomNum; temp++) {
        for (row = 0; row < this->RoomNum; row++) {
            for (col = 0; col < this->RoomNum; col++) {
                //为了防止溢出，所以需要引入一个select值
                //select = (dis[row][temp] == INT_MAX || dis[temp][col] == INT_MAX) ? INT_MAX : (dis[row][temp] + dis[temp][col]);
                //if (this->dis[row][col] > select) {
                //    //更新我们的D矩阵
                //    this->dis[row][col] = select;
                //    //更新我们的P矩阵
                //    this->path[row][col] = this->path[row][temp];
                //}
                if (col != row)
                {
                    //select = DIS[row][temp].size() + DIS[temp][col].size();
                    //if (DIS[row][col].size() > select)
                    //{
                    //    //更新我们的D矩阵
                    //    this->dis[row][col] = select;
                    //    //更新我们的DIS矩阵
                    //    update_DIS_path(DIS[row][temp], DIS[temp][col], &DIS[row][col]);
                    //    //update_DIS_path(DIS[col][temp], DIS[temp][row], &DIS[col][row]);  是否需要添加
                    //    //更新我们的P矩阵
                    //    this->path[row][col] = this->path[row][temp];
                    //}

                    //新添门间距离作为权值
                    if (DIS[row][col].size() == 1)
                    {
                        dispath[row][col] = 0;    //只经过一道门  距离记为0
                    }
                    //select = DIS[row][temp].size() + DIS[temp][col].size();
                    if (DIS[row][temp].size() <= this->DoorNum && DIS[temp][col].size() <= this->DoorNum)   //均为已连接区域
                    {
                        selectdis = CalPathDis(CombinePath(DIS[row][temp], DIS[temp][col]));
                        if (this->dispath[row][col] > selectdis)
                        {
                            //更新我们的D矩阵
                            this->dis[row][col] = DIS[row][temp].size() + DIS[temp][col].size();
                            //更新我们的DIS矩阵
                            update_DIS_path(DIS[row][temp], DIS[temp][col], &DIS[row][col]);
                            //更新门间距矩阵
                            this->dispath[row][col] = selectdis;
                            //更新我们的P矩阵
                            this->path[row][col] = this->path[row][temp];
                        }
                    }
                }
                else
                {
                    dispath[row][col] = 0;           //无需经过门 距离为0
                }
            }
        }
    }
}

void LinkRegion::print()
{
    cout << "图的邻接矩阵为：" << endl;
    int count_row = 0; //打印行的标签
    int count_col = 0; //打印列的标签
    //开始打印
    while (count_row != this->RoomNum) {
        count_col = 0;
        while (count_col != this->RoomNum) {
            if (arc[count_row][count_col] == INT_MAX)
                cout << "∞" << " ";
            else
                cout << arc[count_row][count_col] << " ";
            ++count_col;
        }
        cout << endl;
        ++count_row;
    }
}

void LinkRegion::print_DisAfter() {
    cout << "图的邻接矩阵为：" << endl;
    int count_row = 0; //打印行的标签
    int count_col = 0; //打印列的标签
    //开始打印
    count_row = 0;
    while (count_row != this->RoomNum) {
        count_col = 0;
        while (count_col != this->RoomNum) {
            if (dis[count_row][count_col] == INT_MAX)
                cout << "∞" << " ";
            else
                cout << dis[count_row][count_col] << " ";
            ++count_col;
        }
        cout << endl;
        ++count_row;
    }
}

void LinkRegion::print_PathAfter() {
    cout << "图的DisPath矩阵为：" << endl;
    int count_row = 0; //打印行的标签
    int count_col = 0; //打印列的标签
    //开始打印
    while (count_row != this->RoomNum) {
        count_col = 0;
        while (count_col != this->RoomNum) {
            if (dispath[count_row][count_col] == INT_MAX)
                cout << "∞" << " ";
            else
                cout << dispath[count_row][count_col] << " ";
            ++count_col;
        }
        cout << endl;
        ++count_row;
    }
}

void LinkRegion::print_path() {
    cout << "各个区域间的最短路径：" << endl;
    int row = 0;
    int col = 0;
    int temp = 0;
    for (row = 0; row < this->RoomNum; row++) {
        for (col = row + 1; col < this->RoomNum; col++) {
            cout << "Region" << to_string(row + 1) << "---" << "Region" << to_string(col + 1) << " DoorNum: "
                << this->dis[row][col] << " path: " << " Region" << to_string(row + 1);
            temp = path[row][col];
            //循环输出途径的每条路径。
            while (temp != col) {
                cout << "-->" << "Region" << to_string(temp + 1);
                temp = path[temp][col];
            }
            cout << "-->" << "Region" << to_string(col + 1) << endl;
        }
        cout << endl;
    }

    row = 0;
    col = 0;
    for (row = 0; row < this->RoomNum; row++) {
        for (col = row + 1; col < this->RoomNum; col++) {
            cout << "Region" << to_string(row + 1) << "---" << "Region" << to_string(col + 1) << " DoorPathDis: "
                << this->dispath[row][col] << " path: " << " Region" << to_string(row + 1);
            //循环输出途径的每个门Pose。
            for (int jj = 0; jj < DIS[row][col].size(); jj++)
            {
                cout << "-->" << "X:" << DIS[row][col][jj].X << "Y:" << DIS[row][col][jj].Y << "Angle:" << DIS[row][col][jj].Angle;
            }
            cout << "-->" << "Region" << to_string(col + 1) << endl;
        }
        cout << endl;
    }
}

////工作区域排序
////输入起始工作房间，是否工作选择
////输出从起始工作房间开始将所有工作房间连接并返回的最优排序
//vector<int> LinkRegion::SortRegion(int StartRoom_index, vector<bool> isWork)
//{
//    vector<int> returndata;
//    vector<int> beforeSort;
//    if (isWork.size() == this->RoomNum)
//    {
//        for (int RoomCount = 0; RoomCount < this->RoomNum; RoomCount++)
//        {
//            if (isWork[RoomCount] == true)
//            {
//                beforeSort.push_back(RoomCount);
//            }
//        }
//    }
//    //StartRoom_index在待排序工作序列（beforeSort序列）中的位置New_StartRoom_index
//    int New_StartRoom_index = find(beforeSort.begin(), beforeSort.end(), StartRoom_index) - beforeSort.begin();
//    if (New_StartRoom_index < beforeSort.size())         // beforeSort中含有StartRoom_index房间，说明StartRoom_index属于工作房间，可进行下一步排序
//    {
//        //排序
//        //初始化
//        double DoorDis = 200;            //门附近起始、结尾工作点距离门的距离，mm
//        const int n = beforeSort.size();       //工作区域数量
//        const int m = 2 * n;                   //Ant Num
//        const int NC_max = 2 * m;		//最大迭代次数
//        const double Alpha = 1;		    //表征信息素重要程度的参数
//        const double Beta = 6;		    //表征启发式因子重要程度的参数
//        const double Rho = 0.15;		//信息素蒸发系数
//        const double Q = 200;		    //信息素增加强度系数
//        vector<double> ZeroD_n(n, 0.0);
//        vector<double> OneD_n(n, 1.0);
//        vector<int> ZeroI_n(n, 0);
//        vector<vector<double>> DisSort(n, ZeroD_n);  //新紧邻矩阵用于排序          n*n
//        vector<vector<double>> Eta(n, ZeroD_n);      //启发式因子，为DisSort的倒数 n*n
//        vector<vector<double>> DeltaTau(n, ZeroD_n); //表示启发式因子的变化量      n*n
//        vector<vector<double>> Tau(n, OneD_n);		 //路径上面信息素的浓度        n*n
//        vector<vector<int>> PathHistory(m, ZeroI_n); //禁忌表，存储每个走过的路径  m*n
//        vector<int> Path_best;                       //存储本次迭代的最佳路线
//        double Path_best_length;                     //存储本次迭代最短路径长度
//        //抽取所需的紧邻矩阵
//        for (int i = 0; i < n; i++)
//        {
//            for (int k = i + 1; k < n; k++)
//            {
//                DisSort[i][k] = dispath[beforeSort[i]][beforeSort[k]] + DoorDis * 2.0;
//                DisSort[k][i] = DisSort[i][k];
//                Eta[i][k] = 1.0 / DisSort[i][k];
//                Eta[k][i] = Eta[i][k];
//            }
//        }
//        //迭代开始
//        srand((unsigned)time(NULL));
//        int NC = 0;
//        while (NC < NC_max)
//        {
//            vector<int> temp;
//            for (int i = 0; i < ceil((double)m / (double)n); i++)
//            {
//                for (int j = 0; j < n; j++)
//                    temp.push_back(j);
//            }
//            random_shuffle(temp.begin(), temp.end());	//打乱temp数组中元素的次序
//            for (int i = 0; i < m; i++)
//            {
//                PathHistory[i][0] = temp[i];
//            }
//
//            //本轮周游
//            for (int j = 1; j < n; j++)
//            {
//                for (int i = 0; i < m; i++)
//                {
//                    vector<int> J;			//第i只蚂蚁待访问的城市
//                    vector<double> P;		//第i只蚂蚁待访问的城市的概率
//
//                    double Psum = 0.0;		//概率值和
//                    double rate = 0.0;		//随机数
//                    double choose = 0.0;	//轮盘赌算法累加值
//                    int to_visit = 99999;	//下一个要去的城市
//                    double MinP = 2.0;      //最小选择概率
//
//                    for (int k = 0; k < n; k++)
//                    {
//                        if (find(PathHistory[i].begin(), PathHistory[i].begin() + j, k) == PathHistory[i].begin() + j)	//在第i只蚂蚁已访问中没有找到k
//                        {
//                            J.push_back(k);				//J初始化
//                            P.push_back(0.0);			//P初始化
//                        }
//                    }
//
//                    for (int k = 0; k < P.size(); k++)	//计算去下一座城市的概率
//                    {
//                        P[k] = pow(Tau[PathHistory[i][j - 1]][J[k]], Alpha) * pow(Eta[PathHistory[i][j - 1]][J[k]], Beta);
//                        if (P[k] < MinP)
//                            MinP = P[k];
//                        Psum += P[k];
//                    }
//                    rate = (rand() / (double)RAND_MAX) * ((Psum - MinP) - 0.0) + 0.0;    //使用轮盘赌算法，挑选下一座要去的城市
//                    for (int k = 0; k < P.size(); k++)
//                    {
//                        choose += P[k];
//                        if (choose > rate)
//                        {
//                            to_visit = J[k];
//                            break;
//                        }
//                    }
//                    PathHistory[i][j] = to_visit;
//                }
//            }
//
//            //第四步：记录本次迭代蚂蚁行走的路线数据
//            vector<double> Length(m, 0.0);//本代每只蚂蚁走的路程
//            double min_value = 1E12;	//声明求本代所有蚂蚁行走距离最小值的临时变量
//            int min_index = 0;			//记录本代所有蚂蚁行走距离最小值的下标
//            for (int i = 0; i < m; i++)
//            {
//                for (int j = 0; j < n - 1; j++)
//                {
//                    Length[i] += DisSort[PathHistory[i][j]][PathHistory[i][j + 1]];
//                }
//                Length[i] += DisSort[PathHistory[i][n - 1]][PathHistory[i][0]];
//                //保留上次最优路线至第一行，保障本次迭代情况不至于太差
//                if (0 == i && NC >= 2 && Length[0] > Path_best_length)
//                {
//                    Length[0] = Path_best_length;
//                    //memcpy(Tabu[0], Pathbest, n);
//                    //memcpy(Tabu[0], R_best[NC - 1], sizeof(R_best[NC - 1]));      //将R_best[NC - 1]地址开始n个长度的元素指针转移到Tabu[0],相当于将R_best[NC - 1]复制到Tabu[0]中去
//                    PathHistory.erase(PathHistory.begin());
//                    PathHistory.insert(PathHistory.begin(), Path_best);
//                }
//                //最优路径筛选
//                if (Length[i] < min_value)
//                {
//                    min_value = Length[i];
//                    min_index = i;
//                }
//                //信息素增量
//                for (int j = 0; j < n - 1; j++)
//                {
//                    DeltaTau[PathHistory[i][j]][PathHistory[i][j + 1]] += Q / Length[i];	//此次循环在整个路径上的信息素增量
//                }
//                DeltaTau[PathHistory[i][n - 1]][PathHistory[i][0]] += Q / Length[i];
//            }
//
//            //记录本次迭代最短的路径数据
//            Path_best_length = min_value;						//每代中路径的最短长度
//            //memcpy(R_best[NC], Tabu[min_index], sizeof(Tabu[0]));
//            //memcpy(Pathbest, Tabu[min_index], n);
//            Path_best = PathHistory[min_index];
//
//            //cout << NC << ": L_best is " << Path_best_length << endl;	//打印各代距离信息
//
//            NC++;	//迭代继续
//
//            //第五步：更新信息素
//            for (int i = 0; i < n; i++)
//            {
//                for (int j = 0; j < n; j++)
//                {
//                    Tau[i][j] = (1 - Rho) * Tau[i][j] + DeltaTau[i][j];	//考虑信息素挥发，更新后的信息素
//                }
//            }
//
//            //禁忌表清零
//            PathHistory.clear();
//            for (int i = 0; i < m; i++)
//            {
//                PathHistory.push_back(ZeroI_n);
//            }
//        }
//
//        //输出工作区域排序
//        //New_StartRoom_index在已排序工作序列（Path_best序列）中的位置StartInex
//        int StartInex = find(Path_best.begin(), Path_best.end(), New_StartRoom_index) - Path_best.begin();
//        for (int CountNum = 0; CountNum <= n; CountNum++)
//        {
//            int index = (CountNum + StartInex) < n ? (CountNum + StartInex) : (CountNum + StartInex - n);
//            returndata.push_back(beforeSort[Path_best[index]]);
//        }
//    }
//    else   //StartRoom_index不在允许工作序列中，无法进行排序
//    {
//        returndata.clear();
//    }
//    return returndata;
//}
//
////输入：所有区域的工作点集合 区域排序规划 暂时可不做 做默认排序（0 1 2 3 。。。。）
////输出：依次输出每两个区域间用于A*规划的点位（X Y R）集合
//vector<vector<S_PoseF_1>> LinkRegion::LinkPath(vector<vector<WorkPoint>> RegionWorkPoints_Total, vector<int> SortResult)
//{
//
//    vector<vector<S_PoseF_1>> returndata;
//    vector<S_PoseF_1> returndata_Single;
//    vector<WorkPoint> temptRegion_income;
//    vector<WorkPoint> temptRegion_into;
//    S_PoseF_1 temptPoint;
//    vector<WorkPoint>::const_iterator it;
//    int regionnum_tempt_income = -1;
//    int regionnum_tempt_into = -1;
//    for (int iii = 0; iii < (SortResult.size() - 1); iii++)
//    {
//        //临时空间初始化
//        returndata_Single.clear();
//        //取出发区域与目标区域
//        regionnum_tempt_income = SortResult[iii];
//        temptRegion_income = RegionWorkPoints_Total[regionnum_tempt_income];
//        regionnum_tempt_into = SortResult[iii + 1];
//        temptRegion_into = RegionWorkPoints_Total[regionnum_tempt_into];
//        //取起始点（出发区域最后一个工作点）
//        it = RegionWorkPoints_Total[regionnum_tempt_income].end() - 1;
//        temptPoint.X = (*it).WorkPointX;
//        temptPoint.Y = (*it).WorkPointY;
//        temptPoint.R = (*it).WorkPointR;
//        temptPoint.isDoor = false;
//        returndata_Single.push_back(temptPoint);
//        //循环放入所有途径门位姿
//        int row = SortResult[iii];                   //出发区域编号  应该是SortResult[iii]
//        int col = SortResult[iii + 1];   //目标区域编号  应该是SortResult[iii + 1]
//        for (int jj = 0; jj < DIS[row][col].size(); jj++)
//        {
//            temptPoint.X = DIS[row][col][jj].X;
//            temptPoint.Y = DIS[row][col][jj].Y;
//            temptPoint.R = DIS[row][col][jj].Angle;
//            temptPoint.isDoor = true;
//            returndata_Single.push_back(temptPoint);
//        }
//        //取单段规划路径终点（到达区域的第一个工作点）
//        it = RegionWorkPoints_Total[regionnum_tempt_into].begin();
//        temptPoint.X = (*it).WorkPointX;
//        temptPoint.Y = (*it).WorkPointY;
//        temptPoint.R = (*it).WorkPointR;
//        temptPoint.isDoor = false;
//        returndata_Single.push_back(temptPoint);
//        //单段存入
//        returndata.push_back(returndata_Single);
//    }
//    return returndata;
//}
//
////输入：所有区域的工作点集合 每两个区域间经过了A*规划离散路径、提取关键点位、去除首位点的过渡点位（X Y R）集合LinkPoints
////输出：合并了所有工作点位的集合
//vector<WorkPoint> LinkRegion::PathPointOutput(vector<vector<WorkPoint>> RegionWorkPoints_Total, vector<vector<S_PoseF_1>> LinkPoints, vector<int> SortResult, double MinDisBetweenPs)
//{
//    vector<WorkPoint> returndata;
//    WorkPoint tempt_point;
//    int regionnum_tempt = -1;
//    for (int iii = 0; iii < (SortResult.size() - 1); iii++)
//    {
//        //目标区域编号
//        regionnum_tempt = SortResult[iii];
//        //Points need to check
//        WorkPoint CheckPoint1;
//        CheckPoint1.WorkPointX = RegionWorkPoints_Total[regionnum_tempt].back().WorkPointX;
//        CheckPoint1.WorkPointY = RegionWorkPoints_Total[regionnum_tempt].back().WorkPointY;
//        CheckPoint1.WorkPointR = RegionWorkPoints_Total[regionnum_tempt].back().WorkPointR;
//        WorkPoint CheckPoint2;
//        int hahahahaha = SortResult[iii + 1];
//        CheckPoint2.WorkPointX = RegionWorkPoints_Total[hahahahaha].front().WorkPointX;
//        CheckPoint2.WorkPointY = RegionWorkPoints_Total[hahahahaha].front().WorkPointY;
//        CheckPoint2.WorkPointR = RegionWorkPoints_Total[hahahahaha].front().WorkPointR;
//        //依次存入区域内路径点
//        for (int jjj = 0; jjj < RegionWorkPoints_Total[regionnum_tempt].size(); jjj++)
//        {
//            returndata.push_back(RegionWorkPoints_Total[regionnum_tempt][jjj]);
//        }
//        //依次存入LinkPoints路径点
//        tempt_point.isWayPoint = 1;          //此位为1，表示仅需通过，无需作业
//        tempt_point.SubRegionIndex = -1;
//        tempt_point.WorkLineIndex = -1;
//        tempt_point.WorkPointIndex = -1;
//        for (int jjj = 0; jjj < LinkPoints[iii].size(); jjj++)
//        {
//            int isPointNeeded = 1;
//            if (hypot((LinkPoints[iii][jjj].Y - CheckPoint1.WorkPointY), (LinkPoints[iii][jjj].X - CheckPoint1.WorkPointX)) < MinDisBetweenPs)
//            {
//                if (fabs(LinkPoints[iii][jjj].R - CheckPoint1.WorkPointR) > 0.01745)
//                {
//                    LinkPoints[iii][jjj].X = CheckPoint1.WorkPointX;
//                    LinkPoints[iii][jjj].Y = CheckPoint1.WorkPointY;
//                }
//                else
//                {
//                    isPointNeeded = 0;
//                }
//            }
//            if (hypot((LinkPoints[iii][jjj].Y - CheckPoint2.WorkPointY), (LinkPoints[iii][jjj].X - CheckPoint2.WorkPointX)) < MinDisBetweenPs)
//            {
//                if (fabs(LinkPoints[iii][jjj].R - CheckPoint2.WorkPointR) > 0.01745)
//                {
//                    LinkPoints[iii][jjj].X = CheckPoint2.WorkPointX;
//                    LinkPoints[iii][jjj].Y = CheckPoint2.WorkPointY;
//                }
//                else
//                {
//                    isPointNeeded = 0;
//                }
//            }
//            if (1 == isPointNeeded)
//            {
//                tempt_point.WorkPointX = LinkPoints[iii][jjj].X;
//                tempt_point.WorkPointY = LinkPoints[iii][jjj].Y;
//                tempt_point.WorkPointR = LinkPoints[iii][jjj].R;
//                returndata.push_back(tempt_point);
//            }
//        }
//    }
//    return returndata;
//}
