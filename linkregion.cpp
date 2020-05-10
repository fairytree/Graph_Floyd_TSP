#include"linkregion.h"
//���캯��
LinkRegion::LinkRegion(int RoomNum, int DoorNum) {
    //��ʼ���������ͱ���
    this->RoomNum = RoomNum;
    this->DoorNum = DoorNum;
    //Ϊ�ڽӾ��󿪱ٿռ�͸���ֵ
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
            //�ڽӾ����ʼ��Ϊ�����
            arc[i][k] = INT_MAX;
            dispath[i][k] = 1E8;
            ARC[i][k] = DoorInit;
        }
    }
}

//��������
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
    //cout << "����ÿ���ߵ������յ㣨�����Ŵ�1��ʼ���Լ������λ��" << endl;
    int start;
    int end;
    PoseInfo DoorPose;
    int count = 0;
    while (count != Doors.size())
    {
        //cin >> start >> end >> DoorPose;
        ////�����жϱߵ���Ϣ�Ƿ�Ϸ�
        //while (!this->check_edge_value(start, end, weight)) {
        //    cout << "����ıߵ���Ϣ���Ϸ�������������" << endl;
        //    cin >> start >> end >> weight;
        //}
        start = Doors[count].Income_Room_index;
        end = Doors[count].Into_Room_index;
        if (-1 != start && -1 != end)
        {
            PoseInfo PoseAnti = AntiPose(Doors[count].Pose);    //�������Pose
            //���ڽӾ����Ӧ�ϵĵ㸳ֵ
            arc[start][end] = 1;
            arc[end][start] = 1;   //����ͼ��������д���,����Ȩֵ��ͬ������Գ�
            dispath[start][end] = 0;
            dispath[end][start] = 0;   //����ͼ����Գ�
            //���ڽӾ����Ӧ�ϵĵ㸳ֵ
            ARC[start][end].clear();
            ARC[start][end].push_back(Doors[count].Pose); // ARC��¼��������������ŵ�������Ϣ�� Pose��¼�ŵ�����
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
            //�Ѿ���D��ʼ��Ϊ�ڽӾ����ֵ
            this->dis[row][col] = this->arc[row][col];
            //�Ѿ���D��ʼ��Ϊ�ڽӾ����ֵ
            this->DIS[row][col] = this->ARC[row][col];
            //����P�ĳ�ֵ��Ϊ�����ߵ��յ㶥����±�
            this->path[row][col] = col;
        }
    }

    //����ѭ�������ڼ���ÿ����Ե����·��
    int temp = 0;
    int select = 0;
    double selectdis = 0.0;
    for (temp = 0; temp < this->RoomNum; temp++) {
        for (row = 0; row < this->RoomNum; row++) {
            for (col = 0; col < this->RoomNum; col++) {
                //Ϊ�˷�ֹ�����������Ҫ����һ��selectֵ
                //select = (dis[row][temp] == INT_MAX || dis[temp][col] == INT_MAX) ? INT_MAX : (dis[row][temp] + dis[temp][col]);
                //if (this->dis[row][col] > select) {
                //    //�������ǵ�D����
                //    this->dis[row][col] = select;
                //    //�������ǵ�P����
                //    this->path[row][col] = this->path[row][temp];
                //}
                if (col != row)
                {
                    //select = DIS[row][temp].size() + DIS[temp][col].size();
                    //if (DIS[row][col].size() > select)
                    //{
                    //    //�������ǵ�D����
                    //    this->dis[row][col] = select;
                    //    //�������ǵ�DIS����
                    //    update_DIS_path(DIS[row][temp], DIS[temp][col], &DIS[row][col]);
                    //    //update_DIS_path(DIS[col][temp], DIS[temp][row], &DIS[col][row]);  �Ƿ���Ҫ���
                    //    //�������ǵ�P����
                    //    this->path[row][col] = this->path[row][temp];
                    //}

                    //�����ż������ΪȨֵ
                    if (DIS[row][col].size() == 1)
                    {
                        dispath[row][col] = 0;    //ֻ����һ����  �����Ϊ0
                    }
                    //select = DIS[row][temp].size() + DIS[temp][col].size();
                    if (DIS[row][temp].size() <= this->DoorNum && DIS[temp][col].size() <= this->DoorNum)   //��Ϊ����������
                    {
                        selectdis = CalPathDis(CombinePath(DIS[row][temp], DIS[temp][col]));
                        if (this->dispath[row][col] > selectdis)
                        {
                            //�������ǵ�D����
                            this->dis[row][col] = DIS[row][temp].size() + DIS[temp][col].size();
                            //�������ǵ�DIS����
                            update_DIS_path(DIS[row][temp], DIS[temp][col], &DIS[row][col]);
                            //�����ż�����
                            this->dispath[row][col] = selectdis;
                            //�������ǵ�P����
                            this->path[row][col] = this->path[row][temp];
                        }
                    }
                }
                else
                {
                    dispath[row][col] = 0;           //���辭���� ����Ϊ0
                }
            }
        }
    }
}

void LinkRegion::print()
{
    cout << "ͼ���ڽӾ���Ϊ��" << endl;
    int count_row = 0; //��ӡ�еı�ǩ
    int count_col = 0; //��ӡ�еı�ǩ
    //��ʼ��ӡ
    while (count_row != this->RoomNum) {
        count_col = 0;
        while (count_col != this->RoomNum) {
            if (arc[count_row][count_col] == INT_MAX)
                cout << "��" << " ";
            else
                cout << arc[count_row][count_col] << " ";
            ++count_col;
        }
        cout << endl;
        ++count_row;
    }
}

void LinkRegion::print_DisAfter() {
    cout << "ͼ���ڽӾ���Ϊ��" << endl;
    int count_row = 0; //��ӡ�еı�ǩ
    int count_col = 0; //��ӡ�еı�ǩ
    //��ʼ��ӡ
    count_row = 0;
    while (count_row != this->RoomNum) {
        count_col = 0;
        while (count_col != this->RoomNum) {
            if (dis[count_row][count_col] == INT_MAX)
                cout << "��" << " ";
            else
                cout << dis[count_row][count_col] << " ";
            ++count_col;
        }
        cout << endl;
        ++count_row;
    }
}

void LinkRegion::print_PathAfter() {
    cout << "ͼ��DisPath����Ϊ��" << endl;
    int count_row = 0; //��ӡ�еı�ǩ
    int count_col = 0; //��ӡ�еı�ǩ
    //��ʼ��ӡ
    while (count_row != this->RoomNum) {
        count_col = 0;
        while (count_col != this->RoomNum) {
            if (dispath[count_row][count_col] == INT_MAX)
                cout << "��" << " ";
            else
                cout << dispath[count_row][count_col] << " ";
            ++count_col;
        }
        cout << endl;
        ++count_row;
    }
}

void LinkRegion::print_path() {
    cout << "�������������·����" << endl;
    int row = 0;
    int col = 0;
    int temp = 0;
    for (row = 0; row < this->RoomNum; row++) {
        for (col = row + 1; col < this->RoomNum; col++) {
            cout << "Region" << to_string(row + 1) << "---" << "Region" << to_string(col + 1) << " DoorNum: "
                << this->dis[row][col] << " path: " << " Region" << to_string(row + 1);
            temp = path[row][col];
            //ѭ�����;����ÿ��·����
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
            //ѭ�����;����ÿ����Pose��
            for (int jj = 0; jj < DIS[row][col].size(); jj++)
            {
                cout << "-->" << "X:" << DIS[row][col][jj].X << "Y:" << DIS[row][col][jj].Y << "Angle:" << DIS[row][col][jj].Angle;
            }
            cout << "-->" << "Region" << to_string(col + 1) << endl;
        }
        cout << endl;
    }
}

////������������
////������ʼ�������䣬�Ƿ���ѡ��
////�������ʼ�������俪ʼ�����й����������Ӳ����ص���������
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
//    //StartRoom_index�ڴ����������У�beforeSort���У��е�λ��New_StartRoom_index
//    int New_StartRoom_index = find(beforeSort.begin(), beforeSort.end(), StartRoom_index) - beforeSort.begin();
//    if (New_StartRoom_index < beforeSort.size())         // beforeSort�к���StartRoom_index���䣬˵��StartRoom_index���ڹ������䣬�ɽ�����һ������
//    {
//        //����
//        //��ʼ��
//        double DoorDis = 200;            //�Ÿ�����ʼ����β����������ŵľ��룬mm
//        const int n = beforeSort.size();       //������������
//        const int m = 2 * n;                   //Ant Num
//        const int NC_max = 2 * m;		//����������
//        const double Alpha = 1;		    //������Ϣ����Ҫ�̶ȵĲ���
//        const double Beta = 6;		    //��������ʽ������Ҫ�̶ȵĲ���
//        const double Rho = 0.15;		//��Ϣ������ϵ��
//        const double Q = 200;		    //��Ϣ������ǿ��ϵ��
//        vector<double> ZeroD_n(n, 0.0);
//        vector<double> OneD_n(n, 1.0);
//        vector<int> ZeroI_n(n, 0);
//        vector<vector<double>> DisSort(n, ZeroD_n);  //�½��ھ�����������          n*n
//        vector<vector<double>> Eta(n, ZeroD_n);      //����ʽ���ӣ�ΪDisSort�ĵ��� n*n
//        vector<vector<double>> DeltaTau(n, ZeroD_n); //��ʾ����ʽ���ӵı仯��      n*n
//        vector<vector<double>> Tau(n, OneD_n);		 //·��������Ϣ�ص�Ũ��        n*n
//        vector<vector<int>> PathHistory(m, ZeroI_n); //���ɱ��洢ÿ���߹���·��  m*n
//        vector<int> Path_best;                       //�洢���ε��������·��
//        double Path_best_length;                     //�洢���ε������·������
//        //��ȡ����Ľ��ھ���
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
//        //������ʼ
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
//            random_shuffle(temp.begin(), temp.end());	//����temp������Ԫ�صĴ���
//            for (int i = 0; i < m; i++)
//            {
//                PathHistory[i][0] = temp[i];
//            }
//
//            //��������
//            for (int j = 1; j < n; j++)
//            {
//                for (int i = 0; i < m; i++)
//                {
//                    vector<int> J;			//��iֻ���ϴ����ʵĳ���
//                    vector<double> P;		//��iֻ���ϴ����ʵĳ��еĸ���
//
//                    double Psum = 0.0;		//����ֵ��
//                    double rate = 0.0;		//�����
//                    double choose = 0.0;	//���̶��㷨�ۼ�ֵ
//                    int to_visit = 99999;	//��һ��Ҫȥ�ĳ���
//                    double MinP = 2.0;      //��Сѡ�����
//
//                    for (int k = 0; k < n; k++)
//                    {
//                        if (find(PathHistory[i].begin(), PathHistory[i].begin() + j, k) == PathHistory[i].begin() + j)	//�ڵ�iֻ�����ѷ�����û���ҵ�k
//                        {
//                            J.push_back(k);				//J��ʼ��
//                            P.push_back(0.0);			//P��ʼ��
//                        }
//                    }
//
//                    for (int k = 0; k < P.size(); k++)	//����ȥ��һ�����еĸ���
//                    {
//                        P[k] = pow(Tau[PathHistory[i][j - 1]][J[k]], Alpha) * pow(Eta[PathHistory[i][j - 1]][J[k]], Beta);
//                        if (P[k] < MinP)
//                            MinP = P[k];
//                        Psum += P[k];
//                    }
//                    rate = (rand() / (double)RAND_MAX) * ((Psum - MinP) - 0.0) + 0.0;    //ʹ�����̶��㷨����ѡ��һ��Ҫȥ�ĳ���
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
//            //���Ĳ�����¼���ε����������ߵ�·������
//            vector<double> Length(m, 0.0);//����ÿֻ�����ߵ�·��
//            double min_value = 1E12;	//�����󱾴������������߾�����Сֵ����ʱ����
//            int min_index = 0;			//��¼���������������߾�����Сֵ���±�
//            for (int i = 0; i < m; i++)
//            {
//                for (int j = 0; j < n - 1; j++)
//                {
//                    Length[i] += DisSort[PathHistory[i][j]][PathHistory[i][j + 1]];
//                }
//                Length[i] += DisSort[PathHistory[i][n - 1]][PathHistory[i][0]];
//                //�����ϴ�����·������һ�У����ϱ��ε������������̫��
//                if (0 == i && NC >= 2 && Length[0] > Path_best_length)
//                {
//                    Length[0] = Path_best_length;
//                    //memcpy(Tabu[0], Pathbest, n);
//                    //memcpy(Tabu[0], R_best[NC - 1], sizeof(R_best[NC - 1]));      //��R_best[NC - 1]��ַ��ʼn�����ȵ�Ԫ��ָ��ת�Ƶ�Tabu[0],�൱�ڽ�R_best[NC - 1]���Ƶ�Tabu[0]��ȥ
//                    PathHistory.erase(PathHistory.begin());
//                    PathHistory.insert(PathHistory.begin(), Path_best);
//                }
//                //����·��ɸѡ
//                if (Length[i] < min_value)
//                {
//                    min_value = Length[i];
//                    min_index = i;
//                }
//                //��Ϣ������
//                for (int j = 0; j < n - 1; j++)
//                {
//                    DeltaTau[PathHistory[i][j]][PathHistory[i][j + 1]] += Q / Length[i];	//�˴�ѭ��������·���ϵ���Ϣ������
//                }
//                DeltaTau[PathHistory[i][n - 1]][PathHistory[i][0]] += Q / Length[i];
//            }
//
//            //��¼���ε�����̵�·������
//            Path_best_length = min_value;						//ÿ����·������̳���
//            //memcpy(R_best[NC], Tabu[min_index], sizeof(Tabu[0]));
//            //memcpy(Pathbest, Tabu[min_index], n);
//            Path_best = PathHistory[min_index];
//
//            //cout << NC << ": L_best is " << Path_best_length << endl;	//��ӡ����������Ϣ
//
//            NC++;	//��������
//
//            //���岽��������Ϣ��
//            for (int i = 0; i < n; i++)
//            {
//                for (int j = 0; j < n; j++)
//                {
//                    Tau[i][j] = (1 - Rho) * Tau[i][j] + DeltaTau[i][j];	//������Ϣ�ػӷ������º����Ϣ��
//                }
//            }
//
//            //���ɱ�����
//            PathHistory.clear();
//            for (int i = 0; i < m; i++)
//            {
//                PathHistory.push_back(ZeroI_n);
//            }
//        }
//
//        //���������������
//        //New_StartRoom_index�������������У�Path_best���У��е�λ��StartInex
//        int StartInex = find(Path_best.begin(), Path_best.end(), New_StartRoom_index) - Path_best.begin();
//        for (int CountNum = 0; CountNum <= n; CountNum++)
//        {
//            int index = (CountNum + StartInex) < n ? (CountNum + StartInex) : (CountNum + StartInex - n);
//            returndata.push_back(beforeSort[Path_best[index]]);
//        }
//    }
//    else   //StartRoom_index���������������У��޷���������
//    {
//        returndata.clear();
//    }
//    return returndata;
//}
//
////���룺��������Ĺ����㼯�� ��������滮 ��ʱ�ɲ��� ��Ĭ������0 1 2 3 ����������
////������������ÿ�������������A*�滮�ĵ�λ��X Y R������
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
//        //��ʱ�ռ��ʼ��
//        returndata_Single.clear();
//        //ȡ����������Ŀ������
//        regionnum_tempt_income = SortResult[iii];
//        temptRegion_income = RegionWorkPoints_Total[regionnum_tempt_income];
//        regionnum_tempt_into = SortResult[iii + 1];
//        temptRegion_into = RegionWorkPoints_Total[regionnum_tempt_into];
//        //ȡ��ʼ�㣨�����������һ�������㣩
//        it = RegionWorkPoints_Total[regionnum_tempt_income].end() - 1;
//        temptPoint.X = (*it).WorkPointX;
//        temptPoint.Y = (*it).WorkPointY;
//        temptPoint.R = (*it).WorkPointR;
//        temptPoint.isDoor = false;
//        returndata_Single.push_back(temptPoint);
//        //ѭ����������;����λ��
//        int row = SortResult[iii];                   //����������  Ӧ����SortResult[iii]
//        int col = SortResult[iii + 1];   //Ŀ��������  Ӧ����SortResult[iii + 1]
//        for (int jj = 0; jj < DIS[row][col].size(); jj++)
//        {
//            temptPoint.X = DIS[row][col][jj].X;
//            temptPoint.Y = DIS[row][col][jj].Y;
//            temptPoint.R = DIS[row][col][jj].Angle;
//            temptPoint.isDoor = true;
//            returndata_Single.push_back(temptPoint);
//        }
//        //ȡ���ι滮·���յ㣨��������ĵ�һ�������㣩
//        it = RegionWorkPoints_Total[regionnum_tempt_into].begin();
//        temptPoint.X = (*it).WorkPointX;
//        temptPoint.Y = (*it).WorkPointY;
//        temptPoint.R = (*it).WorkPointR;
//        temptPoint.isDoor = false;
//        returndata_Single.push_back(temptPoint);
//        //���δ���
//        returndata.push_back(returndata_Single);
//    }
//    return returndata;
//}
//
////���룺��������Ĺ����㼯�� ÿ��������侭����A*�滮��ɢ·������ȡ�ؼ���λ��ȥ����λ��Ĺ��ɵ�λ��X Y R������LinkPoints
////������ϲ������й�����λ�ļ���
//vector<WorkPoint> LinkRegion::PathPointOutput(vector<vector<WorkPoint>> RegionWorkPoints_Total, vector<vector<S_PoseF_1>> LinkPoints, vector<int> SortResult, double MinDisBetweenPs)
//{
//    vector<WorkPoint> returndata;
//    WorkPoint tempt_point;
//    int regionnum_tempt = -1;
//    for (int iii = 0; iii < (SortResult.size() - 1); iii++)
//    {
//        //Ŀ��������
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
//        //���δ���������·����
//        for (int jjj = 0; jjj < RegionWorkPoints_Total[regionnum_tempt].size(); jjj++)
//        {
//            returndata.push_back(RegionWorkPoints_Total[regionnum_tempt][jjj]);
//        }
//        //���δ���LinkPoints·����
//        tempt_point.isWayPoint = 1;          //��λΪ1����ʾ����ͨ����������ҵ
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
