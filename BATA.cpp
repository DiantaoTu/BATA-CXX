/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 16:18:24
 */
#include "BATA.h"

using namespace std;

vector<pair<int, int>> LoadTijIndex(string filename)
{
    vector<pair<int, int>> pairs;
    vector<int> idx1, idx2;
    ifstream f(filename);
    string line;
    getline(f, line);
    for(const string& s : SplitString(line, ' '))
        idx1.push_back(str2num<int>(s));
    getline(f, line);
    for(const string& s : SplitString(line, ' '))
        idx2.push_back(str2num<int>(s));
    assert(idx1.size() == idx2.size());
    for(size_t i = 0; i < idx1.size(); i++)
        pairs.push_back({idx1[i], idx2[i]});
    return pairs;
}

eigen_vector<Eigen::Vector3d> LoadTij(string filename)
{
    eigen_vector<Eigen::Vector3d> Tij;
    vector<double> tx, ty, tz;
    ifstream f(filename);
    string line;
    // 文件一共就三行，第一行是x，第二行是y，第三行是z
    getline(f, line);
    for(const string& s : SplitString(line, ' '))
        tx.push_back(str2num<double>(s));
    getline(f, line);
    for(const string& s : SplitString(line, ' '))
        ty.push_back(str2num<double>(s));
    getline(f, line);
    for(const string& s : SplitString(line, ' '))
        tz.push_back(str2num<double>(s));
    assert(tx.size() == ty.size() && ty.size() == tz.size());
    for(size_t i = 0; i < tx.size(); i++)
        Tij.push_back(Eigen::Vector3d(tx[i], ty[i], tz[i]));
    return Tij;
}

eigen_vector<Eigen::Vector3d> BATA(const vector<pair<int,int>>& pairs, const eigen_vector<Eigen::Vector3d>& relative_pose)
{
    assert(pairs.size() == relative_pose.size());
    const int num_relative_pose = relative_pose.size();
    Eigen::MatrixXd tij_all(3, num_relative_pose);
    int num_camera, max_camera_id = -1, min_camera_id = INT16_MAX;
    Eigen::MatrixXd tij_index(2, num_relative_pose);
    for(int i = 0; i < num_relative_pose; i++)
    {
        const pair<int,int>& p = pairs[i];
        max_camera_id = max(max_camera_id, max(p.first, p.second));
        min_camera_id = min(min_camera_id, min(p.first, p.second));
        tij_index(0, i) = p.first;
        tij_index(1, i) = p.second;
        tij_all.col(i) = relative_pose[i];
    }
    num_camera = max_camera_id - min_camera_id + 1;
    
    /*
    Eigen::MatrixXd Ri_T = Eigen::Matrix3d::Identity().replicate(num_relative_pose, 1);
    Eigen::MatrixXd Rj_T = Eigen::Matrix3d::Identity().replicate(num_relative_pose, 1);
    Eigen::MatrixXd index_ti_I = Eigen::VectorXd::LinSpaced(3 * num_relative_pose, 1, 3 * num_relative_pose).replicate(1,3);
    Eigen::MatrixXd index_ti_J_init = ((tij_index.row(0).array() - 1) * 3).matrix();
    Eigen::MatrixXd index_ti_J(3, 3 * num_relative_pose);
    index_ti_J << index_ti_J_init, (index_ti_J_init.array() + 1), (index_ti_J_init.array() +2),
                    index_ti_J_init, (index_ti_J_init.array() + 1), (index_ti_J_init.array() +2),
                    index_ti_J_init, (index_ti_J_init.array() + 1), (index_ti_J_init.array() +2);

    Eigen::MatrixXd index_tj_I = Eigen::VectorXd::LinSpaced(3 * num_relative_pose, 1, 3 * num_relative_pose).replicate(1,3);
    Eigen::MatrixXd index_tj_J_init = ((tij_index.row(1).array() - 1) * 3).matrix();
    Eigen::MatrixXd index_tj_J(3, 3 * num_relative_pose);
    index_tj_J << index_tj_J_init, (index_tj_J_init.array() + 1), (index_tj_J_init.array() +2),
                    index_tj_J_init, (index_tj_J_init.array() + 1), (index_tj_J_init.array() +2),
                    index_tj_J_init, (index_tj_J_init.array() + 1), (index_tj_J_init.array() +2);

    */
    // 设置矩阵A，是一个分块矩阵，共 num_relative_pose 行，num_camera 列，里面的每个分块是单位阵，负单位阵或者零矩阵
    // 对于第k对相对位姿，是关于图像i和j的，那么A矩阵的第k行的第i列是单位阵，第k行第j列是负单位阵，其他为零
    vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(num_relative_pose * 6);
    for(size_t i = 0; i < num_relative_pose; i++)
    {
        // 这里减去了min camera id是因为可能初始的索引不是从0开始的，所以要减去这个，让索引从零开始
        const int idx1 = pairs[i].first - min_camera_id;
        const int idx2 = pairs[i].second - min_camera_id;
        triplet_list.emplace_back(3 * i, 3 * idx1, 1);
        triplet_list.emplace_back(3 * i + 1, 3 * idx1 + 1, 1);
        triplet_list.emplace_back(3 * i + 2, 3 * idx1 + 2, 1);
        triplet_list.emplace_back(3 * i, 3 * idx2, -1);
        triplet_list.emplace_back(3 * i + 1, 3 * idx2 + 1, -1);
        triplet_list.emplace_back(3 * i + 2, 3 * idx2 + 2, -1);
    }
    Eigen::SparseMatrix<double> At0_full(3 * num_relative_pose, 3 * num_camera);
    At0_full.setFromTriplets(triplet_list.begin(), triplet_list.end());
    triplet_list.clear();
    Eigen::SparseMatrix<double> At0 = At0_full.rightCols(3*num_camera -3);  // 舍弃前三列，也就是第一个相机对应的矩阵
    Eigen::SparseMatrix<double> Aeq1(num_relative_pose, 3 * num_relative_pose);
    Aeq1.reserve(num_relative_pose * 3);
    // 这个插入是按照列增加的顺序进行的，而且Aeq1本身就是colMajor的，因此直接插入速度会比较快，就不需要使用三元组了
    for(size_t i = 0; i < num_relative_pose; i++)
    {
        Aeq1.insert(i, 3 * i) = relative_pose[i].x();
        Aeq1.insert(i, 3 * i + 1) = relative_pose[i].y();
        Aeq1.insert(i, 3 * i + 2) = relative_pose[i].z();
    }
    Aeq1 = Aeq1 * At0_full;
    // Aeq 每一列只有两个非零元素，其中Aeq的第一行是Aeq1的每一列之和，后三行是单位阵水平堆叠
    Eigen::SparseMatrix<double> Aeq(4, Aeq1.cols());
    Aeq.reserve(2 * Aeq1.cols());  
    for(size_t i = 0; i < Aeq.cols(); i++)
    {
        Aeq.insert(0, i) = Aeq1.col(i).sum();
        Aeq.insert(i % 3 + 1, i) = 1;
    }
    Eigen::Vector4d beq(num_relative_pose, 0, 0, 0);
    // Initialization with LUDRevised
    Eigen::VectorXd Svec = Eigen::VectorXd::Random(num_relative_pose).array().abs();
    cout << Svec << endl;
    Svec *= num_relative_pose / Svec.sum();
    Eigen::MatrixXd S = Svec.replicate(1,3).reshaped<Eigen::RowMajor>();
    Eigen::VectorXd W = Eigen::VectorXd::Ones(3 * num_relative_pose);
    
    for(int iter = 0; iter < 10; iter ++)
    {
        Eigen::SparseMatrix<double> A(3 * num_relative_pose, 3 * num_relative_pose);
        A.setIdentity();
        A = A * W.asDiagonal();
        Eigen::SparseMatrix<double> At = A.transpose();
        Eigen::VectorXd B = (W.array().sqrt()) * S.array() * tij_all.reshaped<Eigen::ColMajor>().array(); 
        Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> linear_solver;

    }
    return eigen_vector<Eigen::Vector3d>();
}