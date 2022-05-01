/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 16:18:24
 */
#include "BATA.h"

using namespace std;

// 对稀疏矩阵进行拼接，把四个矩阵 ABCD 拼接为一个矩阵 [A B ; C D] ，也就是左上角为A，右上角为B，左下角为C，右下角为D
// 对ABCD的要求是：1. 矩阵必须都是 colMajor的 2. A和B行数相等，A和C列数相等
// 由于矩阵都是列优先的，所以只能按列拼接，也就是水平拼接，所以整体步骤为： 
// 1. 先把ABCD都转置得到At Bt Ct Dt 
// 2. 把 At Ct 进行水平拼接，得到 [At Ct]，把Bt Dt进行水平拼接，得到 [Bt Dt]
// 3. 把[At Ct] 转置，得到 [A; C] ，把[Bt Dt]转置得到 [B;D]
// 4. 把 [A; C]和[B;D]水平拼接得到 [A B ; C D]
Eigen::SparseMatrix<double> SpliceMatrix(const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B, 
                                        const Eigen::SparseMatrix<double>& C, const Eigen::SparseMatrix<double>& D)
{
    assert(!A.IsRowMajor && !B.IsRowMajor && !C.IsRowMajor && !D.IsRowMajor);
    Eigen::SparseMatrix<double> At = A.transpose();
    Eigen::SparseMatrix<double> Bt = B.transpose();
    Eigen::SparseMatrix<double> Ct = C.transpose();
    Eigen::SparseMatrix<double> Dt = D.transpose();
    Eigen::SparseMatrix<double> AtCt(At.rows(), At.cols() + Ct.cols());
    Eigen::SparseMatrix<double> BtDt(Bt.rows(), Bt.cols() + Dt.cols());
    AtCt.leftCols(At.cols()) = At;
    AtCt.rightCols(Ct.cols()) = Ct;
    BtDt.leftCols(Bt.cols()) = Bt;
    BtDt.rightCols(Dt.cols()) = Dt;
    Eigen::SparseMatrix<double> spliced(A.rows() + C.rows(), A.cols() + B.cols());
    spliced.leftCols(AtCt.rows()) = AtCt.transpose();
    spliced.rightCols(BtDt.rows()) = BtDt.transpose();
    return spliced;
}

Eigen::MatrixXd LUDRevised(const int max_iteration, const Eigen::MatrixXd& tij_all, const Eigen::MatrixXd& At0_full)
{
    Eigen::VectorXd tij_sq_sum = tij_all.array().square().colwise().sum();

}

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
    Eigen::VectorXd tij_sq_sum = tij_all.array().square().colwise().sum();
    
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
    Eigen::SparseMatrix<double> Aeq_transpose = Aeq.transpose();
    Eigen::SparseMatrix<double> beq(4,1);
    beq.insert(0,0) = num_relative_pose;
    // Initialization with LUDRevised
    Eigen::VectorXd Svec = Eigen::VectorXd::Random(num_relative_pose).array().abs();
    Svec *= num_relative_pose / Svec.sum();
    Eigen::MatrixXd S = Svec.replicate(3,1);
    Eigen::VectorXd W = Eigen::VectorXd::Ones(3 * num_relative_pose);
    W = W.array().sqrt();
    for(int iter = 0; iter < 5; iter ++)
    {
        Eigen::SparseMatrix<double> A(3 * num_relative_pose, 3 * num_relative_pose);
        A.setIdentity();
        A = A * W.asDiagonal() * At0_full;
        Eigen::SparseMatrix<double> At = A.adjoint();
        Eigen::MatrixXd B = W.array() * S.array() * tij_all.reshaped<Eigen::ColMajor>().array();
        Eigen::SparseMatrix<double> A_full(A.cols() + 4, A.cols() + 4);
        // A_full 可以分为四块，左上角是2 * A^T * A
        // 右上角是 Aeq.transpose()
        // 左下角是 Aeq
        // 右下角是 零矩阵
        A_full = SpliceMatrix(2 * At * A, Aeq_transpose, Aeq, Eigen::SparseMatrix<double>(4,4));

        Eigen::SparseMatrix<double> b = SpliceMatrix(2 * At * Eigen::SparseMatrix<double>(B.sparseView()), Eigen::SparseMatrix<double>(0,0), 
                                                    beq, Eigen::SparseMatrix<double>(0,0));

        // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.compute(A_full);
        if(solver.info() != Eigen::Success)
        {
            cout << "decomposition failed" << endl;
        }
        Eigen::VectorXd X;
        X = solver.solve(b);
        if(solver.info() != Eigen::Success)
        {
            cout << "solving failed" << endl;
        }

        Eigen::VectorXd t = X.head(3 * num_camera);
        Eigen::MatrixXd Aij = (At0_full * t).reshaped(3, num_relative_pose);

        Svec = (Aij.array() * tij_all.array()).colwise().sum();
        Svec = Svec.array() / tij_sq_sum.array();  

        S = Svec.replicate(1,3).reshaped<Eigen::RowMajor>();

        Eigen::MatrixXd tmp = At0_full * t;
        tmp = (tmp.array() - S.array() * tij_all.reshaped<Eigen::ColMajor>().array()).reshaped(3, num_relative_pose);
        tmp = tmp.array().square();
        Eigen::MatrixXd tmp_col_sum = tmp.colwise().sum();
        Eigen::MatrixXd Wvec = (tmp_col_sum.array() + 1e-6).pow(-0.5);

        W = Wvec.replicate(3,1).reshaped<Eigen::ColMajor>();
        W = W.array().sqrt();
    }
    return eigen_vector<Eigen::Vector3d>();
}