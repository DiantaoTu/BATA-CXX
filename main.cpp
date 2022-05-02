/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 16:18:24
 */
#include <vector>
#include <iostream>
#include <string>
#include <vector>

#include "BATA.h"

using namespace std;


int main()
{
    vector<pair<int, int>> pairs = LoadTijIndex("tij_index.txt");
    eigen_vector<Eigen::Vector3d> relative_pose = LoadTij("tij_observe.txt"); 
    Config config;
    eigen_vector<Eigen::Vector3d> global_pose = BATA(pairs, relative_pose, config);
    ofstream f("my_BATA.txt");
    for(const Eigen::Vector3d& p : global_pose)
        f << p.x() << " " << p.y() << " " << p.z() << endl;
    f.close();
    return 0;
}