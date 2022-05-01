/*
 * @Author: your name
 * @Date: 2020-05-20 00:49:59
 * @LastEditTime: 2022-05-01 20:49:23
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /test/main.cpp
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
    return 0;
}