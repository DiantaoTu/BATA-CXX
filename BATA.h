/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 16:17:01
 */
#ifndef _BATA_H_
#define _BATA_H_
#include "common.h"
#include <iostream>
#include <fstream>
#include <Eigen/Sparse>

eigen_vector<Eigen::Vector3d> BATA(const vector<pair<int,int>>& pairs, const eigen_vector<Eigen::Vector3d>& relative_pose);

vector<pair<int, int>> LoadTijIndex(string filename);

eigen_vector<Eigen::Vector3d> LoadTij(string filename);



#endif