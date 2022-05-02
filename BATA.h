/*
 * @Author: Diantao Tu
 * @Date: 2022-04-30 16:17:01
 */
#ifndef _BATA_H_
#define _BATA_H_
#include "common.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

struct Config
{
    double delta = 1e-6;
    int init_iteration = 10;
    int inner_iteration = 10;
    int outer_iteration = 10;
    double robust_threshold = 0.1;
};

/**
 * @description: Run BATA algorithm
 * @param pairs image pairs
 * @param relative_pose relative pose for each image pair
 * @param config configuration
 * @return global pose
 */
eigen_vector<Eigen::Vector3d> BATA(const vector<pair<int,int>>& pairs, const eigen_vector<Eigen::Vector3d>& relative_pose, const Config& config);

/**
 * @description: Load the index of image pairs from file. 2 x n matrix, each column is the index of an image pair
 * @param filename file path
 * @return image index in each pair
 */
vector<pair<int, int>> LoadTijIndex(string filename);

/**
 * @description: Load relative pose form file. 3 x n matrix, each column is the relative pose of an image pair
 * @param filename file path
 * @return relative pose of each pair
 */
eigen_vector<Eigen::Vector3d> LoadTij(string filename);



#endif