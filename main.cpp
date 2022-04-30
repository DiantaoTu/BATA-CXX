/*
 * @Author: your name
 * @Date: 2020-05-20 00:49:59
 * @LastEditTime: 2022-04-30 18:51:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /test/main.cpp
 */
#include <vector>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>


#include <glog/logging.h>
#include <omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <set>

#include <random>
#include <mutex>
#include "BATA.h"

using namespace std;


int main()
{
    vector<pair<int, int>> pairs = LoadTijIndex("tij_index.txt");
    eigen_vector<Eigen::Vector3d> relative_pose = LoadTij("tij_observe.txt"); 
    BATA(pairs, relative_pose);
    return 0;
}