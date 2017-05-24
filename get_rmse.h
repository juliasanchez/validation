#ifndef GET_RMSE
#define GET_RMSE

#include <iostream>
#include <fstream>
#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

using namespace std;
typedef pcl::PointXYZ pcl_point;

void get_rmse(pcl::PointCloud<pcl_point> cloud_src, pcl::PointCloud<pcl_point> cloud_tgt, Eigen::Matrix4f* transform, float* RMSE);

#include "get_rmse.inl"

#endif // GET_MEAN_DISTANCE
