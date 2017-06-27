#ifndef GET_LCP
#define GET_LCP

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

void get_LCP(pcl::KdTreeFLANN<pcl_point>::Ptr tree, pcl::PointCloud<pcl_point>::Ptr cloud_src_transformed, float thresh, float* LCP);

#include "get_LCP.inl"

#endif // GET_LCP
