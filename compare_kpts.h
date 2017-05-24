#ifndef COMPARE_KPTS
#define COMPARE_KPTS

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

void compare_kpts(keypoints1, keypoints2, &transform, &md_kpts, &RMSE_kpts);

#include "compare_kpts.inl"

#endif // COMPARE_KPTS
