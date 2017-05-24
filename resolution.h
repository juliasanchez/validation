#ifndef RESOLUTION
#define RESOLUTION

#include <iostream>
#include <string>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZ pcl_point;

double resolution(pcl::PointCloud<pcl_point>::Ptr cloud_in, pcl::KdTreeFLANN<pcl_point>::Ptr tree);

#include "resolution.inl"

#endif // RESOLUTION
