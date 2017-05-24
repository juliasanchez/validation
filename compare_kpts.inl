void compare_kpts(pcl::PointCloud<pcl_point>::Ptr keypoints1, pcl::PointCloud<pcl_point>::Ptr keypoints2, Eigen::Matrix4f& transform, float* md, float* RMSE)
{
    //transform
    pcl::PointCloud<pcl_point> keypoints1_transformed;
    pcl::transformPointCloud (*keypoints1, *keypoints1, *transform);

    //compute md and RMSE

    *RMSE=0;
    *mean_distance=0;

    for (int k=0; k<keypoints1.size(); k++)
    {
        float dist = (keypoints1->points[k].x-keypoints2->points[k].x)*(keypoints1->points[k].x-keypoints2->points[k].x)+(keypoints1->points[k].y-keypoints2->points[k].y)*(keypoints1->points[k].y-keypoints2->points[k].y)+(keypoints1->points[k].z-keypoints2->points[k].z)*(keypoints1->points[k].z-keypoints2->points[k].z);
        *mean_distance=*mean_distance+sqrt(dist);
        *RMSE=*RMSE+dist;
    }

    *mean_distance=*mean_distance/keypoints1.size();
    *RMSE=sqrt(*RMSE/keypoints1.size());
}
