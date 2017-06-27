void get_LCP(pcl::KdTreeFLANN<pcl_point>::Ptr tree, pcl::PointCloud<pcl_point>::Ptr cloud_src_transformed, float thresh, float* LCP)
{

    *LCP=0;
    int n=0;

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    for (int k=0; k<cloud_src_transformed->size(); k++)
    {

        if ( tree->nearestKSearch (cloud_src_transformed->points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            if(sqrt(pointNKNSquaredDistance[0])<thresh)
            {
                *LCP=*LCP+1;
                n++;
            }
        }
    }

    *LCP=*LCP/cloud_src_transformed->size();
}
