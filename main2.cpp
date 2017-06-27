#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include "get_LCP.h"
#include "resolution.h"
#include "compare_kpts.h"

using namespace std;
typedef pcl::PointXYZ pcl_point;

void load_matrix(std:: string, Eigen::Matrix4f*);
void load_kpts(std:: string file_name, pcl::PointCloud<pcl_point>::Ptr cloud);

int main(int argc, char *argv[])
{

    int i=atoi(argv[1]);
    int j=atoi(argv[2]);
    std::stringstream sstm;

    std::string file1;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/hokuyo/pcd/Hokuyo_"<<i<<".pcd";
    file1 = sstm.str();
    std::string file2;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/hokuyo/pcd/Hokuyo_"<<j<<".pcd";
    file2 = sstm.str();

    std::string name_kpts1;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/hokuyo/keypoints/"<<i<<"_"<<j<<"-"<<i<<".txt";
    name_kpts1 = sstm.str();

    std::string name_kpts2;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/hokuyo/keypoints/"<<j<<"_"<<j<<"-"<<i<<".txt";
    name_kpts2 = sstm.str();

    pcl::PointCloud<pcl_point>::Ptr keypoints1(new pcl::PointCloud<pcl_point>);
    load_kpts(name_kpts1, keypoints1);
    pcl::PointCloud<pcl_point>::Ptr keypoints2(new pcl::PointCloud<pcl_point>);
    load_kpts(name_kpts2, keypoints2);


    pcl::io::savePCDFileASCII ("keypoints1.pcd", *keypoints1);
    pcl::io::savePCDFileASCII ("keypoints2.pcd", *keypoints2);

    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( file1, *cloud_src );

    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( file2, *cloud_tgt );

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    std::string transform_name;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/hokuyo/transform/my_method/Hokuyo_"<<i<<"_Hokuyo_"<<j<<".txt";
    transform_name = sstm.str();

    load_matrix(transform_name, &transform);
    std::cout<<"path : "<<transform_name<<std::endl<<std::endl;
    std::cout<<"transform : "<<std::endl<<transform<<std::endl<<std::endl<<std::endl;

    pcl::KdTreeFLANN<pcl_point>::Ptr tree_tgt(new pcl::KdTreeFLANN<pcl_point>);
    tree_tgt->setInputCloud(cloud_tgt);

    float res=resolution(cloud_tgt, tree_tgt);
    std::cout<<"resolution : "<<res<<std::endl<<std::endl;

    //compute LCP and mean distance and RMSE

    float LCP=0;
    float md=0;
    float md_kpts=0;
    float RMSE=0;
    float RMSE_kpts=0;
    get_LCP(cloud_src, tree_tgt, 0.02, &transform, &LCP, &md, &RMSE);
    compare_kpts(keypoints1, keypoints2, &transform, &md_kpts, &RMSE_kpts);

    //---------------------------------------------------------------------------------------------------------

    std::cout<<"LCP with our method : "<<LCP*100<<"%"<<std::endl<<std::endl;

    std::cout<<"mean distance between source and target keypoints : "<<md_kpts*100<<"cm"<<std::endl;
    std::cout<<"mean distance between source and target all points : "<<md*100<<"cm"<<std::endl<<std::endl;

    std::cout<<"RMSE keypoints : "<<RMSE_kpts<<"m"<<std::endl;
    std::cout<<"RMSE all points : "<<RMSE<<"m"<<std::endl<<std::endl;

}




void load_matrix(std:: string file_name, Eigen::Matrix4f* matrix)
{
    ifstream fin;
    fin.open (file_name);
    Eigen::Matrix4f res = Eigen::Matrix4f::Identity();

    if (fin.is_open())
    {
        for (int row = 0; row < 4; row++)
            for (int col = 0; col < 4; col++)
            {
                float item = 0.0;
                fin >> item;
                res(row, col) = item;
            }
        fin.close();
    }
    *matrix=res;
}

void load_kpts(std:: string file_name, pcl::PointCloud<pcl_point>::Ptr cloud)
{
    ifstream fin;
    fin.open (file_name);

    if (fin.is_open())
    {
        while (!fin.eof())
        {
            pcl_point point;
            float item = 0.0;
            fin >> item;
            if(!fin.eof())
            {
                point.x = item;
                fin >> item;
                point.y = item;
                fin >> item;
                point.z = item;
                cloud->points.push_back(point);
            }
        }
    }
    fin.close();
    cloud->width = (uint32_t)  cloud->points.size();
    cloud->height = 1;

}

