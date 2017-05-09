#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include "get_LCP.h"
#include "get_mean_distance.h"

using namespace std;
typedef pcl::PointXYZ pcl_point;

void load_matrix(std:: string, Eigen::Matrix4f*);

int main(int argc, char *argv[])
{

    int i=atoi(argv[1]);
    int j=atoi(argv[2]);
    std::stringstream sstm;

    std::string file1;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/pcd/sampled/1_08cm/SW"<<i<<".pcd";
    file1 = sstm.str();
    std::string file2;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/pcd/sampled/1_08cm/SW"<<j<<".pcd";
    file2 = sstm.str();

    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( file1, *cloud_src );

    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( file2, *cloud_tgt );

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f truth = Eigen::Matrix4f::Identity();

    std::string transform_name;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/transforms/my_method/SW"<<i<<"_SW"<<j<<".txt";
 //   sstm<<"SW3_sampled_SW2_sampled.txt";
    transform_name = sstm.str();
    std::string ground_truth_name;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/transforms/leica/SW"<<i<<"_SW"<<j<<".txt";
    ground_truth_name = sstm.str();

    load_matrix(transform_name, &transform);
    load_matrix(ground_truth_name, &truth);

    //compute LCP 10cm

    float LCP10=0;
    get_LCP(*cloud_src, *cloud_tgt, &transform,0.1, &LCP10);
    float LCP10_truth=0;
    get_LCP(*cloud_src, *cloud_tgt, &truth,0.1, &LCP10_truth);

    //compute LCP 1cm

    float LCP1=0;
    get_LCP(*cloud_src, *cloud_tgt, &transform, 0.01, &LCP1);
    float LCP1_truth=0;
    get_LCP(*cloud_src, *cloud_tgt, &truth, 0.01, &LCP1_truth);

    //PCL validation

    pcl::registration::TransformationValidationEuclidean<pcl_point, pcl_point> validation;
    double pcl_score= validation.validateTransformation (cloud_src, cloud_tgt, transform);

    double pcl_score_truth= validation.validateTransformation (cloud_src, cloud_tgt, truth);

    //compute mean distance to closest neighbor

    float mean;
    float mean_truth;

    get_mean_distance(*cloud_src, *cloud_tgt, &truth, &mean_truth);
    get_mean_distance(*cloud_src, *cloud_tgt, &transform, &mean);

    //difference element to element of transformation matrix
    float error=0;
    for (int k=0; k<4; k++)
    {
        for (int l=0; l<4; l++)
        {
            error=error+(transform(k,l)-truth(k,l))*(transform(k,l)-truth(k,l));
        }
    }

    error=sqrt(error);

    //difference translation

    float error_trans=0;
    for (int l=0; l<3; l++)
    {
        error_trans=error_trans+(transform(l,3)-truth(l,3))*(transform(l,3)-truth(l,3));
    }
    error_trans=sqrt(error_trans);

    //difference angle rotation

    float theta_x_truth=atan2(truth(2,1),truth(2,2));
    float theta_y_truth=atan2(-truth(2,0),sqrt(truth(2,1)*truth(2,1)+truth(2,2)*truth(2,2)));
    float theta_z_truth=atan2(truth(1,0),truth(0,0));

    float theta_x=atan2(transform(2,1),transform(2,2));
    float theta_y=atan2(-transform(2,0),sqrt(transform(2,1)*transform(2,1)+transform(2,2)*transform(2,2)));
    float theta_z=atan2(transform(1,0),transform(0,0));

    float err_x=theta_x-theta_x_truth;
    float err_y=theta_y-theta_y_truth;
    float err_z=theta_z-theta_z_truth;

    //mean diff point to point method/ground truth

    float error_points=0;
    pcl::PointCloud<pcl_point>::Ptr cloud_src_my_method(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_src_leica(new pcl::PointCloud<pcl_point>);

    pcl::transformPointCloud (*cloud_src, *cloud_src_my_method, transform);
    pcl::transformPointCloud (*cloud_src, *cloud_src_leica, truth);

    for (int k=0; k<cloud_src->points.size(); k++)
    {
        float error_x=cloud_src_my_method->points[k].x-cloud_src_leica->points[k].x;
        float error_y=cloud_src_my_method->points[k].y-cloud_src_leica->points[k].y;
        float error_z=cloud_src_my_method->points[k].z-cloud_src_leica->points[k].z;
        error_points=error_points+sqrt(error_x*error_x+error_y*error_y+error_z*error_z);
    }

    error_points=error_points/cloud_src->points.size();

    //---------------------------------------------------------------------------------------------------------

    std::cout<<"LCP to 10 cm with Leica : "<<LCP10_truth*100<<"%"<<std::endl;
    std::cout<<"LCP to 10 cm with our method : "<<LCP10*100<<"%"<<std::endl<<std::endl;

    std::cout<<"LCP to 1 cm with Leica : "<<LCP1_truth*100<<"%"<<std::endl;
    std::cout<<"LCP to 1 cm with our method : "<<LCP1*100<<"%"<<std::endl<<std::endl;

    std::cout<<"PCL score leica : "<<pcl_score_truth<<"%"<<std::endl;
    std::cout<<"PCL score our method: "<<pcl_score<<"%"<<std::endl<<std::endl;

    std::cout<<"mean distance between source and target leica : "<<mean_truth*100<<"cm"<<std::endl;
    std::cout<<"mean distance between source and target our method : "<<mean*100<<"cm"<<std::endl<<std::endl;

    std::cout<<"similarity between matrices : "<<100/(1+error)<<"%"<<std::endl<<std::endl;

    std::cout<<"error in translation between the two methods : "<<error_trans*100<<"cm"<<std::endl<<std::endl;
    std::cout<<"error in rotation between the two methods : "<<err_x*180/M_PI<<"°\t"<<err_y*180/M_PI<<"°\t"<<err_z*180/M_PI<<"°"<<std::endl<<std::endl;

    std::cout<<"mean difference between points after transform with our method and with leica method : "<<error_points*100<<"cm"<<std::endl<<std::endl;


    ofstream file;
    file.open("evaluation/results", std::ofstream::out | std::ofstream::app);
    file<<"\n"<<"\n"<<"\n"
        <<"file1: "<<file1<<std::endl
        <<"file2: "<<file2<<std::endl
        <<"LCP 10cm truth : "<<LCP10_truth<<"\tLCP 10cm found : "<<LCP10<<std::endl
        <<"LCP 1cm truth : "<<LCP1_truth<<"\tLCP 1cm  found : "<<LCP1<<std::endl
        <<"PCL score truth : "<<pcl_score_truth<<"\tPCL score found :"<<pcl_score<<std::endl
        <<"mean distance truth : "<<mean_truth<<"\tmean distance found :"<<mean<<std::endl
        <<"similarity between matrices : "<<1/(1+error)<<std::endl
        <<"translation difference : "<<error_trans<<std::endl
        <<"rotation difference : "<<err_x<<"\t"<<err_y<<"\t"<<err_z<<std::endl
        <<"euclidian distance : "<<error_points<<std::endl<<std::endl;
    file.close();

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
