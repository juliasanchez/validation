#include <iostream>
#include <fstream>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include "get_LCP.h"
#include "resolution.h"

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
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/pcd/sampled/1_04cm/SW"<<i<<".pcd";
    file1 = sstm.str();
    std::string file2;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/pcd/sampled/1_04cm/SW"<<j<<".pcd";
    file2 = sstm.str();


    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( file1, *cloud_src );

    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);
    pcl::io::loadPCDFile<pcl_point>( file2, *cloud_tgt );

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f truth = Eigen::Matrix4f::Identity();

    std::string transform_name;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/transforms/leica_sans_cible/SW"<<i<<"_SW"<<j<<".txt";
 //   sstm<<"SW3_sampled_SW2_sampled.txt";
    transform_name = sstm.str();
    std::string ground_truth_name;
    sstm.str("");
    sstm<<"/home/julia/Documents/data_base/patio_pp_exported_data/transforms/leica/SW"<<i<<"_SW"<<j<<".txt";
    ground_truth_name = sstm.str();

    load_matrix(transform_name, &transform);
    load_matrix(ground_truth_name, &truth);
    std::cout<<"transform_truth : "<<std::endl<<truth<<std::endl<<std::endl<<std::endl;
    std::cout<<"transform : "<<std::endl<<transform<<std::endl<<std::endl<<std::endl;

    pcl::KdTreeFLANN<pcl_point>::Ptr tree_tgt(new pcl::KdTreeFLANN<pcl_point>);
    tree_tgt->setInputCloud(cloud_tgt);

    float res=resolution(cloud_tgt, tree_tgt);
    std::cout<<"resolution : "<<res<<std::endl<<std::endl;

    //compute LCP and mean distance and RMSE

    float LCP=0;
    float md=0;
    float md_truth=0;
    float RMSE=0;
    float RMSE_truth=0;
    get_LCP(cloud_src, tree_tgt, res, &transform, &LCP, &md, &RMSE);
    float LCP_truth=0;
    get_LCP(cloud_src, tree_tgt, res, &truth, &LCP_truth, &md_truth, &RMSE_truth);
    float rap=abs(LCP-LCP_truth)/LCP_truth;


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

    //RMSE source/target considering correspondences with ground truth

    pcl::PointCloud<pcl_point> cloud_src_transformed;
    pcl::transformPointCloud (*cloud_src, cloud_src_transformed, truth);

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    map<int, int> mapa;

    float rmse1=0;
    std::vector<int> points;

    for (int k=0; k<cloud_src_transformed.size(); k++)
    {

        if ( tree_tgt->nearestKSearch (cloud_src_transformed.points[k], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            if(sqrt(pointNKNSquaredDistance[0])<0.05)
            {
                mapa[k]= pointIdxNKNSearch[0];
                rmse1= rmse1 +pointNKNSquaredDistance[0];
                points.push_back(k);
            }
        }
    }

    rmse1=sqrt(rmse1/points.size());

    pcl::transformPointCloud (*cloud_src, cloud_src_transformed, transform);
    float rmse0=0;

    for (int k=0; k<points.size(); k++)
    {
        float diff_x=cloud_src_transformed.points[points[k]].x-cloud_tgt->points[mapa[points[k]]].x;
        float diff_y=cloud_src_transformed.points[points[k]].y-cloud_tgt->points[mapa[points[k]]].y;
        float diff_z=cloud_src_transformed.points[points[k]].z-cloud_tgt->points[mapa[points[k]]].z;
        rmse0= rmse0+ diff_x*diff_x +diff_y*diff_y +diff_z +diff_z;
    }

    rmse0=sqrt(rmse0/points.size());

    //---------------------------------------------------------------------------------------------------------

    std::cout<<"LCP with Leica : "<<LCP_truth*100<<"%"<<std::endl;
    std::cout<<"LCP with our method : "<<LCP*100<<"%"<<std::endl<<std::endl;
    std::cout<<"relation LCP : "<<rap*100<<"%"<<std::endl<<std::endl;

    std::cout<<"mean distance between source and target leica : "<<md_truth*100<<" cm"<<std::endl;
    std::cout<<"mean distance between source and target our method : "<<md*100<<" cm"<<std::endl<<std::endl;

    std::cout<<"RMSE source and target leica : "<<RMSE_truth<<" m"<<std::endl;
    std::cout<<"RMSE source and target our method : "<<RMSE<<" m"<<std::endl<<std::endl;

    std::cout<<"similarity between matrices : "<<100/(1+error)<<"%"<<std::endl<<std::endl;

    std::cout<<"error in translation between the two methods : "<<error_trans*100<<" cm"<<std::endl<<std::endl;
    std::cout<<"error in rotation between the two methods : "<<err_x*180/M_PI<<"°\t"<<err_y*180/M_PI<<"°\t"<<err_z*180/M_PI<<"°"<<std::endl<<std::endl;

    std::cout<<"mean difference between points after transform with our method and with leica method : "<<error_points*100<<" cm"<<std::endl<<std::endl;
    std::cout<<"rmse with ground truth reference: "<<rmse0<<" cm"<<std::endl<<std::endl;
    std::cout<<"rmse for ground truth: "<<rmse1<<" cm"<<std::endl<<std::endl;


    ofstream file;
    file.open("evaluation/results", std::ofstream::out | std::ofstream::app);
    file<<"\n"<<"\n"<<"\n"
        <<"file1: "<<file1<<std::endl
        <<"file2: "<<file2<<std::endl
        <<"LCP truth : "<<LCP_truth<<"\tLCP : "<<LCP<<std::endl
        <<"mean distance truth : "<<md_truth<<"\tmean distance :"<<md<<std::endl
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
