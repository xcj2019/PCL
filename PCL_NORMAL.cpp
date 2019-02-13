#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include<pcl/features/normal_3d.h>
#include<pcl/features/principal_curvatures.h>



int
main (int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile ("test2.pcd", *cloud);

    //计算法线--------------------------------------------------------------------------------------
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree); //设置搜索方法
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    //ne.setRadiusSearch(0.05); //设置半径邻域搜索
    ne.setKSearch(5);
    ne.compute(*cloud_normals); //计算法向量
    //计算法线--------------------------------------------------------------------------------------
    //计算曲率-------------------------------------------------------------------------------------
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pc.setInputCloud(cloud);
    pc.setInputNormals(cloud_normals);
    pc.setSearchMethod(tree);
    //pc.setRadiusSearch(0.05);
    pc.setKSearch(5);
    pc.compute(*cloud_curvatures);



    for (size_t i = 0; i < cloud_curvatures->size(); i++ )
    {
        double  normal_x=cloud_curvatures->points[i].principal_curvature_x;
        double  normal_y=cloud_curvatures->points[i].principal_curvature_y;
        double  normal_z=cloud_curvatures->points[i].principal_curvature_z;

        std::cout << "normal_x " << normal_x  << "normal_y " << normal_y  << "normal_z " << normal_z << std::endl;

    }

    // Save output
    //pcl::io::savePCDFile ("bun0-mls.pcd", cloud_curvatures);



    std::cout << "cloud_curvatures->size()" <<cloud_curvatures->size()<< std::endl;
    std::cout << "Normals good" << std::endl;
}