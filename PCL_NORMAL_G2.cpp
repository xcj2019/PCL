//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
//#include <pcl/features/principal_curvatures.h>
//
//
//int
//main (int argc, char** argv)
//{
//    // Load input file into a PointCloud<T> with an appropriate type
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    // Load bun0.pcd -- should be available with the PCL archive in test
//    pcl::io::loadPCDFile ("bin_Laser-00152_-00851.pcd", *cloud);
//
//    // Create a KD-Tree
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//
//    // Output has the PointNormal type in order to store the normals calculated by MLS
//    pcl::PointCloud<pcl::Normal>  ::Ptr Normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::PointCloud<pcl::PointNormal> ::Ptr PointWithNormals (new pcl::PointCloud<pcl::PointNormal>);
//
//    // Init object (second point type is for the normals, even if unused)
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//
//    mls.setComputeNormals (true);
//
//    // Set parameters
//    mls.setInputCloud (cloud);
//    mls.setPolynomialOrder (2);
//    mls.setSearchMethod (tree);
//    mls.setSearchRadius (0.03);
//
//    // Reconstruct
//    mls.process (*PointWithNormals);
//
//
//
//    size_t size;
//    size_t i;
//    size = PointWithNormals->points.size();
//    Normals->points.resize(PointWithNormals->size());
//
//    for (i = 0; i < size; i++ )
//    {
//
//        Normals->points[i].normal_x = PointWithNormals->points[i].normal_x;
//        Normals->points[i].normal_y = PointWithNormals->points[i].normal_y;
//        Normals->points[i].normal_z = PointWithNormals->points[i].normal_z;
//    }
//
//
//    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
//
//// Provide the original point cloud (without normals)
//    principalCurvaturesEstimation.setInputCloud (cloud);
//
//// Provide the point cloud with normals
//    principalCurvaturesEstimation.setInputNormals(Normals);
//
//// Use the same KdTree from the normal estimation
//    principalCurvaturesEstimation.setSearchMethod (tree);
//    principalCurvaturesEstimation.setKSearch(10);
//
//// Actually compute the principal curvatures
//    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
//    principalCurvaturesEstimation.compute (*principalCurvatures); // only provide with pc1,pc2 and normals
//
//
//
//
//
//
//    // Save output
////    pcl::io::savePCDFile ("Normals.pcd", principalCurvatures);
//}

#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

int main (int argc, char** argv){
    typedef pcl::PointXYZRGBA PointT;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

//    pcl::io::loadPCDFile ("test.pcd", *cloud);//读取pcd文件

    if (pcl::io::loadPCDFile ("test.pcd", *cloud) == -1){
//* load the file
        PCL_ERROR ("Couldn't read PCD file \n");
        return (-1);
    }
    printf("Loaded %d data points from PCD\n",
           cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); i+=10000)
        printf("%8.3f %8.3f %8.3f\n",
               cloud->points[i].x,
               cloud->points[i].y,
               cloud->points[i].z

        );

    pcl::visualization::PCLVisualizer viewer("Cloud viewer");
    viewer.setCameraPosition(0,0,0,0,0,0);
    viewer.addCoordinateSystem(0.3);

    viewer.addPointCloud(cloud);
    while(!viewer.wasStopped())
        viewer.spinOnce(50);
    return (0);
}