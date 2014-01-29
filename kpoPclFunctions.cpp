#include "kpoPclFunctions.h"
#include <pcl/common/io.h>

#include <pcl/features/shot.h>

#include <pcl/surface/mls.h>

#include <pcl/search/kdtree.h>

kpoPclFunctions::kpoPclFunctions()
{
}


void kpoPclFunctions::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &normals)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    normals.reset (new pcl::PointCloud<pcl::PointNormal>);
    mls.process (*normals);
}



void computeShotDescriptors(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const pcl::PointCloud<pcl::PointNormal>::ConstPtr &normals)
{
/*
    pcl::SHOTEstimation<... > shot;
    shot.setSearchMethod (tree); //kdtree
    shot.setIndices (indices); //keypoints
    shot.setInputCloud (cloud); //input
    shot.setInputNormals(normals);//normals
    shot.setRadiusSearch (0.06); //support
    shot.compute (*shots); //descriptors
*/
}
