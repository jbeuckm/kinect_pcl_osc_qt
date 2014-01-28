#include "kpoPclFunctions.h"
#include <pcl/common/io.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/shot.h>
#include <pcl/features/vfh.h>

#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

kpoPclFunctions::kpoPclFunctions()
{
}


void kpoPclFunctions::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::PointCloud<pcl::PointNormal> &normals)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

//    mls.process (normals);
}
