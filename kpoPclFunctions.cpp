#include "kpoPclFunctions.h"
#include <pcl/common/io.h>

#include <pcl/features/shot.h>

#include <pcl/surface/mls.h>

#include <pcl/search/kdtree.h>
#include <pcl/keypoints/uniform_sampling.h>


kpoPclFunctions::kpoPclFunctions()
{
    ss_ = .01f;
}


void kpoPclFunctions::computeNormals(const pcl::PointCloud<PointType>::ConstPtr &cloud, pcl::PointCloud<NormalType>::Ptr &normals)
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



void kpoPclFunctions::computeShotDescriptors(const pcl::PointCloud<PointType>::ConstPtr &cloud, const pcl::PointCloud<NormalType>::ConstPtr &normals)
{

    pcl::PointCloud<int> sampled_indices;
    pcl::PointCloud<PointType>::Ptr keypoints (new pcl::PointCloud<PointType> ());
/*
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
    std::cout << "Cloud total points: " << cloud->size () << "; Selected Keypoints: " << keypoints->size () << std::endl;


    pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::SHOTEstimation<PointType, NormalType, DescriptorType> shot;

    shot.setSearchMethod (tree); //kdtree
    shot.setIndices (indices); //keypoints
    shot.setInputCloud (cloud); //input
    shot.setInputNormals(normals);//normals
    shot.setRadiusSearch (0.06); //support
    shot.compute (*descriptors); //descriptors
*/
}
