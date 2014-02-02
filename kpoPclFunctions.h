#ifndef PCL_FUNCTIONS_H
#define PCL_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/io.h>
#include <pcl/surface/mls.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>


class kpoPclFunctions
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::Normal NormalType;
    typedef pcl::ReferenceFrame RFType;
    typedef pcl::SHOT352 DescriptorType;

    kpoPclFunctions();

    void estimateNormals(const pcl::PointCloud<PointType>::ConstPtr &cloud, pcl::PointCloud<NormalType>::Ptr &normals);

    void computeShotDescriptors(const pcl::PointCloud<PointType>::ConstPtr &cloud, const pcl::PointCloud<NormalType>::ConstPtr &normals, pcl::PointCloud<DescriptorType>::Ptr &descriptors);

    void matchModelInScene(const pcl::PointCloud<DescriptorType>::ConstPtr &scene_descriptors, const pcl::PointCloud<DescriptorType>::ConstPtr &model_descriptors);

private:

    float downsampling_radius_;

    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    pcl::UniformSampling<PointType> uniform_sampling;
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> shot;
    pcl::KdTreeFLANN<DescriptorType> match_search;
};

#endif // PCL_FUNCTIONS_H
