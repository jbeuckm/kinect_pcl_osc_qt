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

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include "kpo_types.h"

class kpoPclFunctions
{
public:

    kpoPclFunctions();

    void removeNoise(const CloudConstPtr &cloud, CloudPtr &filtered_cloud);

    void setDownsamplingRadius(float _radius);

    void estimateNormals(const Cloud::ConstPtr &cloud, NormalCloudPtr &normals);

    void computeShotDescriptors(const CloudConstPtr &cloud, const CloudConstPtr &keypoints, const NormalCloud::ConstPtr &normals, DescriptorCloud::Ptr &descriptors);

    void correlateDescriptors(const DescriptorCloud::ConstPtr &scene_descriptors, const DescriptorCloud::ConstPtr &model_descriptors, pcl::CorrespondencesPtr &model_scene_corrs);

    std::vector<pcl::Correspondences> clusterCorrespondences(const CloudConstPtr &scene_keypoints, const CloudConstPtr &model_keypoints, const pcl::CorrespondencesPtr &model_scene_corrs);

    void estimateReferenceFrames(const Cloud::ConstPtr &cloud,
                                 const NormalCloud::ConstPtr &normals,
                                 const Cloud::ConstPtr &keypoints,
                                 RFCloud::Ptr &rf);

    void setHoughSceneCloud(const CloudConstPtr &scene_keypoints, const RFCloud::ConstPtr &scene_rf);
    void houghCorrespondences(const CloudConstPtr &model_keypoints, const RFCloud::ConstPtr &model_rf, const pcl::CorrespondencesPtr &model_scene_corrs,
                std::vector<pcl::Correspondences> &clustered_corrs, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &rototranslations);

    double computeCloudResolution (const CloudConstPtr &cloud);
    void downSample(const CloudConstPtr &cloud, CloudPtr &keypoints);


private:

    float downsampling_radius_;
    float shot_radius_;
    float rf_rad_;

    float cg_size_;
    float cg_thresh_;

    pcl::StatisticalOutlierRemoval<PointType> sor;

    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    pcl::UniformSampling<PointType> uniform_sampling;
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> shot;
    pcl::KdTreeFLANN<DescriptorType> match_search;

    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
};

#endif // PCL_FUNCTIONS_H
