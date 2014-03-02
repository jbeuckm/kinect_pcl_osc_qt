#ifndef PCL_FUNCTIONS_H
#define PCL_FUNCTIONS_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/io.h>
#include <pcl/surface/mls.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/filter.h>

#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>

#include <pcl/correspondence.h>

#include <pcl/features/board.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <opencv/cv.h>

#include "kpo_types.h"
#include "kpoObjectDescription.h"


typedef boost::function<void(kpoObjectDescription)> AnalyzerCallback;


class kpoAnalyzerThread
{
public:

    kpoAnalyzerThread(float downsampling_radius=.0075f);

    void setInputCloud(CloudPtr &cloud);
    void operator ()();
    AnalyzerCallback callback_;

    CloudPtr scene_cloud_;
    CloudPtr scene_keypoints_;
    NormalCloud::Ptr scene_normals_;
    DescriptorCloud::Ptr scene_descriptors_;
    RFCloud::Ptr scene_refs_;

    void removeNoise(const CloudConstPtr &cloud, Cloud &filtered_cloud);

    double computeCloudResolution (const CloudConstPtr &cloud);

    void estimateNormals(CloudPtr &cloud, NormalCloudPtr &normals);

    void setDownsamplingRadius(float _radius);
    void downSample(const CloudConstPtr &cloud, CloudPtr &keypoints);

    void computeShotDescriptors(const CloudConstPtr &cloud, const CloudConstPtr &keypoints, const NormalCloud::ConstPtr &normals, DescriptorCloud::Ptr &descriptors);

    void estimateReferenceFrames(const Cloud::ConstPtr &cloud,
                                 const NormalCloud::ConstPtr &normals,
                                 const Cloud::ConstPtr &keypoints,
                                 RFCloud::Ptr &rf);


    void openniImage2opencvMat(const XnRGB24Pixel* pImageMap, cv::Mat& cv_image, int rows, int cols)
    {
      int sizes[2] = {rows, cols};
      cv_image = cv::Mat(2, sizes, CV_8UC3, (void*) pImageMap);
    }
    void openniImage2opencvMat(const XnDepthPixel* pDepthMap, cv::Mat& cv_depth, int rows, int cols)
    {
      int sizes[2] = {rows, cols};
      cv_depth = cv::Mat(2, sizes, CV_16UC1, (void*) pDepthMap);
    }

private:

    float downsampling_radius_;
    float shot_radius_;
    float rf_rad_;

    float cg_size_;
    float cg_thresh_;


    pcl::StatisticalOutlierRemoval<PointType> statistical_outlier_remover;

    pcl::NormalEstimation<PointType, NormalType> norm_est;

    pcl::UniformSampling<PointType> uniform_sampling;

    pcl::SHOTColorEstimation<PointType, NormalType, DescriptorType> shot;

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;

};

#endif // PCL_FUNCTIONS_H
