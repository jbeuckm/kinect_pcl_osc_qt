#ifndef KPO_ANALYZER_THREAD
#define KPO_ANALYZER_THREAD


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


typedef boost::function< void(kpoCloudDescription) > AnalyzerCallback;


class kpoAnalyzerThread
{
public:

    kpoAnalyzerThread();

    void copyInputCloud(Cloud::Ptr cloud, std::string filename, unsigned object_id);

    AnalyzerCallback callback_;
    void setAnalyzerCallback(AnalyzerCallback callback);

    void operator ()();

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

    float downsampling_radius_;
    float shot_radius_;
    float rf_rad_;

    float cg_size_;
    float cg_thresh_;

    std::string filename;
    unsigned object_id;

};

#endif // KPO_ANALYZER_THREAD
