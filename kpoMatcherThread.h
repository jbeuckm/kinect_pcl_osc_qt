#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/recognition/cg/hough_3d.h>

#include "kpo_types.h"

class kpoMatcherThread {

public:
    int object_id;
    std::string filename;

    kpoMatcherThread(Cloud::Ptr model_keypoints_, DescriptorCloud::Ptr model_descriptors_, RFCloud::Ptr model_refs_);

    Cloud::Ptr model_keypoints;
    DescriptorCloud::Ptr model_descriptors;
    RFCloud::Ptr model_refs;

    Cloud::Ptr scene_keypoints;
    DescriptorCloud::Ptr scene_descriptors;
    RFCloud::Ptr scene_refs;

    float cg_size_;
    float cg_thresh_;

    void copySceneClouds(Cloud::Ptr scene_keypoints_, DescriptorCloud::Ptr scene_descriptors_, RFCloud::Ptr scene_refs_);

    // to be called what matches are found
    MatchCallback callback_;
    void setMatchCallback(MatchCallback callback);

    void operator ()();

};

