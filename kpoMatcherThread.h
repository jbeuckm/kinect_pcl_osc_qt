#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/recognition/cg/hough_3d.h>

#include "kpo_types.h"


class kpoMatcherThread {

public:

    kpoMatcherThread(Cloud::Ptr model_keypoints_, DescriptorCloud::Ptr model_descriptors_, RFCloud::Ptr model_refs_);

    Cloud::Ptr model_keypoints;
    DescriptorCloud::Ptr model_descriptors;
    RFCloud::Ptr model_refs;

    Cloud::Ptr scene_keypoints;
    DescriptorCloud::Ptr scene_descriptors;
    RFCloud::Ptr scene_refs;

    pcl::KdTreeFLANN<DescriptorType> match_search;
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> hough_clusterer;

    int operator ()();

};

