#include "kpoObjectDescription.h"

kpoObjectDescription::kpoObjectDescription()
{
}

kpoObjectDescription::kpoObjectDescription(pcl::PointCloud<pcl::PointXYZ>::Ptr _keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr _descriptors)
{
    keypoints = _keypoints;
    descriptors = _descriptors;
}
