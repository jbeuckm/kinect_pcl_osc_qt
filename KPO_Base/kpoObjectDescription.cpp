#include "kpoObjectDescription.h"

kpoObjectDescription::kpoObjectDescription()
{
}

kpoObjectDescription::kpoObjectDescription(CloudPtr _cloud, CloudPtr _keypoints, NormalCloudPtr _normals, DescriptorCloud::Ptr _descriptors, RFCloud::Ptr _reference_frames)
{
    cloud = _cloud;
    keypoints = _keypoints;
    normals = _normals;
    descriptors = _descriptors;
    reference_frames = _reference_frames;
}
