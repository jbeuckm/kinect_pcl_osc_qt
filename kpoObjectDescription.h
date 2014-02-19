#ifndef KPOOBJECTDESCRIPTION_H
#define KPOOBJECTDESCRIPTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kpo_types.h"


class kpoObjectDescription
{
public:
    kpoObjectDescription();
    kpoObjectDescription(CloudPtr _cloud, CloudPtr _keypoints, NormalCloudPtr _normals, DescriptorCloud::Ptr _descriptors, RFCloud::Ptr _reference_frames);

    int object_id;

    Cloud::Ptr cloud;
    NormalCloud::Ptr normals;
    Cloud::Ptr keypoints;
    DescriptorCloud::Ptr descriptors;
    RFCloud::Ptr reference_frames;

};

#endif // KPOOBJECTDESCRIPTION_H
