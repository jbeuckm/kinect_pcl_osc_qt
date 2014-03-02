#ifndef KPOCLOUDANALYZER_H
#define KPOCLOUDANALYZER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "kpo_types.h"

#include "kpoPclFunctions.h"
#include "kpoObjectDescription.h"

typedef boost::function<void(kpoObjectDescription)> AnalysisCallback;

class kpoCloudAnalyzer
{
public:
    kpoCloudAnalyzer();

    void setInputCloud(CloudPtr inputCloud);

    std::string filename;
    unsigned object_id;

    AnalysisCallback callback_;

    void operator ()();

private:
    kpoPclFunctions pcl_functions_;

    CloudPtr scene_cloud_;
    CloudPtr scene_keypoints_;
    NormalCloud::Ptr scene_normals_;
    DescriptorCloud::Ptr scene_descriptors_;
    RFCloud::Ptr scene_refs_;
};

#endif // KPOCLOUDANALYZER_H
