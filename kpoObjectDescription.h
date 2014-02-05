#ifndef KPOOBJECTDESCRIPTION_H
#define KPOOBJECTDESCRIPTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class kpoObjectDescription
{
public:
    kpoObjectDescription();
    kpoObjectDescription(pcl::PointCloud<pcl::PointXYZ>::Ptr _keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr _descriptors);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints;
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors;
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr reference_frames;

};

#endif // KPOOBJECTDESCRIPTION_H
