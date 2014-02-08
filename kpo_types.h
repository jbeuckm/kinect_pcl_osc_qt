#ifndef KPO_TYPES_H
#define KPO_TYPES_H

typedef pcl::PointXYZ PointType;

typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;

typedef pcl::ReferenceFrame RFType;
typedef pcl::PointCloud<RFType> RFCloud;

#endif // KPO_TYPES_H
