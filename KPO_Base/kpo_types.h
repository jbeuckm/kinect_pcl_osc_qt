#ifndef KPO_TYPES_H
#define KPO_TYPES_H

#include <boost/function.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>

typedef pcl::PointXYZRGBA PointType;

typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

typedef pcl::SHOT1344 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;

typedef pcl::ReferenceFrame RFType;
typedef pcl::PointCloud<RFType> RFCloud;

struct kpoCloudDescription
{
    std::string filename;
    unsigned object_id;

    Cloud::Ptr cloud;
    NormalCloud::Ptr normals;
    Cloud::Ptr keypoints;
    DescriptorCloud::Ptr descriptors;
    RFCloud::Ptr reference_frames;
};


typedef std::vector<cv::Point> Contour;

struct kpoObjectContour
{
    std::string filename;
    unsigned object_id;
    Contour contour;
};


// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

#endif // KPO_TYPES_H
