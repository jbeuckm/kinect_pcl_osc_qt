#ifndef KPO_TYPES_H
#define KPO_TYPES_H

#include <boost/function.hpp>

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

typedef boost::function<void(unsigned, Eigen::Vector3f, Eigen::Matrix3f)> MatchCallback;


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
