#ifndef KPO_TYPES_H
#define KPO_TYPES_H

#include <boost/function.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>

#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <iostream>
#include <sstream>

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

class kpoCloudDescription
{
public:
    std::string filename;
    unsigned object_id;

    Cloud cloud;
    NormalCloud normals;
    Cloud keypoints;
    DescriptorCloud descriptors;
    RFCloud reference_frames;
};


typedef std::vector<cv::Point> Contour;

struct kpoObjectContour
{
    std::string filename;
    unsigned object_id;
    Contour contour;

    double error;

    bool operator < (const kpoObjectContour &c)
    {
        return (error < c.error);
    }
};


namespace boost {
namespace serialization {

template<typename Archive>
void serialize(Archive& ar, kpoObjectContour& o, const unsigned int version) {
  ar & o.filename & o.object_id & o.contour;
}

template<typename Archive>
void serialize(Archive& ar, cv::Point& o, const unsigned int version) {
  ar & o.x & o.y;
}

} // namespace serialization
} // namespace boost


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
