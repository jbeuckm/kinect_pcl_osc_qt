#ifndef PCL_FUNCTIONS_H
#define PCL_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



class kpoPclFunctions
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::Normal NormalType;
    typedef pcl::ReferenceFrame RFType;
    typedef pcl::SHOT352 DescriptorType;

    kpoPclFunctions();

    void estimateNormals(const pcl::PointCloud<PointType>::ConstPtr &cloud, pcl::PointCloud<NormalType>::Ptr &normals);

    void computeShotDescriptors(const pcl::PointCloud<PointType>::ConstPtr &cloud, const pcl::PointCloud<NormalType>::ConstPtr &normals);

private:

    float ss_;

};

#endif // PCL_FUNCTIONS_H
