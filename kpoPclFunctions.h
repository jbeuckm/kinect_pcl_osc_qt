#ifndef PCL_FUNCTIONS_H
#define PCL_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



class kpoPclFunctions
{
public:
    kpoPclFunctions();

    void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &normals);

    void computeShotDescriptors(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const pcl::PointCloud<pcl::PointNormal>::ConstPtr &normals);

};

#endif // PCL_FUNCTIONS_H
