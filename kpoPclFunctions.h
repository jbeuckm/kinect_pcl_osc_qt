#ifndef PCL_FUNCTIONS_H
#define PCL_FUNCTIONS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class kpoPclFunctions
{
public:
    kpoPclFunctions();

    void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &normals);

};

#endif // PCL_FUNCTIONS_H
