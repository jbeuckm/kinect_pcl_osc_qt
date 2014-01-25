#ifndef VTKPOINTCLOUDVIEWER_H
#define VTKPOINTCLOUDVIEWER_H

#include <pcl/visualization/cloud_viewer.h>

class VTKPointCloudViewer : public QVTKWidget
{
public:
    VTKPointCloudViewer();
private:
    pcl::visualization::PCLVisualizer vis;
};

#endif // VTKPOINTCLOUDVIEWER_H
