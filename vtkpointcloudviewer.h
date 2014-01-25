#ifndef VTKPOINTCLOUDVIEWER_H
#define VTKPOINTCLOUDVIEWER_H

#include <QVTKWidget.h>
#include <pcl/visualization/cloud_viewer.h>

class VTKPointCloudWidget : public QVTKWidget
{
public:
    VTKPointCloudWidget();
    VTKPointCloudWidget(QWidget *parent);
//    void showPointCloud(SensorPicture pic);

private:
    pcl::visualization::PCLVisualizer vis;
};

#endif // VTKPOINTCLOUDVIEWER_H
