#include "vtkpointcloudviewer.h"


VTKPointCloudWidget::VTKPointCloudWidget(QWidget *parent) : QVTKWidget(parent)
{
  this->resize(500, 500);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

  vis.addPointCloud<pcl::PointXYZ>(pc);
  vtkSmartPointer<vtkRenderWindow> renderWindow = vis.getRenderWindow();
  this->SetRenderWindow(renderWindow);
  this->show();
}

/*
void VTKPointCloudWidget::showPointCloud(SensorPicture pic)
{
  // converts the sensor image to a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc = cvtSP2PC(pic);

  pc->width = pic.width;
  pc->height = pic.height;

  vis.updatePointCloud<pcl::PointXYZ>(pc);

  this->update();
}
*/
