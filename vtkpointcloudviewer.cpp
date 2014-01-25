
#include "vtkpointcloudviewer.h"


VTKPointCloudWidget::VTKPointCloudWidget(QWidget *parent) : QVTKWidget(parent)
{
    /*
  this->resize(500, 500);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

  vis.addPointCloud<pcl::PointXYZ>(pc);

  this->SetRenderWindow(vis.getRenderWindow());
  this->show();

*/

      boost::shared_ptr<pcl::visualization::PCLVisualizer> pv(new pcl::visualization::PCLVisualizer("3D Viewer"));
/*
       if ( m_cloud.size() > 0 )
       {
               pv->addPointCloud<pcl::PointXYZ>(m_cloud.makeShared());
       }
*/
       pv->setBackgroundColor(0, 0, 0.1);

       vtkSmartPointer<vtkRenderWindow> renderWindow = pv->getRenderWindow();

//       this->SetRenderWindow( renderWindow );
       this->update();

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
