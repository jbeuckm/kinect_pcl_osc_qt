#include "workflow.h"
#include "ui_workflowui.h"



Workflow::Workflow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WorkflowUI)
{
    ui->setupUi(this);

    interface = new pcl::OpenNIGrabber();

    pc = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
    vis = new pcl::visualization::PCLVisualizer();

    vis->addPointCloud<pcl::PointXYZ> (pc, "kinect", 0);

    vtkSmartPointer< vtkRenderWindow > rw = vis->getRenderWindow();

    ui->qvtkWidget->SetRenderWindow(rw);
//    this->show();

}

Workflow::~Workflow()
{
    delete vis;
    delete ui;
}


void Workflow::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    vis->updatePointCloud<pcl::PointXYZ>(cloud);
}

void Workflow::startKinect()
{
  pcl::Grabber* interface = new pcl::OpenNIGrabber();

  boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
    boost::bind (&Workflow::cloud_cb_, this, _1);

  interface->registerCallback (f);

  interface->start ();
/*
  while (!viewer.wasStopped())
    {
  boost::this_thread::sleep (boost::posix_time::seconds (1));
    }
*/
  interface->stop ();
}

