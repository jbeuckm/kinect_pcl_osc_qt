#ifndef WORKFLOWUI_H
#define WORKFLOWUI_H

#include <QMainWindow>
#include "oscsender.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h>

#include <pcl/io/openni_grabber.h>
#include <vtkRenderWindow.h>


namespace Ui {
class WorkflowUI;
}

class Workflow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit Workflow(QWidget *parent = 0);
    ~Workflow();
    
private:
    Ui::WorkflowUI *ui;

    pcl::Grabber* interface;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    pcl::visualization::CloudViewer viewer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

    pcl::visualization::PCLVisualizer *vis;

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
    void startKinect();

    oscSender osc;
};

#endif // WORKFLOWUI_H
