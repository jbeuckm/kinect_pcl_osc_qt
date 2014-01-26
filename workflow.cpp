#include "workflow.h"
#include "ui_workflowui.h"
#include <vtkRenderWindow.h>


Workflow::Workflow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WorkflowUI)
{
    ui->setupUi(this);

    vis = new pcl::visualization::PCLVisualizer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

    vis->addPointCloud<pcl::PointXYZ>(pc);

    vtkSmartPointer< vtkRenderWindow > rw = vis->getRenderWindow();

    ui->qvtkWidget->SetRenderWindow(rw);
//    this->show();

}

Workflow::~Workflow()
{
    delete vis;
    delete ui;
}


