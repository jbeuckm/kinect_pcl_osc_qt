#include "workflowui.h"
#include "ui_workflowui.h"



WorkflowUI::WorkflowUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WorkflowUI)
{
    ui->setupUi(this);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

    vis.addPointCloud<pcl::PointXYZ>(pc);

//    ui->qvtkWidget->SetRenderWindow(vis.getRenderWindow());
//    this->show();

}

WorkflowUI::~WorkflowUI()
{
    delete ui;
}


