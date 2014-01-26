#ifndef WORKFLOWUI_H
#define WORKFLOWUI_H

#include <QMainWindow>
#include "oscsender.h"
#include <pcl/visualization/cloud_viewer.h>
#include <vtkSmartPointer.h>

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

    pcl::visualization::PCLVisualizer *vis;

    oscSender osc;
};

#endif // WORKFLOWUI_H
