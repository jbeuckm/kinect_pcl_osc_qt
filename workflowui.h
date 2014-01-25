#ifndef WORKFLOWUI_H
#define WORKFLOWUI_H

#include <QMainWindow>
#include "oscsender.h"
#include <pcl/visualization/cloud_viewer.h>


namespace Ui {
class WorkflowUI;
}

class WorkflowUI : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit WorkflowUI(QWidget *parent = 0);
    ~WorkflowUI();
    
private:
    Ui::WorkflowUI *ui;

    pcl::visualization::PCLVisualizer vis;

    oscSender osc;
};

#endif // WORKFLOWUI_H
