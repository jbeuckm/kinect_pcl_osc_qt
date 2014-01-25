#ifndef WORKFLOWUI_H
#define WORKFLOWUI_H

#include <QMainWindow>
#include "oscsender.h"


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

    oscSender osc;
};

#endif // WORKFLOWUI_H
