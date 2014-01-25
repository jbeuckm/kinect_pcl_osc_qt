#ifndef WORKFLOWUI_H
#define WORKFLOWUI_H

#include <QMainWindow>

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
};

#endif // WORKFLOWUI_H
