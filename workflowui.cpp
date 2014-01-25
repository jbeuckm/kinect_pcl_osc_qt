#include "workflowui.h"
#include "ui_workflowui.h"

WorkflowUI::WorkflowUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WorkflowUI)
{
    ui->setupUi(this);
}

WorkflowUI::~WorkflowUI()
{
    delete ui;
}


