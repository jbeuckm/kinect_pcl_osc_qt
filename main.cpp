#include <QtGui/QApplication>
#include "workflowui.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    WorkflowUI w;
    w.show();
    
    return a.exec();
}
