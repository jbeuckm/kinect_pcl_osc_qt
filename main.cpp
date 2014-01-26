#include <QtGui/QApplication>
#include "workflow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Workflow w;
    w.show();
    
    return a.exec();
}
