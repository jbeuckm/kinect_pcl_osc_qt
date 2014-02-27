#include <QtCore/QCoreApplication>


#include "kpoAppCurses.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    

    // Open the first available camera
    pcl::OpenNIGrabber grabber ("#1");
    // Check if an RGB stream is provided
    if (!grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud> ())
    {
        PCL_ERROR ("Device #1 does not provide an RGB stream!\n");
        return (-1);
    }

    kpoAppCurses v (grabber);


    return (app.exec ());
}
