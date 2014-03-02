#ifndef KPOAPPCURSES_H
#define KPOAPPCURSES_H

#include "kpoBaseApp.h"

class kpoAppCurses : public kpoBaseApp
{
public:
    kpoAppCurses(pcl::OpenNIGrabber& grabber);

    void loadSettings();
};

#endif // KPOAPPCURSES_H
