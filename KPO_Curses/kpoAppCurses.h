#ifndef KPOAPPCURSES_H
#define KPOAPPCURSES_H

#include "kpoBaseApp.h"

class kpoAppCurses : public kpoBaseApp
{
public:
    kpoAppCurses(pcl::OpenNIGrabber& grabber);

};

#endif // KPOAPPCURSES_H
