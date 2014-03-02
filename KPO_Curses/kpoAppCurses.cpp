#include "kpoAppCurses.h"

kpoAppCurses::kpoAppCurses(pcl::OpenNIGrabber& grabber)
    : kpoBaseApp(grabber)
{
    paused_ = false;
    process_scene_ = true;
    match_models_ = true;
}
