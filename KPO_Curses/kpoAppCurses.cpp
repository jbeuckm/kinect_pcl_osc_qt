#include "kpoAppCurses.h"

kpoAppCurses::kpoAppCurses(pcl::OpenNIGrabber& grabber)
    : kpoBaseApp(grabber)
{
    remove_noise_ = false;
    paused_ = false;
    estimate_normals_ = true;
    match_models_ = true;
}
