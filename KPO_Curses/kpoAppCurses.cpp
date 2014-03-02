#include "kpoAppCurses.h"

kpoAppCurses::kpoAppCurses(pcl::OpenNIGrabber& grabber)
    : kpoBaseApp(grabber)
{
    paused_ = false;
    process_scene_ = true;
    match_models_ = true;
}

void kpoAppCurses::loadSettings()
{
    kpoBaseApp::loadSettings();

    process_scene_ = true;
    match_models_ = true;
}
