#ifndef KPOBASEAPP_H
#define KPOBASEAPP_H


// QT4
#include <QMutex>
#include <QTimer>
#include <QObject>
#include <QSettings>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "kpoPclFunctions.h"
#include "kpoObjectDescription.h"
#include "kpo_types.h"
#include "kpoOscSender.h"


class kpoBaseApp
{
public:

    kpoBaseApp (pcl::OpenNIGrabber& grabber);

protected:
    pcl::OpenNIGrabber& grabber_;
    std::string device_id_;

    CloudPtr scene_cloud_;
    CloudPtr scene_keypoints_;
    NormalCloud::Ptr scene_normals_;
    DescriptorCloud::Ptr scene_descriptors_;
    RFCloud::Ptr scene_rf_;

    std::vector< boost::shared_ptr<kpoObjectDescription> > models_;
    std::vector< boost::shared_ptr<kpoObjectDescription> > match_queue_;

    pcl::PassThrough<PointType> depth_filter_;


};

#endif // KPOBASEAPP_H
