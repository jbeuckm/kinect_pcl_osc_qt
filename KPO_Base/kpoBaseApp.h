#ifndef KPOBASEAPP_H
#define KPOBASEAPP_H


#include <iostream>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/threadpool.hpp>

// QT4
#include <QtCore>
#include <QMutex>
#include <QTimer>
#include <QObject>
#include <QSettings>
#include <QtDebug>
#include <QDirIterator>
#include <QElapsedTimer>


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "kpoPclFunctions.h"
#include "kpoObjectDescription.h"
#include "kpo_types.h"
#include "kpoOscSender.h"
#include "kpoMatcherThread.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "KPO_Base_global.h"
#include "BlobFinder.h"

class KPO_BASESHARED_EXPORT kpoBaseApp
{
public:

    kpoBaseApp (pcl::OpenNIGrabber& grabber);
    ~kpoBaseApp()
    {
      if (grabber_.isRunning ()) {
        grabber_.stop ();
      }
      saveSettings();
    }


    pcl::OpenNIGrabber& grabber_;
    std::string device_id_;

    pcl::PassThrough<PointType> depth_filter_;

    cv::Mat scene_image_;

    cv::Mat scene_depth_image_;
    int depth_image_threshold_;
    BlobFinder depth_blob_finder;

    CloudPtr scene_cloud_;
    CloudPtr scene_keypoints_;
    NormalCloud::Ptr scene_normals_;
    DescriptorCloud::Ptr scene_descriptors_;
    RFCloud::Ptr scene_refs_;

    pcl::UniformSampling<PointType> uniform_sampling;
    double grabber_downsampling_radius_;
    double keypoint_downsampling_radius_;

    std::vector< boost::shared_ptr<kpoObjectDescription> > models_;
    std::vector< boost::shared_ptr<kpoObjectDescription> > match_queue_;

    QMutex mtx_;

    kpoPclFunctions pcl_functions_;
    bool paused_;
    bool remove_noise_;
    bool estimate_normals_;
    bool compute_descriptors_;
    bool match_models_;

    boost::threadpool::pool model_loading_thread_pool;
    boost::threadpool::pool thread_pool;
    std::vector< boost::shared_ptr<kpoMatcherThread> > matcher_threads;
    unsigned thread_load;
    int model_index;

    void matchesFound(int object_id, Eigen::Vector3f translation, Eigen::Matrix3f rotation);

    double depth_threshold_;

    QString m_sSettingsFile;

    QString models_folder_;

    void loadModelFiles();
    void loadExemplar(string filepath, int object_id);
    void addCurrentObjectToMatchList(int object_id);

    virtual void loadSettings();
    virtual void saveSettings();

    void cloud_callback (const CloudConstPtr& cloud);
    void process_cloud (const CloudConstPtr& cloud);

    void image_callback (const boost::shared_ptr<openni_wrapper::Image> &image);
    void depth_callback (const boost::shared_ptr< openni_wrapper::DepthImage > &depth_image);
    virtual void processDepthBlobs(BlobFinder bf);

    void pause();

    kpoOscSender osc_sender;
    QString osc_sender_ip_;
    int osc_sender_port_;
};

#endif // KPOBASEAPP_H
