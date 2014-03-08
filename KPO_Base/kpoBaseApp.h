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

#include "kpoAnalyzerThread.h"
#include "kpo_types.h"
#include "kpoOscSender.h"
#include "kpoMatcherThread.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "KPO_Base_global.h"
#include "BlobFinder.h"

#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <iostream>
#include <sstream>

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

    QString contours_folder_;
    void loadContourFiles();
    kpoObjectContour load_contour_file(string file_path);
    void save_contour_file(kpoObjectContour object_contour, string file_path);

    cv::Mat scene_depth_image_;
    int depth_image_threshold_;
    BlobFinder depth_blob_finder;

    std::vector< kpoObjectContour > contour_objects_;
    void findMatchingContours(Contour scene_contour);

    CloudPtr scene_cloud_;
    CloudPtr scene_keypoints_;
    NormalCloud::Ptr scene_normals_;
    DescriptorCloud::Ptr scene_descriptors_;
    RFCloud::Ptr scene_refs_;

    pcl::UniformSampling<PointType> uniform_sampling;
    double grabber_downsampling_radius_;
    double keypoint_downsampling_radius_;

    QString m_sSettingsFile;

    QString models_folder_;

    void loadModelFiles();
    void load_model_cloud(string filepath, int object_id);
//    boost::threadpool::pool analyzer_thread_pool;
    void modelCloudAnalyzed(kpoCloudDescription od);

    virtual void loadSettings();
    virtual void saveSettings();

    QMutex mtx_;

    void sceneCloudAnalyzed(kpoCloudDescription od);

    bool paused_;
    bool process_scene_;
    bool match_models_;

    boost::threadpool::pool thread_pool;
    std::vector< boost::shared_ptr<kpoMatcherThread> > matcher_threads;
    unsigned thread_load;
    int model_index;

    void matchesFound(int object_id, Eigen::Vector3f translation, Eigen::Matrix3f rotation);


    double depth_threshold_;

    void cloud_callback (const CloudConstPtr& cloud);
    void process_cloud (const CloudConstPtr& cloud);
    boost::thread *analyze_thread;
    int analyze_thread_count;

    void image_callback (const boost::shared_ptr<openni_wrapper::Image> &image);
    void depth_callback (const boost::shared_ptr< openni_wrapper::DepthImage > &depth_image);
    virtual void processDepthBlobs(BlobFinder bf);

    void pause();

    boost::shared_ptr< kpoOscSender > osc_sender;
    QString osc_sender_ip_;
    int osc_sender_port_;



    void openniImage2opencvMat(const XnRGB24Pixel* pImageMap, cv::Mat& cv_image, int rows, int cols)
    {
      int sizes[2] = {rows, cols};
      cv_image = cv::Mat(2, sizes, CV_8UC3, (void*) pImageMap);
    }
    void openniImage2opencvMat(const XnDepthPixel* pDepthMap, cv::Mat& cv_depth, int rows, int cols)
    {
      int sizes[2] = {rows, cols};
      cv_depth = cv::Mat(2, sizes, CV_16UC1, (void*) pDepthMap);
    }

    Cloud::Ptr bb_hull_cloud_;
    Cloud::Ptr boundingbox_ptr;
    std::vector<pcl::Vertices> bb_polygons;
    void build_bounding_box();
    void crop_bounding_box_(const CloudConstPtr &cloud, CloudPtr &output_cloud);
};

#endif // KPOBASEAPP_H
