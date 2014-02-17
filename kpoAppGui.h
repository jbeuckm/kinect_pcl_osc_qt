
#ifndef PCL_APPS_OPENNI_PASSTHROUGH_3D_
#define PCL_APPS_OPENNI_PASSTHROUGH_3D_

// QT4
#include <QMainWindow>
#include <QMutex>
#include <QTimer>
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QFileDialog>
#include <QStringListModel>

// PCL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "kpoBaseApp.h"
#include "kpoPclFunctions.h"
#include "kpoObjectDescription.h"
#include "kpo_types.h"
#include "kpoOscSender.h"

// from openni_passthrough_qt
#ifdef __GNUC__
#pragma GCC system_header
#endif

#include <ui_kpoApp.h>
#include "kpoAppGui.h"



// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

namespace Ui
{
  class KinectPclOsc;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class kpoAppGui : public QMainWindow, public kpoBaseApp
{
  Q_OBJECT

  public:

    kpoAppGui (pcl::OpenNIGrabber& grabber);

    ~kpoAppGui ()
    {
      if (grabber_.isRunning ()) {
        grabber_.stop ();
      }
      saveSettings();
    }
    
    void cloud_callback (const CloudConstPtr& cloud);
    void process_cloud (const CloudConstPtr& cloud);

    void pause();

    kpoOscSender oscSender;

  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    pcl::UniformSampling<PointType> uniform_sampling;
    float grabber_downsampling_radius_;

  private:
    Ui::KinectPclOsc *ui_;
    QStringListModel *modelListModel;
    void addStringToModelsList(string str);

    QMutex mtx_;
    QTimer *vis_timer_;

    kpoPclFunctions pcl_functions_;
    bool paused_;
    bool remove_noise_;
    bool estimate_normals_;
    bool compute_descriptors_;
    bool match_models_;

    void saveDescriptors(string filename, const DescriptorCloud::Ptr &descriptors);
    void loadDescriptors(string filename);

    QString m_sSettingsFile;
    void loadSettings();
    void saveSettings();

    void setDepthFromSliderValue(int val);


  public slots:

    void adjustPassThroughValues (int new_value);

  private slots:
    void timeoutSlot ();
    void updateView();
    

    void on_computeNormalsCheckbox_toggled(bool checked);

    void on_pauseCheckBox_toggled(bool checked);

    void on_findSHOTdescriptors_toggled(bool checked);

    void on_saveDescriptorButton_clicked();

    void on_matchModelsCheckbox_toggled(bool checked);

    void on_loadDescriptorButton_clicked();

    void on_presampleRadiusSlider_valueChanged(int value);

    void on_loadRawCloudButton_clicked();

    void on_removeNoiseCheckBox_toggled(bool checked);

    void on_setOscTargetButton_clicked();

    void on_depthThresholdlSlider_valueChanged(int value);

signals:
    void valueChanged (int new_value);
};

#endif    // PCL_APPS_OPENNI_PASSTHROUGH_3D_
