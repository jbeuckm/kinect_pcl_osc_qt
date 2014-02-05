/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_APPS_OPENNI_PASSTHROUGH_3D_
#define PCL_APPS_OPENNI_PASSTHROUGH_3D_

#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QFileDialog>
#include <QStringListModel>

// PCL
#include "openni_passthrough_qt.h"
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
class KinectPclOsc : public QMainWindow
{
  Q_OBJECT

  public:

    KinectPclOsc (pcl::OpenNIGrabber& grabber);

    ~KinectPclOsc ()
    {
      if (grabber_.isRunning ()) {
        grabber_.stop ();
      }
    }
    
    void cloud_callback (const CloudConstPtr& cloud);
    void pause();

  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
    pcl::OpenNIGrabber& grabber_;
    std::string device_id_;

    CloudPtr scene_cloud_;
    CloudPtr scene_keypoints_;
    NormalCloud::Ptr scene_normals_;
    DescriptorCloud::Ptr scene_descriptors_;
    RFCloud::Ptr scene_rf_;

    std::vector< boost::shared_ptr<kpoObjectDescription> > models_;

    pcl::PassThrough<PointType> depth_filter_;

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
    bool show_normals_;
    bool compute_descriptors_;
    bool match_models_;

    void saveDescriptors(string filename, const DescriptorCloud::Ptr &descriptors);
    void loadDescriptors(string filename);

  public slots:

    void adjustPassThroughValues (int new_value);

  private slots:
    void timeoutSlot ();
    

    void on_computeNormalsCheckbox_toggled(bool checked);

    void on_pauseCheckBox_toggled(bool checked);

    void on_findSHOTdescriptors_toggled(bool checked);

    void on_saveDescriptorButton_clicked();

    void on_matchModelsCheckbox_toggled(bool checked);

    void on_loadDescriptorButton_clicked();

    void on_presampleRadiusSlider_valueChanged(int value);

signals:
    void valueChanged (int new_value);
};

#endif    // PCL_APPS_OPENNI_PASSTHROUGH_3D_
