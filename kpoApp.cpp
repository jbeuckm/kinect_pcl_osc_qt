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

#include "kpoApp.h"
// QT4
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QFileDialog>
// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <vtkRenderWindow.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
KinectPclOsc::KinectPclOsc (pcl::OpenNIGrabber& grabber)
  : vis_ ()
  , grabber_(grabber)
  , device_id_ ()
  , cloud_pass_()
  , depth_filter_ ()
  , mtx_ ()
  , ui_ (new Ui::KinectPclOsc)
  , vis_timer_ (new QTimer (this))
{
  paused_ = false;
  show_normals_ = false;

  // Create a timer and fire it up every 5ms
  vis_timer_->start (5);

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot ()));

  ui_->setupUi (this);

  this->setWindowTitle ("Kinect > PCL > OSC");
  vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
  ui_->qvtk_widget->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (ui_->qvtk_widget->GetInteractor (), ui_->qvtk_widget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  ui_->qvtk_widget->update (); 

  // Start the OpenNI data acquision
  boost::function<void (const CloudConstPtr&)> f = boost::bind (&KinectPclOsc::cloud_callback, this, _1);
  boost::signals2::connection c = grabber_.registerCallback (f);

  grabber_.start ();

  // Set defaults
  depth_filter_.setFilterFieldName ("z");
  depth_filter_.setFilterLimits (0.5, 5.0);
  
  ui_->fieldValueSlider->setRange (5, 50);
  ui_->fieldValueSlider->setValue (50);

  connect (ui_->fieldValueSlider, SIGNAL (valueChanged (int)), this, SLOT (adjustPassThroughValues (int)));
  connect (ui_->zOffsetSlider, SIGNAL (valueChanged (int)), this, SLOT (adjustZoffset (int)));
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
void KinectPclOsc::cloud_callback (const CloudConstPtr& cloud)
{
    if (paused_) return;

  QMutexLocker locker (&mtx_);  
  FPS_CALC ("computation");

  // Computation goes here
  CloudPtr compressedCloud(new Cloud);

  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompression(pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
  std::stringstream compressedData;

  // Compress the cloud (you would save the stream to disk).
  octreeCompression.encodePointCloud(cloud, compressedData);

  // Decompress the cloud.
  octreeCompression.decodePointCloud(compressedData, compressedCloud);


  cloud_pass_.reset (new Cloud);
  depth_filter_.setInputCloud (compressedCloud);
  depth_filter_.filter (*cloud_pass_);

  if (show_normals_) {

      normals_.reset (new pcl::PointCloud<NormalType>);
      pcl_functions_.estimateNormals(cloud_pass_, normals_);

      if (compute_descriptors_) {

            descriptors_.reset(new pcl::PointCloud<DescriptorType>);
          pcl_functions_.computeShotDescriptors(cloud_pass_, normals_, descriptors_);
    }
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void KinectPclOsc::timeoutSlot ()
{
  if (!cloud_pass_ || paused_)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (1));
    return;
  }

  {
      QMutexLocker locker (&mtx_);

/*
      if (show_normals_ && normals_) {

          if (!vis_->removePointCloud("cloud_pass", 0)) {

              vis_->addPointCloudNormals (cloud_pass_, normals_, 100, .02, "cloud_pass");

              vis_->resetCameraViewpoint ("cloud_pass");

              vis_->setCameraPosition(0, 0, -1, //position
                                      0, 0, 1, //view
                                      0, -1, 0); // up
          }
          {
              vis_->addPointCloudNormals (cloud_pass_, normals_, 100, .02, "cloud_pass");
          }

      }
      else {
*/
          if (!vis_->updatePointCloud (cloud_pass_, "cloud_pass"))
          {
              vis_->addPointCloud (cloud_pass_, "cloud_pass");
              vis_->resetCameraViewpoint ("cloud_pass");

              vis_->setCameraPosition(0, 0, -1, //position
                                      0, 0, 1, //view
                                      0, -1, 0); // up
          }
//      }
  }
  FPS_CALC ("visualization");
  ui_->qvtk_widget->update ();
}



int main (int argc, char ** argv)
{
  // Initialize QT
  QApplication app (argc, argv); 

  // Open the first available camera
  pcl::OpenNIGrabber grabber ("#1");
  // Check if an RGB stream is provided
  if (!grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud> ())
  {
    PCL_ERROR ("Device #1 does not provide an RGB stream!\n");
    return (-1);
  }

  KinectPclOsc v (grabber);
  v.show ();
  return (app.exec ());
}



void KinectPclOsc::on_computeDescriptorsButton_clicked()
{
    if (!cloud_pass_)
    {
        std::cout << "cloud_pass_ looks empty." << std::endl;
      return;
    }
    if (!normals_)
    {
       std::cout << "normals_ looks empty." << std::endl;
      return;
    }

    CloudPtr temp_cloud;
    pcl::PointCloud<NormalType>::Ptr temp_normals_cloud;
    {
      QMutexLocker locker (&mtx_);

      temp_cloud.swap (cloud_pass_);

      temp_normals_cloud.swap (normals_);
    }

//    pcl_functions_.computeShotDescriptors(temp_cloud, temp_normals_cloud);
}

void KinectPclOsc::on_computeNormalsCheckbox_toggled(bool checked)
{
    show_normals_ = checked;
}

void KinectPclOsc::on_pauseCheckBox_toggled(bool checked)
{
    paused_ = checked;
}

void KinectPclOsc::on_findSHOTdescriptors_toggled(bool checked)
{
    compute_descriptors_ = checked;
}


void KinectPclOsc::on_saveDescriptorButton_clicked()
{
    paused_ = true;

    QString objectName =  ui_->objectNameTextInput->text();

    std::cout << "will save object " << objectName.toStdString() << std::endl;

    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Descriptor"),
                                "descriptor.dsc",
                                tr("Descriptors (*.dsc)"));

    if (!fileName.isEmpty()) {
        pcl::PCDWriter writer;
        writer.write<DescriptorType> (fileName.toStdString(), *descriptors_, false);
    }
}
