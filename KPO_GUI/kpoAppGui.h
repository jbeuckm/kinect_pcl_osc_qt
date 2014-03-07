
#ifndef PCL_APPS_OPENNI_PASSTHROUGH_3D_
#define PCL_APPS_OPENNI_PASSTHROUGH_3D_

// QT4
#include <QMainWindow>
#include <QMutexLocker>
#include <QEvent>
#include <QFileDialog>
#include <QStringListModel>

// PCL
#include <pcl/visualization/pcl_visualizer.h>


#ifdef __GNUC__
#pragma GCC system_header
#endif

#include <ui_kpoAppGui.h>
#include "kpoBaseApp.h"

#include "BlobRenderer.h"

using namespace cv;


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

  protected:
    QTimer *vis_timer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;

    void loadSettings();
    void pause();

  private:
    Ui::KinectPclOsc *ui_;
    QImage scene_qimage_;

    QStringListModel *modelListModel;

    void drawRgbImage();
    void drawDepthImage();

    BlobRenderer *blob_renderer;

    QImage MatToQImage(const Mat& mat);


  public slots:


  private slots:
    void timeoutSlot ();
    void updateView();

    void on_saveCloudButton_clicked();
    void on_loadRawCloudButton_clicked();
    void on_browseForModelsButton_clicked();

    void on_pauseCheckBox_toggled(bool checked);
    void on_processSceneCheckBox_toggled(bool checked);
    void on_matchModelsCheckbox_toggled(bool checked);

    void on_setOscTargetButton_clicked();

    void on_downsamplingRadiusSlider_valueChanged(int value);


    void on_depthThresholdSlider_valueChanged(int value);
    void on_depthImageThresholdSlider_valueChanged(int value);

    void on_contourSelected(Contour contour);


    void on_browseContoursFolderButton_clicked();

signals:
    void valueChanged (int new_value);
};

#endif    // PCL_APPS_OPENNI_PASSTHROUGH_3D_
