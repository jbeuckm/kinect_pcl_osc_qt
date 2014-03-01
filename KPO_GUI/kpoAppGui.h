
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

    void processDepthBlobs(BlobFinder bf);



  protected:
    QTimer *vis_timer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;

    void loadSettings();
    void pause();

  private:
    Ui::KinectPclOsc *ui_;
    QImage scene_qimage_;

    void loadExemplar(string filename, int object_id);

    QStringListModel *modelListModel;
    void addStringToModelsList(string str);

    void drawRgbImage();
    void drawDepthImage();

    BlobRenderer *blob_renderer;

    QImage MatToQImage(const Mat& mat);

  public slots:


  private slots:
    void timeoutSlot ();
    void updateView();

    void on_pauseCheckBox_toggled(bool checked);

    void on_saveDescriptorButton_clicked();
    void on_loadDescriptorButton_clicked();
    void on_loadRawCloudButton_clicked();

    void on_removeNoiseCheckBox_toggled(bool checked);
    void on_presampleRadiusSlider_valueChanged(int value);


    void on_computeNormalsCheckbox_toggled(bool checked);
    void on_findSHOTdescriptors_toggled(bool checked);

    void on_matchModelsCheckbox_toggled(bool checked);

    void on_setOscTargetButton_clicked();

    void on_downsamplingRadiusSlider_valueChanged(int value);

    void on_browseForModelsButton_clicked();

    void on_depthThresholdSlider_valueChanged(int value);

    void on_depthImageThresholdSlider_valueChanged(int value);

signals:
    void valueChanged (int new_value);
};

#endif    // PCL_APPS_OPENNI_PASSTHROUGH_3D_
