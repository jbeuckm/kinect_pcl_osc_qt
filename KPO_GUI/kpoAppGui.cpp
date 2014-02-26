#include <string>

#include "kpoAppGui.h"

// QT4

// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <vtkRenderWindow.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
kpoAppGui::kpoAppGui (pcl::OpenNIGrabber& grabber)
    : vis_ ()
    , vis_timer_ (new QTimer (this))
    , kpoBaseApp(grabber)
//    , device_id_ ()
//    , scene_cloud_()
//    , depth_filter_ ()
    , ui_ (new Ui::KinectPclOsc)
{
    remove_noise_ = false;
    paused_ = false;
    estimate_normals_ = false;
    match_models_ = false;

    ui_->setupUi (this);

    this->setWindowTitle ("Kinect > PCL > OSC");
    vis_.reset (new pcl::visualization::PCLVisualizer ("", false));
    vis_->resetCameraViewpoint ("scene_cloud_");

    vis_->setCameraPosition(0, 0, -1, //position
                            0, 0, 1, //view
                            0, -1, 0); // up

    ui_->qvtk_widget->SetRenderWindow (vis_->getRenderWindow ());
    vis_->setupInteractor (ui_->qvtk_widget->GetInteractor (), ui_->qvtk_widget->GetRenderWindow ());
    vis_->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    ui_->qvtk_widget->update ();



    modelListModel = new QStringListModel(this);
    QStringList list;

    modelListModel->setStringList(list);


    connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot ()));
    vis_timer_->start (5);
}

void kpoAppGui::loadSettings()
{
    kpoBaseApp::loadSettings();

    if (ui_->depthThresholdSlider) {
        ui_->depthThresholdSlider->setValue(depth_threshold_ * 1000);
    }

    ui_->downsamplingRadiusSlider->setValue(keypoint_downsampling_radius_ * 10000);
    ui_->downsamplingRadiusEdit->setText(QString::number(keypoint_downsampling_radius_, 'g', 3));

    ui_->computeNormalsCheckbox->setChecked(estimate_normals_);
    ui_->findSHOTdescriptors->setChecked(compute_descriptors_);
    ui_->matchModelsCheckbox->setChecked(match_models_);

    ui_->modelsFolderEdit->setText(models_folder_);
}



void kpoAppGui::loadModelFiles()
{
/*
    QProgressDialog progress("Loading Model files...", "Cancel", 0, count, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setValue(i);
    progress.setValue(count);
*/
}



void kpoAppGui::loadExemplar(string filename, int object_id)
{
    kpoBaseApp::loadExemplar(filename, object_id);

    addStringToModelsList(filename);
}


void kpoAppGui::addStringToModelsList(string str)
{
    modelListModel->insertRow(modelListModel->rowCount());
    QModelIndex index = modelListModel->index(modelListModel->rowCount()-1);
    modelListModel->setData(index, QString(str.c_str()).section("/",-1,-1) );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
void kpoAppGui::timeoutSlot ()
{
    if (!scene_cloud_ || paused_)
    {
        boost::this_thread::sleep (boost::posix_time::milliseconds (1));
        return;
    }

    updateView();
}


void kpoAppGui::updateView()
{
    {
        QMutexLocker locker (&mtx_);

        vis_->removePointCloud("scene_cloud_", 0);
        vis_->addPointCloud (scene_cloud_, "scene_cloud_");

        if (estimate_normals_ && scene_normals_) {
            vis_->removePointCloud("normals", 0);
            vis_->addPointCloudNormals<PointType, NormalType> (scene_cloud_, scene_normals_, 100, .05, "normals");
            vis_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "normals");
        }

        if (compute_descriptors_ && scene_keypoints_) {
            vis_->removePointCloud("scene_keypoints", 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints_, 0, 0, 255);
            vis_->addPointCloud (scene_keypoints_, scene_keypoints_color_handler, "scene_keypoints");
            vis_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
        }
    }
    //  FPS_CALC ("visualization");
    ui_->qvtk_widget->update ();


    cv::Mat3b src = scene_image_;

    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    for (int y = 0; y < src.rows; ++y) {
            const cv::Vec3b *srcrow = src[y];
            QRgb *destrow = (QRgb*)dest.scanLine(y);
            for (int x = 0; x < src.cols; ++x) {
                    destrow[x] = qRgba(srcrow[x][0], srcrow[x][1], srcrow[x][2], 255);
            }
    }

    scene_qimage_ = dest;

    ui_->sceneImageLabel->setPixmap(QPixmap::fromImage(scene_qimage_));

    ui_->sceneImageLabel->show();
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

    kpoAppGui v (grabber);
    v.show ();
    return (app.exec ());
}



void kpoAppGui::on_pauseCheckBox_toggled(bool checked)
{
    paused_ = checked;
}


void kpoAppGui::on_computeNormalsCheckbox_toggled(bool checked)
{
    estimate_normals_ = checked;
    ui_->findSHOTdescriptors->setEnabled(checked);
}


void kpoAppGui::on_findSHOTdescriptors_toggled(bool checked)
{
    compute_descriptors_ = checked;
}

void kpoAppGui::pause()
{
    paused_ = true;
    ui_->pauseCheckBox->setChecked(true);
}


void kpoAppGui::on_saveDescriptorButton_clicked()
{
    pause();

    std::string objectname =  ui_->objectNameTextInput->text().toStdString();

    std::cout << "will save object " << objectname << std::endl;

    std::replace( objectname.begin(), objectname.end(), ' ', '_');

    QString defaultFilename = QString::fromUtf8(objectname.c_str()) + QString(".descriptor.pcd");

    QString filename = QFileDialog::getSaveFileName(this, tr("Save Descriptor"),
                                                    defaultFilename,
                                                    tr("Descriptors (*.dsc)"));

    if (!filename.isEmpty()) {
//        addCurrentObjectToMatchList(std::stoi(objectname));
    }
}




void kpoAppGui::on_loadDescriptorButton_clicked()
{
    pause();

    QString filename = QFileDialog::getOpenFileName(this, tr("Load Descriptor"),
                                                    "",
                                                    tr("Files (*.descriptor.pcd)"));
/*
    if (!filename.isEmpty()) {
        loadExemplar(filename.toStdString());
    }
*/
}



void kpoAppGui::on_matchModelsCheckbox_toggled(bool checked)
{
    match_models_ = checked;
}


void kpoAppGui::on_presampleRadiusSlider_valueChanged(int value)
{
    grabber_downsampling_radius_ = 0.1f / float(value);
}

void kpoAppGui::on_loadRawCloudButton_clicked()
{
    pause();

    QString filename = QFileDialog::getOpenFileName(this, tr("Load Raw Cloud"),
                                                    "",
                                                    tr("Files (*.pcd)"));

    if (!filename.isEmpty()) {

        CloudPtr cloud(new Cloud());

        pcl::PCDReader reader;
        reader.read<PointType> (filename.toStdString(), *cloud);

        process_cloud(cloud);
        updateView();
    }
}

void kpoAppGui::on_removeNoiseCheckBox_toggled(bool checked)
{
    remove_noise_ = checked;
}

void kpoAppGui::on_setOscTargetButton_clicked()
{
    osc_sender_port_ = ui_->portTextInput->text().toInt();
    osc_sender_ip_ = ui_->ipTextInput->text();

    osc_sender.setNetworkTarget(osc_sender_ip_.toStdString().c_str(), osc_sender_port_);
}




void kpoAppGui::on_downsamplingRadiusSlider_valueChanged(int value)
{
    keypoint_downsampling_radius_ = float(value) / 10000.0f;

    scene_pcl_functions_.setDownsamplingRadius(keypoint_downsampling_radius_);
    ui_->downsamplingRadiusEdit->setText(QString::number(keypoint_downsampling_radius_, 'g', 3));
}

void kpoAppGui::on_browseForModelsButton_clicked()
{
    pause();

    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Models Folder"),
                                                    "/home",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);

    if (!dir.isEmpty()) {
        models_folder_ = dir;
        ui_->modelsFolderEdit->setText(dir);
    }

}

void kpoAppGui::on_depthThresholdSlider_valueChanged(int value)
{
    depth_threshold_ = float(value) / 1000.0f;
}
