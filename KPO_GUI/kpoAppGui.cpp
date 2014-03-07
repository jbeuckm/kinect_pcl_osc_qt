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
    , ui_ (new Ui::KinectPclOsc)
{
    paused_ = false;
    process_scene_ = false;
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

    QRect blobsSize(ui_->blobs->geometry().topLeft(), ui_->blobs->geometry().bottomRight());

    blob_renderer = new BlobRenderer(ui_->centralwidget);

    blob_renderer->setGeometry(blobsSize);
    blob_renderer->show();

    connect(blob_renderer, SIGNAL(contourSelected(Contour)), this, SLOT(on_contourSelected(Contour)));

    modelListModel = new QStringListModel(this);
    QStringList list;

    modelListModel->setStringList(list);


    connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot ()));
    vis_timer_->start (5);
}

void kpoAppGui::loadSettings()
{
    kpoBaseApp::loadSettings();

    if (ui_->depthImageThresholdSlider) {
        ui_->depthImageThresholdSlider->setValue(depth_image_threshold_);
    }
/*
    if (ui_->depthThresholdSlider) {
        ui_->depthThresholdSlider->setValue(depth_threshold_ * 1000);
    }

    ui_->downsamplingRadiusSlider->setValue(keypoint_downsampling_radius_ * 10000);
*/
    ui_->downsamplingRadiusEdit->setText(QString::number(keypoint_downsampling_radius_, 'g', 3));

    ui_->processSceneCheckBox->setChecked(process_scene_);
    ui_->matchModelsCheckbox->setChecked(match_models_);

    ui_->modelsFolderEdit->setText(models_folder_);


     ui_->portTextInput->setText(QString::number(osc_sender_port_));
     ui_->ipTextInput->setText(osc_sender_ip_);
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


        vis_->removePointCloud("bounding_box_", 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> bounding_box_color_handler (bb_hull_cloud_, 0, 0, 255);
        vis_->addPointCloud (bb_hull_cloud_, bounding_box_color_handler, "bounding_box_");
        vis_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "bounding_box_");


        vis_->removePointCloud("scene_keypoints", 0);
        if (process_scene_ && scene_keypoints_) {
            pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints_, 0, 0, 255);
            vis_->addPointCloud (scene_keypoints_, scene_keypoints_color_handler, "scene_keypoints");
            vis_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_keypoints");
        }

        vis_->removePointCloud("normals", 0);
        if (process_scene_ && scene_normals_) {
            vis_->addPointCloudNormals<PointType, NormalType> (scene_cloud_, scene_normals_, 100, .05, "normals");
            vis_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "normals");
        }

        drawRgbImage();

        blob_renderer->resetPaths();
        for (int i=0; i<depth_blob_finder.contours.size(); i++) {
            if (depth_blob_finder.radius[i] > 20) {
                blob_renderer->addPath(depth_blob_finder.contours[i]);
            }
        }
    }
    //  FPS_CALC ("visualization");
    ui_->qvtk_widget->update ();

    blob_renderer->update();
}

void kpoAppGui::drawDepthImage()
{
    cv::Mat resized;
    cv::Mat src;
    scene_depth_image_.convertTo(src, CV_8UC3);

    scene_qimage_ = MatToQImage(resized);

}

void kpoAppGui::drawRgbImage()
{
    cv::Mat3b resized = scene_image_;

    QImage dest(resized.cols, resized.rows, QImage::Format_ARGB32);
    for (int y = 0; y < resized.rows; ++y) {
            const cv::Vec3b *srcrow = resized[y];
            QRgb *destrow = (QRgb*)dest.scanLine(y);
            for (int x = 0; x < resized.cols; ++x) {
                    destrow[x] = qRgba(srcrow[x][0], srcrow[x][1], srcrow[x][2], 255);
            }
    }

    scene_qimage_ = dest;

    blob_renderer->updateBackgroundImage(dest);
}



int main (int argc, char ** argv)
{
    QApplication app (argc, argv);

    pcl::OpenNIGrabber grabber ("#1");

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


void kpoAppGui::pause()
{
    paused_ = true;
    ui_->pauseCheckBox->setChecked(true);
}


void kpoAppGui::on_saveCloudButton_clicked()
{
    pause();

    std::string objectname =  ui_->objectNameTextInput->text().toStdString();

    std::cout << "will save object " << objectname << std::endl;

    std::replace( objectname.begin(), objectname.end(), ' ', '_');

    QString defaultFilename = QString::fromUtf8(objectname.c_str()) + QString(".pcd");

    QString filename = QFileDialog::getSaveFileName(this, tr("Save Pointcloud"),
                                                    defaultFilename,
                                                    tr("Pointcloud (*.pcd)"));


    if (!filename.isEmpty()) {
        pcl::PCDWriter writer;
        writer.writeASCII(filename.toStdString(), *scene_cloud_);
    }
}




void kpoAppGui::on_matchModelsCheckbox_toggled(bool checked)
{
    match_models_ = checked;
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


void kpoAppGui::on_setOscTargetButton_clicked()
{
    osc_sender_port_ = ui_->portTextInput->text().toInt();
    osc_sender_ip_ = ui_->ipTextInput->text();

    osc_sender->setNetworkTarget(osc_sender_ip_.toStdString().c_str(), osc_sender_port_);
}




void kpoAppGui::on_downsamplingRadiusSlider_valueChanged(int value)
{
    keypoint_downsampling_radius_ = float(value) / 10000.0f;

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

void kpoAppGui::on_depthImageThresholdSlider_valueChanged(int value)
{
    depth_image_threshold_ = value;
    std::cout << "depth_image_threshold_ = " << depth_image_threshold_ << std::endl;
}

void kpoAppGui::on_contourSelected(Contour contour)
{
    pause();

    std::cout << "selected contour with " << contour.size() << std::endl;

    std::string objectname =  ui_->contourObjectIdTextInput->text().toStdString();

    QString qstr(objectname.c_str());
    int object_id = qstr.replace(QRegExp("[a-z]"), "").toInt();
    std::cout << "object id = " << object_id << std::endl;

    kpoObjectContour object;
    object.contour = contour;
    object.object_id = object_id;

    contour_objects_.push_back(object);

    string path = contours_folder_.toStdString() +"/" + objectname + ".path";
    std::cout << "will save file " << path << std::endl;

    save_contour_file(object, path);
}


QImage kpoAppGui::MatToQImage(const Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type()==CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
            colorTable.push_back(qRgb(i,i,i));
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img;
    }
    // 8-bits unsigned, NO. OF CHANNELS=3
    if(mat.type()==CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
} // MatToQImage()

void kpoAppGui::on_processSceneCheckBox_toggled(bool checked)
{
    process_scene_ = checked;
}

void kpoAppGui::on_browseContoursFolderButton_clicked()
{
    pause();

    QString dir = QFileDialog::getExistingDirectory(this, tr("Select Contours Folder"),
                                                    "/home",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) {
        contours_folder_ = dir;
        ui_->contoursFolderTextInput->setText(dir);
    }
}
