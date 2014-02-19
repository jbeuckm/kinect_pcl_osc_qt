#include "kpoBaseApp.h"

kpoBaseApp::kpoBaseApp (pcl::OpenNIGrabber& grabber)
    : grabber_(grabber)
    , pcl_functions_( kpoPclFunctions(.01f) )
    , mtx_ ()
{
    // Start the OpenNI data acquision
    boost::function<void (const CloudConstPtr&)> f = boost::bind (&kpoBaseApp::cloud_callback, this, _1);
    boost::signals2::connection c = grabber_.registerCallback (f);

    // Set defaults
    depth_filter_.setFilterFieldName ("z");
    depth_filter_.setFilterLimits (0.5, 5.0);

    grabber_downsampling_radius_ = .005f;

    m_sSettingsFile = QApplication::applicationDirPath() + "/settings.ini";

    std::cout <<  m_sSettingsFile.toStdString() << endl;
    loadSettings();

    grabber_.start ();
}


void kpoBaseApp::pause()
{
    paused_ = true;
}


void kpoBaseApp::loadSettings()
{
    std::cout << "loadSettings()" << std::endl;

    QSettings settings(m_sSettingsFile, QSettings::NativeFormat);

    depth_threshold_ = settings.value("depth_threshold_", 5).toDouble();

    keypoint_downsampling_radius_ = settings.value("keypoint_downsampling_radius_", .0075).toDouble();
    pcl_functions_.setDownsamplingRadius(keypoint_downsampling_radius_);

    models_folder_ = settings.value("models_folder_", "/home").toString();

    osc_sender_ip_ = settings.value("osc_sender_ip_", "192.168.0.4").toString();
    osc_sender_port_ = settings.value("osc_sender_port_", 12345).toInt();
    oscSender.setNetworkTarget(osc_sender_ip_.toStdString().c_str(), osc_sender_port_);

    match_models_ = true;
    estimate_normals_ = true;
    compute_descriptors_ = true;
    loadModelFiles();

    estimate_normals_ = settings.value("estimate_normals_", true).toBool();
    compute_descriptors_ = settings.value("compute_descriptors_", true).toBool();

}


void kpoBaseApp::loadModelFiles()
{
    std::cout << "kpoBaseApp::loadModelFiles() with " << models_folder_.toStdString() << std::endl;

    QStringList nameFilter("*.pcd");
    QDir directory(models_folder_);
    QStringList model_files = directory.entryList(nameFilter);

    int count = model_files.length();
    std::cout << "will load " << count << " model files." << std::endl;

    for (int i=0; i<count; i++) {

        QString qs_filename = model_files[i];
        string filename = qs_filename.toStdString();

        std::cout << "reading " << filename << std::endl;

        int object_id = qs_filename.replace(QRegExp("[a-z]*.pcd"), "").toInt();

        std::cout << "model has id " << object_id << std::endl;

        loadExemplar(models_folder_.toStdString() + "/" + filename, object_id);
    }

}



// load a raw model cap and process it into a matchable set of keypoints, descriptors
void kpoBaseApp::loadExemplar(string filepath, int object_id)
{
    pcl::PointCloud<PointType>::Ptr model_(new pcl::PointCloud<PointType>());

    pcl::PCDReader reader;
    reader.read<PointType> (filepath, *model_);

    process_cloud(model_);

    if (scene_cloud_->size() != 0) {

        addCurrentObjectToMatchList(object_id);

    }
}

void kpoBaseApp::saveSettings()
{
    std::cout << "saveSettings()" << std::endl;

    QSettings settings(m_sSettingsFile, QSettings::NativeFormat);

    std::cout << "depth_threshold_ = " << depth_threshold_ << std::endl;
    settings.setValue("depth_threshold_", depth_threshold_);

    settings.setValue("keypoint_downsampling_radius_", keypoint_downsampling_radius_);

    settings.setValue("models_folder_", models_folder_);

    settings.setValue("estimate_normals_", estimate_normals_);
    settings.setValue("compute_descriptors_", compute_descriptors_);

    settings.setValue("osc_sender_ip_", osc_sender_ip_);
    settings.setValue("osc_sender_port_", osc_sender_port_);

    settings.sync();
}


// Save the currently processed cloud/keypoints/descriptors tpo be matched
void kpoBaseApp::addCurrentObjectToMatchList(int object_id)
{
    std::cout << "saving cloud with " << scene_cloud_->size() << " points" << std::endl;
    std::cout << "saving keypoints with " << scene_keypoints_->size() << " points" << std::endl;
    std::cout << "saving normals with " << scene_normals_->size() << " points" << std::endl;
    std::cout << "saving descriptors with " << scene_descriptors_->size() << " points" << std::endl;
    std::cout << "saving ref frames with " << scene_rf_->size() << " points" << std::endl;

    boost::shared_ptr<kpoObjectDescription> object_desc(new kpoObjectDescription(scene_cloud_, scene_keypoints_, scene_normals_, scene_descriptors_, scene_rf_));

    object_desc->object_id = object_id;

    models_.push_back(object_desc);

//    addStringToModelsList(filename);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
void kpoBaseApp::cloud_callback (const CloudConstPtr& cloud)
{
    if (paused_) return;

    process_cloud(cloud);
}


void kpoBaseApp::process_cloud (const CloudConstPtr& cloud)
{
    std::cout << "process_cloud() with " << cloud->size() << std::endl;
    QMutexLocker locker (&mtx_);
    //  FPS_CALC ("computation");

    // Computation goes here
    CloudPtr compressedCloud(new Cloud);

    pcl::PointCloud<int> sampled_indices;

    CloudPtr cleanCloud(new Cloud);
    CloudPtr filteredCloud(new Cloud);
    scene_cloud_.reset (new Cloud);
/*
    if (remove_noise_) {

        pcl_functions_.removeNoise(cloud, cleanCloud);

        depth_filter_.setInputCloud (cleanCloud);
    }
    else {
*/
        depth_filter_.setInputCloud (cloud);
//    }

    /*
    depth_filter_.filter (*filteredCloud);


    uniform_sampling.setInputCloud (filteredCloud);
    uniform_sampling.setRadiusSearch (grabber_downsampling_radius_);

    uniform_sampling.compute (sampled_indices);

    pcl::copyPointCloud (*filteredCloud, sampled_indices.points, *scene_cloud_);
*/
    std::cout << "depth_threshold_ " << depth_threshold_ << std::endl;

    depth_filter_.setFilterLimits(0, depth_threshold_);
    depth_filter_.filter (*scene_cloud_);

    oscSender.send("/pointcloud/size", scene_cloud_->size());

    if (scene_cloud_->size() < 25) {
        std::cout << "cloud too small" << std::endl;
        return;
    }


    if (estimate_normals_) {

        scene_normals_.reset (new NormalCloud ());
        pcl_functions_.estimateNormals(scene_cloud_, scene_normals_);

        if (compute_descriptors_) {

            scene_keypoints_.reset(new Cloud ());
            pcl_functions_.downSample(scene_cloud_, scene_keypoints_);

            scene_descriptors_.reset(new DescriptorCloud ());
            pcl_functions_.computeShotDescriptors(scene_cloud_, scene_keypoints_, scene_normals_, scene_descriptors_);


//            double res = pcl_functions_.computeCloudResolution(scene_cloud_);
//            std::cout << "resolution = " << res << std::endl;

            scene_rf_.reset(new RFCloud ());
            pcl_functions_.estimateReferenceFrames(scene_cloud_, scene_normals_, scene_keypoints_, scene_rf_);


            if (match_models_) {

                pcl_functions_.setHoughSceneCloud(scene_keypoints_, scene_rf_);

                for (std::vector< boost::shared_ptr<kpoObjectDescription> >::iterator it = models_.begin(); it != models_.end(); ++it) {

                    int count = matchModel(*it);

                    if (count != 0) {
                        int position = it - models_.begin() ;
                        oscSender.send("/object", (*it)->object_id);
                    }
                    std::cout << count << " ";

                }

                std::cout << std::endl;

            }
        }
    }

}


int kpoBaseApp::matchModel(boost::shared_ptr<kpoObjectDescription> model_)
{

    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl_functions_.correlateDescriptors(scene_descriptors_, model_->descriptors, model_scene_corrs);
    std::cout << "msc" << model_scene_corrs->size() << "/" << model_->descriptors->size() << " ";

    if (model_scene_corrs->size() < 10) {
        return 0;
    }

    std::vector<pcl::Correspondences> clustered_corrs;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

    pcl_functions_.houghCorrespondences(model_->keypoints, model_->reference_frames, model_scene_corrs, clustered_corrs, rototranslations);

    return clustered_corrs.size();
}
