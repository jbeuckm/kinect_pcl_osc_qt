#include "kpoBaseApp.h"

kpoBaseApp::kpoBaseApp (pcl::OpenNIGrabber& grabber)
    : grabber_(grabber)
    , mtx_ ()
{

}


void kpoBaseApp::pause()
{
    paused_ = true;
}


void kpoBaseApp::loadSettings()
{
    std::cout << "loadSettings()" << std::endl;

    QSettings settings(m_sSettingsFile, QSettings::NativeFormat);

    depth_threshold_ = settings.value("depth_threshold_", 5).toFloat();

    keypoint_downsampling_radius_ = settings.value("keypoint_downsampling_radius_", .0075).toFloat();
    pcl_functions_.setDownsamplingRadius(keypoint_downsampling_radius_);

    estimate_normals_ = settings.value("estimate_normals_", true).toBool();
    compute_descriptors_ = settings.value("compute_descriptors_", true).toBool();

    models_folder_ = settings.value("models_folder_", "/home").toString();

    match_models_ = settings.value("match_models_", true).toBool();

    osc_sender_ip_ = settings.value("osc_sender_ip_", "192.168.0.4").toString();
    osc_sender_port_ = settings.value("osc_sender_port_", 12345).toInt();
    oscSender.setNetworkTarget(osc_sender_ip_.toStdString().c_str(), osc_sender_port_);
}


void kpoBaseApp::saveSettings()
{
    std::cout << "saveSettings()" << std::endl;

    QSettings settings(m_sSettingsFile, QSettings::NativeFormat);

    float min;
    depth_filter_.getFilterLimits(min, depth_threshold_);
    std::cout << "depth_threshold_ = " << depth_threshold_ << std::endl;
    settings.setValue("depth_threshold_", depth_threshold_);

    settings.setValue("keypoint_downsampling_radius_", keypoint_downsampling_radius_);

    settings.setValue("models_folder_", models_folder_);
    settings.setValue("match_models_", match_models_);

    settings.setValue("estimate_normals_", estimate_normals_);
    settings.setValue("compute_descriptors_", compute_descriptors_);

    settings.setValue("osc_sender_ip_", osc_sender_ip_);
    settings.setValue("osc_sender_port_", osc_sender_port_);

    settings.sync();
}


// load a raw model cap and process it into a matchable set of keypoints, descriptors
void kpoBaseApp::loadExemplar(string filename)
{
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors_(new pcl::PointCloud<DescriptorType>());

    pcl::PCDReader reader;
    reader.read<DescriptorType> (filename, *model_descriptors_);

//    models_.push_back(model_descriptors_);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////
void kpoBaseApp::cloud_callback (const CloudConstPtr& cloud)
{
    if (paused_) return;

    process_cloud(cloud);
}


void kpoBaseApp::process_cloud (const CloudConstPtr& cloud)
{
    QMutexLocker locker (&mtx_);
    //  FPS_CALC ("computation");

    // Computation goes here
    CloudPtr compressedCloud(new Cloud);

    pcl::PointCloud<int> sampled_indices;

    CloudPtr cleanCloud(new Cloud);
    CloudPtr filteredCloud(new Cloud);
    scene_cloud_.reset (new Cloud);

    if (remove_noise_) {

        pcl_functions_.removeNoise(cloud, cleanCloud);

        depth_filter_.setInputCloud (cleanCloud);
    }
    else {
        depth_filter_.setInputCloud (cloud);
    }

    /*
    depth_filter_.filter (*filteredCloud);


    uniform_sampling.setInputCloud (filteredCloud);
    uniform_sampling.setRadiusSearch (grabber_downsampling_radius_);

    uniform_sampling.compute (sampled_indices);

    pcl::copyPointCloud (*filteredCloud, sampled_indices.points, *scene_cloud_);
*/
    depth_filter_.setFilterLimits(0, depth_threshold_);
    depth_filter_.filter (*scene_cloud_);

    oscSender.send("/pointcloud/size", scene_cloud_->size());

    if (scene_cloud_->size() < 25) return;


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


            if (match_models_) {

                scene_rf_.reset(new RFCloud ());
                pcl_functions_.estimateReferenceFrames(scene_cloud_, scene_normals_, scene_keypoints_, scene_rf_);

                pcl_functions_.setHoughSceneCloud(scene_keypoints_, scene_rf_);

                for (std::vector< boost::shared_ptr<kpoObjectDescription> >::iterator it = models_.begin(); it != models_.end(); ++it) {

                    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

                    pcl_functions_.correlateDescriptors(scene_descriptors_, (*it)->descriptors, model_scene_corrs);

                    std::vector<pcl::Correspondences> clustered_corrs;
                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

                    pcl_functions_.houghCorrespondences((*it)->keypoints, (*it)->reference_frames, model_scene_corrs, clustered_corrs, rototranslations);

                    if (clustered_corrs.size() != 0) {
                        int position = it - models_.begin() ;
                        oscSender.send("/object", position);
                    }

                    std::cout << clustered_corrs.size() << " ";
                }

                std::cout << std::endl;

            }
        }
    }

}
