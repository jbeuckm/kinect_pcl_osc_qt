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

    depthThreshold = settings.value("depthThreshold", 5).toFloat();
}


void kpoBaseApp::saveSettings()
{
    std::cout << "saveSettings()" << std::endl;

    QSettings settings(m_sSettingsFile, QSettings::NativeFormat);

    float min;
    depth_filter_.getFilterLimits(min, depthThreshold);
    std::cout << "depthThreshold = " << depthThreshold << std::endl;
    settings.setValue("depthThreshold", depthThreshold);

    settings.sync();

//    qDebug() << QApplication::applicationDirPath();
}


void kpoBaseApp::loadDescriptors(string filename)
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

    oscSender.send("/pointcloud/size", filteredCloud->size());

    uniform_sampling.setInputCloud (filteredCloud);
    uniform_sampling.setRadiusSearch (grabber_downsampling_radius_);

    uniform_sampling.compute (sampled_indices);

    pcl::copyPointCloud (*filteredCloud, sampled_indices.points, *scene_cloud_);
*/
    depth_filter_.filter (*scene_cloud_);

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

            scene_rf_.reset(new RFCloud ());
            pcl_functions_.estimateReferenceFrames(scene_cloud_, scene_normals_, scene_keypoints_, scene_rf_);


            if (match_models_) {

                pcl_functions_.setHoughSceneCloud(scene_keypoints_, scene_rf_);

                for (std::vector< boost::shared_ptr<kpoObjectDescription> >::iterator it = models_.begin(); it != models_.end(); ++it) {

                    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

                    pcl_functions_.correlateDescriptors(scene_descriptors_, (*it)->descriptors, model_scene_corrs);

                    std::vector<pcl::Correspondences> clustered;

                    clustered = pcl_functions_.houghCorrespondences((*it)->keypoints, (*it)->reference_frames, model_scene_corrs);

                    if (clustered.size() != 0) {
                        int position = it - models_.begin() ;
                        oscSender.send("/object", position);
                    }

                    std::cout << clustered.size() << " ";
                }

                std::cout << std::endl;

            }
        }
    }

}
