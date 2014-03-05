#include "kpoBaseApp.h"

#include "BlobFinder.h"

#define THREADED_ANALYSIS false


kpoBaseApp::kpoBaseApp (pcl::OpenNIGrabber& grabber)
    : grabber_(grabber)
    , mtx_ ()
    , matcher_thread_pool(8)
    , analyzer_thread_pool(8)
    , osc_sender (new kpoOscSender())
{
    // Start the OpenNI data acquision
    boost::function<void (const CloudConstPtr&)> f = boost::bind (&kpoBaseApp::cloud_callback, this, _1);
    boost::signals2::connection c = grabber_.registerCallback (f);

    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> ic = boost::bind (&kpoBaseApp::image_callback, this, _1);
    boost::signals2::connection d = grabber_.registerCallback (ic);

    boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> dc = boost::bind (&kpoBaseApp::depth_callback, this, _1);
    boost::signals2::connection e = grabber_.registerCallback (dc);


    // Set defaults
    depth_filter_.setFilterFieldName ("z");
    depth_filter_.setFilterLimits (0.5, 5.0);

    depth_image_threshold_ = 128;

    grabber_downsampling_radius_ = .005f;

    QString pwd("");
    char * PWD;
    PWD = getenv ("PWD");
    pwd.append(PWD);
    m_sSettingsFile = QString::fromUtf8( (pwd.toStdString() + "settings.ini").c_str() );

    model_index = 0;

    std::cout <<  m_sSettingsFile.toStdString() << endl;
    loadSettings();

    thread_load = 12;

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

    depth_threshold_ = settings.value("depth_threshold_", 1.031).toDouble();

    keypoint_downsampling_radius_ = settings.value("keypoint_downsampling_radius_", .0075).toDouble();

    models_folder_ = settings.value("models_folder_", "/myshare/pointclouds/objects").toString();
    contours_folder_ = settings.value("contours_folder_", "/myshare/contours").toString();

    osc_sender_ip_ = settings.value("osc_sender_ip_", "192.168.0.48").toString();
    osc_sender_port_ = settings.value("osc_sender_port_", 7000).toInt();
    std::cout << "loaded ip " << osc_sender_ip_.toStdString() << ":" << osc_sender_port_ << std::endl;
    osc_sender->setNetworkTarget(osc_sender_ip_.toStdString().c_str(), osc_sender_port_);

    process_scene_ = true;
    match_models_ = false;

    loadModelFiles();

    process_scene_ = settings.value("process_scene_", false).toBool();
    match_models_ = settings.value("match_models_", false).toBool();

}
void kpoBaseApp::saveSettings()
{
    std::cout << "saveSettings()" << std::endl;

    QSettings settings(m_sSettingsFile, QSettings::NativeFormat);

    std::cout << "depth_threshold_ = " << depth_threshold_ << std::endl;
    settings.setValue("depth_threshold_", depth_threshold_);

    settings.setValue("keypoint_downsampling_radius_", keypoint_downsampling_radius_);

    settings.setValue("models_folder_", models_folder_);
    settings.setValue("contours_folder_", contours_folder_);

    settings.setValue("process_scene_", process_scene_);
    settings.setValue("match_models_", match_models_);

    settings.setValue("osc_sender_ip_", osc_sender_ip_);
    settings.setValue("osc_sender_port_", osc_sender_port_);

    settings.sync();
}


void kpoBaseApp::loadModelFiles()
{
    std::cout << "kpoBaseApp::loadModelFiles() with " << models_folder_.toStdString() << std::endl;

    QStringList nameFilter("*.pcd");
    QDir directory(models_folder_);
    QStringList model_files = directory.entryList(nameFilter);

    int count = model_files.length();
    std::cout << "will load " << count << " model files." << std::endl;
    if (count < thread_load) {
        thread_load = count;
    }

    for (int i=0; i<count; i++) {

        QString qs_filename = model_files[i];
        string filename = qs_filename.toStdString();

        std::cout << "reading " << filename << std::endl;

        int object_id = qs_filename.replace(QRegExp("[a-z]*.pcd"), "").toInt();

        std::cout << "object_id " << object_id << std::endl;

        load_model_cloud(models_folder_.toStdString() + "/" + filename, object_id);
    }

    if (THREADED_ANALYSIS) {
        analyzer_thread_pool.wait();
    }
}



// load a raw model capture and process it into a matchable set of keypoints, descriptors
void kpoBaseApp::load_model_cloud(string filename, int object_id)
{
    pcl::PointCloud<PointType>::Ptr model_(new pcl::PointCloud<PointType>());

    pcl::PCDReader reader;
    reader.read<PointType> (filename, *model_);

    if (model_->size() != 0) {

        if (THREADED_ANALYSIS) {

            kpoAnalyzerThread *kat = new kpoAnalyzerThread(keypoint_downsampling_radius_);
            boost::shared_ptr<kpoAnalyzerThread> analyzer(kat);

            analyzer->copyInputCloud(model_, filename, object_id);

            AnalyzerCallback ac = boost::bind (&kpoBaseApp::modelCloudAnalyzed, this, _1);
            analyzer->setAnalyzerCallback( ac );

            analyzer_thread_pool.schedule(boost::ref( *analyzer ));
        }
        else {
            kpoAnalyzerThread analyzer(keypoint_downsampling_radius_);

            analyzer.copyInputCloud(model_, filename, object_id);
            analyzer.setAnalyzerCallback( boost::bind (&kpoBaseApp::modelCloudAnalyzed, this, _1) );

            analyzer();
        }
    }
}

void kpoBaseApp::modelCloudAnalyzed(kpoCloudDescription od)
{
    QMutexLocker locker (&mtx_);

    std::cout << "modelCloudAnalyzed()" << std::endl;

    boost::shared_ptr<kpoMatcherThread> model_thread(new kpoMatcherThread(od.keypoints, od.descriptors, od.reference_frames));
    model_thread->object_id = od.object_id;
    model_thread->filename = od.filename;

    MatchCallback f = boost::bind (&kpoBaseApp::matchesFound, this, _1, _2, _3);
    model_thread->setMatchCallback(f);

    matcher_threads.push_back(model_thread);
}

void kpoBaseApp::matchesFound(int object_id, Eigen::Vector3f translation, Eigen::Matrix3f rotation)
{
    std::cout << "found object " << object_id << " at ";
    std::cout << translation(0) << "," << translation(1) << "," << translation(2) << std::endl;

    osc_sender->sendObject(object_id, translation(0), translation(1), translation(2));
}


void kpoBaseApp::depth_callback (const boost::shared_ptr< openni_wrapper::DepthImage > &depth_image)
{
    QMutexLocker locker (&mtx_);

    unsigned image_width_ = depth_image->getWidth();
    unsigned image_height_ = depth_image->getHeight();

    const XnDepthPixel* pDepthMap = depth_image->getDepthMetaData().Data();

    cv::Mat depth(480, 640, CV_8UC1);
    int x, y, i = 0;
    for(  y =0; y < 480 ; y++)
    {
        for( x = 0; x < 640; x++)
        {
            depth.at<unsigned char >(y,x) = pDepthMap[i] / 8;

            i++;
        }
    }

    threshold( depth, scene_depth_image_, depth_image_threshold_, 255, THRESH_TOZERO_INV );

    depth_blob_finder.find(scene_depth_image_);

    processDepthBlobs(depth_blob_finder);
}
void kpoBaseApp::processDepthBlobs(BlobFinder bf)
{
    for( int i = 0; i < bf.numBlobs; i++ )
    {
        if (bf.radius[i] > 15) {
            osc_sender->sendBlob(bf.center[i].x, bf.center[i].y, bf.radius[i]);

            findMatchingContours(bf.contours[i]);
        }
    }
}

bool operator <(const kpoObjectContour& a, const kpoObjectContour& b) { return (a.error < b.error); }

void kpoBaseApp::findMatchingContours(Contour scene_contour)
{
    for (int i=0; i<contour_objects_.size(); i++) {
        kpoObjectContour test_object = contour_objects_[i];
        contour_objects_[i].error = cv::matchShapes(scene_contour, test_object.contour, CV_CONTOURS_MATCH_I3, 0);
    }

    std::sort(contour_objects_.begin(), contour_objects_.end());

    for (int i=0; i<contour_objects_.size(); i++) {
        kpoObjectContour test_object = contour_objects_[i];
//        std::cout << test_object.object_id << " ";
    }

//    std::cout << std::endl;
}


void kpoBaseApp::image_callback (const boost::shared_ptr<openni_wrapper::Image> &image)
{
    unsigned image_width_ = image->getWidth();
    unsigned image_height_ = image->getHeight();

    static unsigned rgb_array_size = 0;
    static boost::shared_array<unsigned char> rgb_array ((unsigned char*)(NULL));

    static unsigned char* rgb_buffer = 0;

    // here we need exact the size of the point cloud for a one-one correspondence!
    if (rgb_array_size < image_width_ * image_height_ * 3)
    {
      rgb_array_size = image_width_ * image_height_ * 3;
      rgb_array.reset (new unsigned char [rgb_array_size]);
      rgb_buffer = rgb_array.get ();
    }
    image->fillRGB (image_width_, image_height_, rgb_buffer, image_width_ * 3);

    {
        QMutexLocker locker (&mtx_);

        openniImage2opencvMat((XnRGB24Pixel*)rgb_buffer, scene_image_, image_height_, image_width_);
    }

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
void kpoBaseApp::cloud_callback (const CloudConstPtr& cloud)
{
    if (paused_) {
        return;
    }

    osc_sender->send("/kinect/pointcloud/size", cloud->size());

    if (matcher_thread_pool.pending() < thread_load) {

        process_cloud(cloud);
    }

}


void kpoBaseApp::process_cloud (const CloudConstPtr& cloud)
{
    QMutexLocker locker (&mtx_);
    FPS_CALC ("computation");

    scene_cloud_.reset (new Cloud);

    depth_filter_.setInputCloud (cloud);
    depth_filter_.setFilterLimits(0, depth_threshold_);
    depth_filter_.filter (*scene_cloud_);

    if (process_scene_) {

        if (THREADED_ANALYSIS) {

            kpoAnalyzerThread *kat = new kpoAnalyzerThread(keypoint_downsampling_radius_);
            boost::shared_ptr<kpoAnalyzerThread> analyzer(kat);

            analyzer->copyInputCloud(scene_cloud_, "", 0);

            AnalyzerCallback ac = boost::bind (&kpoBaseApp::sceneCloudAnalyzed, this, _1);
            analyzer->setAnalyzerCallback( ac );

            analyzer_thread_pool.schedule(boost::ref( *analyzer ));
        }
        else {

            kpoAnalyzerThread analyzer(keypoint_downsampling_radius_);

            analyzer.copyInputCloud(scene_cloud_, "", 0);
            analyzer.setAnalyzerCallback( boost::bind (&kpoBaseApp::sceneCloudAnalyzed, this, _1) );

            analyzer();
        }
    }
}

void kpoBaseApp::sceneCloudAnalyzed(kpoCloudDescription od)
{
    std::cout << "sceneCloudAnalyzed()" << std::endl;
    std::cout << "cloud has " << od.cloud->size() << std::endl;


    if (match_models_) {

        int batch = thread_load - matcher_thread_pool.pending();

        for (unsigned i=0; i<batch; i++) {

            boost::shared_ptr<kpoMatcherThread> matcher = matcher_threads.at(model_index);

            matcher->copySceneClouds(od.keypoints, od.descriptors, od.reference_frames);

            matcher_thread_pool.schedule(boost::ref( *matcher ));

            model_index = (model_index + 1) % matcher_threads.size();

        }
    }
}



void kpoBaseApp::loadContourFiles()
{
    std::cout << "kpoBaseApp::loadContourFiles() with " << contours_folder_.toStdString() << std::endl;

    QStringList nameFilter("*.path");
    QDir directory(contours_folder_);
    QStringList contour_files = directory.entryList(nameFilter);

    int count = contour_files.length();
    std::cout << "will load " << count << " contour files." << std::endl;

    for (int i=0; i<count; i++) {

        QString qs_filename = contour_files[i];
        string filename = qs_filename.toStdString();

        std::cout << "reading " << filename << std::endl;

        kpoObjectContour obj = load_contour_file(contours_folder_.toStdString() + "/" + filename);
        contour_objects_.push_back(obj);
    }
}


kpoObjectContour kpoBaseApp::load_contour_file(string file_path)
{
    kpoObjectContour object_contour;

    {
      // Create and input archive
      std::ifstream ifs(file_path.c_str());
      boost::archive::text_iarchive ar(ifs);

      // Load data
      ar & object_contour;
    }

    return object_contour;
}

void kpoBaseApp::save_contour_file(kpoObjectContour object_contour, string file_path)
{
    {
      // Create an output archive
      std::ofstream ofs(file_path.c_str());
      boost::archive::text_oarchive ar(ofs);

     // Write data
      ar & object_contour;
    }
}


