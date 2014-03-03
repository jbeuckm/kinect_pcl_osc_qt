#include "kpoMatcherThread.h"

kpoMatcherThread::kpoMatcherThread(Cloud::Ptr model_keypoints_, DescriptorCloud::Ptr model_descriptors_, RFCloud::Ptr model_refs_)
    : model_keypoints(new Cloud ())
    , model_descriptors(new DescriptorCloud ())
    , model_refs(new RFCloud ())
    , scene_keypoints(new Cloud ())
    , scene_descriptors(new DescriptorCloud ())
    , scene_refs(new RFCloud ())
{
    cg_size_ = 0.01f;
    cg_thresh_ = 5.0f;

    pcl::copyPointCloud(*model_keypoints_, *model_keypoints);
    pcl::copyPointCloud(*model_descriptors_, *model_descriptors);
    pcl::copyPointCloud(*model_refs_, *model_refs);

    std::cout << "THREAD: model_keypoints->size = " << model_keypoints->size() << std::endl;
}

void kpoMatcherThread::setMatchCallback(MatchCallback callback)
{
    callback_ = callback;
}


void kpoMatcherThread::copySceneClouds(Cloud::Ptr scene_keypoints_, DescriptorCloud::Ptr scene_descriptors_, RFCloud::Ptr scene_refs_)
{
    pcl::copyPointCloud(*scene_keypoints_, *scene_keypoints);
    pcl::copyPointCloud(*scene_descriptors_, *scene_descriptors);
    pcl::copyPointCloud(*scene_refs_, *scene_refs);
}


void kpoMatcherThread::operator ()()
{
    if (scene_descriptors->size() == 0) return;

    pcl::KdTreeFLANN<DescriptorType> match_search;

    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    match_search.setInputCloud (model_descriptors);

    size_t size = scene_descriptors->size ();
    for (size_t i = 0; i < size; i++)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
            continue;
        }

        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);

        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }

    std::vector<pcl::Correspondences> clustered_corrs;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> hough_clusterer;

    hough_clusterer.setUseInterpolation (true);
    hough_clusterer.setUseDistanceWeight (false);
    hough_clusterer.setHoughBinSize (cg_size_);
    hough_clusterer.setHoughThreshold (cg_thresh_);


    hough_clusterer.setInputCloud (model_keypoints);
    hough_clusterer.setInputRf (model_refs);

    hough_clusterer.setSceneCloud (scene_keypoints);
    hough_clusterer.setSceneRf (scene_refs);

    hough_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    hough_clusterer.recognize (rototranslations, clustered_corrs);

    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        std::cout << "matched from file " << filename << std::endl;

        callback_(object_id, translation, rotation);
    }
}
