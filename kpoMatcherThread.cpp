#include "kpoMatcherThread.h"

kpoMatcherThread::kpoMatcherThread(Cloud::Ptr model_keypoints_, DescriptorCloud::Ptr model_descriptors_, RFCloud::Ptr model_refs_)
{
    model_keypoints = model_keypoints_;
    model_descriptors = model_descriptors_;
    model_refs = model_refs_;

    scene_keypoints.reset (new Cloud ());
    scene_descriptors.reset (new DescriptorCloud ());
    scene_refs.reset (new RFCloud ());

    hough_clusterer.setInputCloud (model_keypoints);
    hough_clusterer.setInputRf (model_refs);
}


void kpoMatcherThread::copySceneClouds(Cloud::Ptr scene_keypoints_, DescriptorCloud::Ptr scene_descriptors_, RFCloud::Ptr scene_refs_)
{
    pcl::copyPointCloud(*scene_keypoints_, *scene_keypoints);
    pcl::copyPointCloud(*scene_descriptors_, *scene_descriptors);
    pcl::copyPointCloud(*scene_refs_, *scene_refs);
}


int kpoMatcherThread::operator ()()
{
    std::cout << "hello from thread" << std::endl;
/*
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    match_search.setInputCloud (model_descriptors);

    size_t size = scene_descriptors->size ();
    for (size_t i = 0; i < size; ++i)
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
//        std::cout << "msc" << model_scene_corrs->size() << "/" << model_->descriptors->size() << " ";

    if (model_scene_corrs->size() < 10) {
        return -1;
    }

    std::vector<pcl::Correspondences> clustered_corrs;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

    hough_clusterer.setSceneCloud (scene_keypoints);
    hough_clusterer.setSceneRf (scene_refs);

    hough_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    hough_clusterer.cluster (clustered_corrs);
    hough_clusterer.recognize (rototranslations, clustered_corrs);

    return clustered_corrs.size();
    */
    return 0;
}
