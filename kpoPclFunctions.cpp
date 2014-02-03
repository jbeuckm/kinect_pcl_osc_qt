#include "kpoPclFunctions.h"

kpoPclFunctions::kpoPclFunctions()
{
    downsampling_radius_ = .01f;
    shot_radius_ = 0.02f;
}



void kpoPclFunctions::estimateNormals(const pcl::PointCloud<PointType>::ConstPtr &cloud, pcl::PointCloud<NormalType>::Ptr &normals)
{
    norm_est.setKSearch (10);
    norm_est.setInputCloud (cloud);
    norm_est.compute (*normals);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr kpoPclFunctions::computeShotDescriptors(const pcl::PointCloud<PointType>::ConstPtr &cloud, const pcl::PointCloud<NormalType>::ConstPtr &normals, pcl::PointCloud<DescriptorType>::Ptr &descriptors)
{

    pcl::PointCloud<int> sampled_indices;
    pcl::PointCloud<PointType>::Ptr keypoints (new pcl::PointCloud<PointType> ());

    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (downsampling_radius_);

    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
//    std::cout << "Cloud total points: " << cloud->size () << "; Selected Keypoints: " << keypoints->size () << std::endl;

    shot.setRadiusSearch (shot_radius_);

    shot.setInputCloud (keypoints);
    shot.setInputNormals (normals);
    shot.setSearchSurface (cloud);
    shot.compute (*descriptors);

    /*
    shot.setSearchMethod (tree); //kdtree
    shot.setIndices (indices); //keypoints
    shot.setInputCloud (cloud); //input
    shot.setInputNormals(normals);//normals
    shot.setRadiusSearch (0.06); //support
    shot.compute (*descriptors); //descriptors
*/
    return keypoints;
}


void kpoPclFunctions::matchModelInScene(const pcl::PointCloud<DescriptorType>::ConstPtr &scene_descriptors, const pcl::PointCloud<DescriptorType>::ConstPtr &model_descriptors, pcl::CorrespondencesPtr &model_scene_corrs)
{

    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
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

}


std::vector<pcl::Correspondences> kpoPclFunctions::clusterCorrespondences(const pcl::PointCloud<PointType>::ConstPtr &scene_keypoints, const pcl::PointCloud<PointType>::ConstPtr &model_keypoints, const pcl::CorrespondencesPtr &model_scene_corrs)
{
    std::vector<pcl::Correspondences> clustered_corrs;

    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);

    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    gc_clusterer.cluster (clustered_corrs);
    //gc_clusterer.recognize (rototranslations, clustered_corrs);

    return clustered_corrs;
}

