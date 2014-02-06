#include "kpoPclFunctions.h"

kpoPclFunctions::kpoPclFunctions()
{
    downsampling_radius_ = .01f;
    shot_radius_ = 0.02f;

    cg_size_ = 0.01f;
    cg_thresh_ = 5.0f;

    rf_rad_ = 0.015f;
}



void kpoPclFunctions::estimateNormals(const Cloud::ConstPtr &cloud, NormalCloud::Ptr &normals)
{
    norm_est.setKSearch (10);
    norm_est.setInputCloud (cloud);
    norm_est.compute (*normals);
}


void kpoPclFunctions::downSample(const Cloud::ConstPtr &cloud, Cloud::Ptr &keypoints)
{
    pcl::PointCloud<int> sampled_indices;

    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (downsampling_radius_);

    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
    std::cout << "Cloud total points: " << cloud->size () << "; Selected Keypoints: " << keypoints->size () << std::endl;
}


void kpoPclFunctions::computeShotDescriptors(const Cloud::ConstPtr &cloud, const Cloud::ConstPtr &keypoints, const NormalCloud::ConstPtr &normals, DescriptorCloud::Ptr &descriptors)
{
    shot.setRadiusSearch (shot_radius_);

    shot.setInputCloud (keypoints);
    shot.setInputNormals (normals);
    shot.setSearchSurface (cloud);
    shot.compute (*descriptors);
}


void kpoPclFunctions::correlateDescriptors(const DescriptorCloud::ConstPtr &scene_descriptors, const DescriptorCloud::ConstPtr &model_descriptors, pcl::CorrespondencesPtr &model_scene_corrs)
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


std::vector<pcl::Correspondences> kpoPclFunctions::clusterCorrespondences(const Cloud::ConstPtr &scene_keypoints, const Cloud::ConstPtr &model_keypoints, const pcl::CorrespondencesPtr &model_scene_corrs)
{
    std::vector<pcl::Correspondences> clustered_corrs;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

//    gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);

    return clustered_corrs;
}


std::vector<pcl::Correspondences> kpoPclFunctions::houghCorrespondences(
        const Cloud::ConstPtr &scene_keypoints,
        const RFCloud::ConstPtr &scene_rf,
        const Cloud::ConstPtr &model_keypoints,
        const RFCloud::ConstPtr &model_rf,
        const pcl::CorrespondencesPtr &model_scene_corrs)
{
    std::vector<pcl::Correspondences> clustered_corrs;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    std::cout << "model " << model_keypoints->size() << "/" << model_rf->size() << std::endl;
    std::cout << "scene " << scene_keypoints->size() << "/" << scene_rf->size() << std::endl;
    std::cout << "correspondences " << model_scene_corrs->size() << std::endl;

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
    std::cout << "model instances: " << rototranslations.size() << std::endl;

    return clustered_corrs;
}


void kpoPclFunctions::estimateReferenceFrames(const Cloud::ConstPtr &cloud,
                             const NormalCloud::ConstPtr &normals,
                             const Cloud::ConstPtr &keypoints,
                             RFCloud::Ptr &rf)
{
    if (!rf) {
        rf.reset(new RFCloud ());
    }

    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (keypoints);
    rf_est.setInputNormals (normals);
    rf_est.setSearchSurface (cloud);
    rf_est.compute (*rf);
}


double kpoPclFunctions::computeCloudResolution (const Cloud::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}
