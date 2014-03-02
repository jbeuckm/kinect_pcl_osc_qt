#include <QElapsedTimer>

#include "kpoAnalyzerThread.h"


kpoAnalyzerThread::kpoAnalyzerThread(float downsampling_radius = .005f)
    : downsampling_radius_(downsampling_radius)
    , scene_cloud_ (new Cloud())
{
    uniform_sampling.setRadiusSearch (downsampling_radius_);

    statistical_outlier_remover.setMeanK (50);
    statistical_outlier_remover.setStddevMulThresh (1.0);

    shot_radius_ = 0.04f;

    cg_size_ = 0.01f;
    cg_thresh_ = 5.0f;

    rf_rad_ = 0.015f;

    if (false) {
        norm_est.setKSearch (16);
    }
    else {
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
        norm_est.setSearchMethod (tree);
        norm_est.setRadiusSearch (0.02);
    }

    shot.setRadiusSearch (shot_radius_);

    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);
}

void kpoAnalyzerThread::setInputCloud(CloudPtr &cloud)
{
    pcl::copyPointCloud(*cloud, *scene_cloud_);
}

void kpoAnalyzerThread::operator ()()
{
    Cloud cleanCloud;
    removeNoise(scene_cloud_, cleanCloud);
    pcl::copyPointCloud(cleanCloud, *scene_cloud_);


    if (scene_cloud_->size() < 25) {
        std::cout << "cloud too small" << std::endl;
        return;
    }
    if (scene_cloud_->size() > 35000) {
        std::cout << "cloud too large" << std::endl;
        return;
    }
    std::cout << "cloud has " << scene_cloud_->size() << " points" << std::endl;


    scene_normals_.reset (new NormalCloud ());
    estimateNormals(scene_cloud_, scene_normals_);

    scene_keypoints_.reset(new Cloud ());
    downSample(scene_cloud_, scene_keypoints_);

    scene_descriptors_.reset(new DescriptorCloud ());
    computeShotDescriptors(scene_cloud_, scene_keypoints_, scene_normals_, scene_descriptors_);

    scene_refs_.reset(new RFCloud ());
    estimateReferenceFrames(scene_cloud_, scene_normals_, scene_keypoints_, scene_refs_);

    kpoObjectDescription od;
    od.cloud = scene_cloud_;
    od.keypoints = scene_keypoints_;
    od.normals = scene_normals_;
    od.descriptors = scene_descriptors_;
    od.reference_frames = scene_refs_;

    callback_(od);
}


void kpoAnalyzerThread::setDownsamplingRadius(float _radius)
{
    downsampling_radius_ = _radius;
    uniform_sampling.setRadiusSearch (downsampling_radius_);
}


void kpoAnalyzerThread::removeNoise(const CloudConstPtr &cloud, Cloud &filtered_cloud)
{
    statistical_outlier_remover.setInputCloud (cloud);
    statistical_outlier_remover.filter (filtered_cloud);
}


void kpoAnalyzerThread::estimateNormals(CloudPtr &cloud, NormalCloudPtr &normals)
{
    norm_est.setInputCloud (cloud);
    norm_est.compute (*normals);
}


void kpoAnalyzerThread::downSample(const CloudConstPtr &cloud, CloudPtr &keypoints)
{
    pcl::PointCloud<int> sampled_indices;

    uniform_sampling.setInputCloud (cloud);

    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
}


void kpoAnalyzerThread::computeShotDescriptors(const CloudConstPtr &cloud, const CloudConstPtr &keypoints, const NormalCloud::ConstPtr &normals, DescriptorCloud::Ptr &descriptors)
{
    shot.setInputCloud (keypoints);
    shot.setInputNormals (normals);
    shot.setSearchSurface (cloud);
    shot.compute (*descriptors);
}



void kpoAnalyzerThread::estimateReferenceFrames(const CloudConstPtr &cloud,
                             const NormalCloud::ConstPtr &normals,
                             const CloudConstPtr &keypoints,
                             RFCloud::Ptr &rf)
{
    if (!rf) {
        rf.reset(new RFCloud ());
    }

    rf_est.setInputCloud (keypoints);
    rf_est.setInputNormals (normals);
    rf_est.setSearchSurface (cloud);
    rf_est.compute (*rf);
}


double kpoAnalyzerThread::computeCloudResolution (const CloudConstPtr &cloud)
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

