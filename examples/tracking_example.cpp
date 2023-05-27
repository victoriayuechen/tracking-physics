#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/io/openni_grabber.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h> // for transformPointCloud

// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <boost/format.hpp>

#include <mutex>
#include <thread>

using namespace pcl::tracking;
using namespace std::chrono_literals;

typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;

std::mutex mtx_;
std::mutex write_;
ParticleFilter::Ptr tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;

std::ofstream truthOutput;
std::ofstream guessOutput;

//       //Draw red particles 
//       {
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

// 	if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
// 	  viz.addPointCloud (particle_cloud, red_color, "particle cloud");
//       }
//       return true;
//     }
//   else
//     {
//       return false;
//     }
// }

// //Draw model reference point cloud
// void
// drawResult (pcl::visualization::PCLVisualizer& viz)
// {
//   ParticleXYZRPY result = tracker_->getResult ();
//   Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

//   //move close to camera a little for better visualization
//   transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
//   CloudPtr result_cloud (new Cloud ());
//   pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

//   //Draw blue model reference point cloud
//   {
//     pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

//     if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
//       viz.addPointCloud (result_cloud, blue_color, "resultcloud");
//   }
// }

// //visualization's callback function
// void
// viz_cb (pcl::visualization::PCLVisualizer& viz)
// {
//   std::lock_guard<std::mutex> lock (mtx_);
    
//   if (!cloud_pass_)
//     {
//       std::this_thread::sleep_for(1s);
//       return;
//    }

//   //Draw downsampled point cloud from sensor    
//   if (new_cloud_ && cloud_pass_downsampled_)
//     {
//       CloudPtr cloud_pass;
//       cloud_pass = cloud_pass_downsampled_;
    
//       if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
// 	{
// 	  viz.addPointCloud (cloud_pass, "cloudpass");
// 	  viz.resetCameraViewpoint ("cloudpass");
// 	}
//       bool ret = drawParticles (viz);
//       if (ret)
//         drawResult (viz);
//     }
//   new_cloud_ = false;
// }

pcl::PointXYZ getCenter(pcl::PointCloud<RefPointType>::Ptr& cloud) {
    auto cloudPoints = cloud->points;
    float numPoints = cloudPoints.size();
    pcl::PointXYZ center = pcl::PointXYZ(0.0f, 0.0f, 0.0f);

    for (const auto& p : cloudPoints) {
        center.x += p.x;
        center.y += p.y;
        center.z += p.z;
    }

    center.x /= numPoints;
    center.y /= numPoints;
    center.z /= numPoints;

    return center;
}

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result) {
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-40.0, 40.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}

void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size) {
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void evaluate() {
    std::lock_guard<std::mutex> lock (write_);
    std::cout << "----" <<  std::endl;
    // Gets the particle XYZRPY (centroid of particle cloud)
    ParticleT state = tracker_->getResult();
    {
       std::cout << "Guess: " << state.x << "," << state.y << "," << state.z << ", " << state.roll << "," << state.pitch << "," << state.yaw << std::endl;

//        // Write predicted centroids to file
        std::string out = std::to_string(state.x) + "," + std::to_string(state.y) + "," + std::to_string(state.z) + "\n";
        guessOutput << out;
    }

    auto center = getCenter(target_cloud);
    {
        std::cout << "Truth: " << center.x << " " << center.y << " " << center.z << std::endl;

//        // Write true centroids to file
      std::string out = std::to_string(center.x) + "," + std::to_string(center.y) + "," + std::to_string(center.z) + "\n";
      truthOutput << out;
    }
}

//OpenNI Grabber's cloud Callback function
void cloud_cb (const CloudConstPtr &cloud)
{
  std::lock_guard<std::mutex> lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  filterPassThrough (cloud, *cloud_pass_);
  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

//  if(counter < 10){
//	counter++;
//  }else{
    //Track the object
    tracker_->setInputCloud (cloud_pass_downsampled_);
    tracker_->compute ();
    new_cloud_ = true;
    evaluate();
  // }
}

int main () {
  target_cloud.reset(new Cloud());
  if (pcl::io::loadPCDFile ("../data/frame_0.pcd", *target_cloud) == -1) {
    std::cout << "pcd file not found" << std::endl;
    exit(-1);
  }

  counter = 0;

  //Set parameters
  new_cloud_  = false;
  downsampling_grid_size_ =  0.002;

  std::vector<double> default_step_covariance = std::vector<double> (6, 0.01);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;

  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (1));

  ParticleT bin_size;
  bin_size.x = 0.1f;
  bin_size.y = 0.1f;
  bin_size.z = 0.1f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;

  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (1000);
  tracker->setDelta (0.99);
  tracker->setEpsilon (0.2);
  tracker->setBinSize (bin_size);

  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (600);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setUseNormal (false);

  //Setup coherence object for tracking
  ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence
    (new ApproxNearestPairPointCloudCoherence<RefPointType>);

  DistanceCoherence<RefPointType>::Ptr distance_coherence
    (new DistanceCoherence<RefPointType>);
  coherence->addPointCoherence (distance_coherence);

  pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (0.1));
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance (0.1);

  tracker_->setCloudCoherence (coherence);

  //prepare the model of tracker's target
  Eigen::Vector4f c;
  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  CloudPtr transed_ref (new Cloud);
  CloudPtr transed_ref_downsampled (new Cloud);

  pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
  gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trans);

//===========================================
  int frameCount = 0; 
  int maxFrames = 1000;
  // frameCount++;


  truthOutput.open("../analysis/truth.txt");
  guessOutput.open("../analysis/guess.txt");

  while (frameCount < maxFrames) {
      target_cloud.reset (new Cloud);
      if (pcl::io::loadPCDFile<RefPointType> ("../data/frame_" + std::to_string(frameCount) + ".pcd", *target_cloud) == -1) {
      PCL_ERROR ("Could not read PCD file \n");
      return -1; 
    }
    
    cloud_cb(target_cloud);
    frameCount++; 
  }

}