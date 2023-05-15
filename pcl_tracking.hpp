#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h> // for transformPointCloud

#include <pcl/visualization/cloud_viewer.h>
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
#define DATA_PATH = "../data/frame_"

// using namespace pcl::tracking;
using namespace std::chrono_literals;
typedef pcl::PointXYZ RefPointType; // actual type of the points in cloud
typedef pcl::PointXYZRPY Particle;  // type of the points that will be used in tracking 

class Tracker {
    // For managing the video frames 
    long frameCount = 0L; 
    long frameMax; 
    
    // Array storing the 6 degrees of freedom of current object. 
    std::array<double, 6> dof; 
    
    // Managing the threads for tracking 
    std::thread threadTracking; 
    std::mutex trackingMutex; 
    std::mutex mutexPosRot; 

    // for tracking using PCL 
    pcl::PointIndices::Ptr floorPoints; 
    pcl::ModelCoefficients::Ptr floorCoefficients; 
    pcl::PointCloud<RefPointType>::Ptr floorCloud; 
    pcl::PointCloud<RefPointType>::Ptr objectCloud; 
    pcl::SACSegmentation<RefPointType> 







}
