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
#define SENSOR_FROM_CENTER_Y -0.52
#define SENSOR_FROM_CENTER_Z 1.18
#define INITIAL_NOISE_COVARIANCE 0.00001
#define INITIAL_NOISE_MEAN 0.0
#define COHERENCE_LIMIT 0.01

// using namespace pcl::tracking;
using namespace std::chrono_literals;
typedef pcl::PointXYZ RefPointType; // actual type of the points in cloud
typedef pcl::tracking::PointXYZRPY Particle;  // type of the points that will be used in tracking 

class Tracker {
    protected: 
        // Parameters for filter 
        float downsampleGridSize; 

        // Storing the 6 degrees of freedom of current object. 
        Particle objPosRot; 
        std::array<double, 6> dof; 
        
        // Managing the threads for tracking 
        std::thread threadTracking; 
        std::mutex trackingMutex; 
        std::mutex mutexPosRot; 

        // For segmenting the object from the floor plane 
        pcl::PointIndices::Ptr floorPoints; 
        pcl::ModelCoefficients::Ptr floorCoefficients; 
        pcl::PointCloud<RefPointType>::Ptr floorCloud; 
        pcl::PointCloud<RefPointType>::Ptr objectCloud; 
        pcl::SACSegmentation<RefPointType> floorSegmentation; 

        // For downsampling and segmentation 
        pcl::ApproximateVoxelGrid<RefPointType> downSampleGrid;
        pcl::ExtractIndices<RefPointType> floorObjectSegment; 

        // The actual filter applied for tracking 
        pcl::tracking::ParticleFilterTracker<RefPointType, Particle>::Ptr tracker; 
        // Sets up the parameters of the filter 
        void setUpTracking(std::string modelLoc, int numParticles, double variance, bool KLD); 
        void tracking(); // TrackingThread

        // Not sure what this method would be for. 
        // void setThreadTracking(); // SetThreadTracking

        void savePointCloud(const pcl::PointCloud<RefPointType>::ConstPtr &cloud); 
        void runRANSAC(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);
    public:
        // For managing the video frames 
        long frameCount = 0L; 
        long frameMax; 
         
        void cloudCallBack(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);
        std::array<double, 6> getDOF(); 
        void setDownsampleSize(float size); 
        void incrementFrame(); 
}

class VirtualCamera : public Tracker {
    private: 
        pcl::Grabber* kinectDataGrabber;
        std::string dataPath; 
        void setUpCameraListener(std::string videoLoc); 
        void startCameraListener(bool video, bool save); 
    public:
        void setUpCamera(std::string modelLoc, int numParticles, double variance, bool kld, bool save, std::string resultLoc); 
        void stopCameraListener(bool video);
}