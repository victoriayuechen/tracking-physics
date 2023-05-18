#include <mutex>
#include <thread>
#include <fstream>
#include <ostream>
#include <cstdio>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/segmentation/sac_segmentation.h>

#define DATA_PATH = "../data/frame_"
#define SENSOR_FROM_CENTER_Y -0.52
#define SENSOR_FROM_CENTER_Z 1.18
#define INITIAL_NOISE_COVARIANCE 0.00001
#define INITIAL_NOISE_MEAN 0.0
#define INITIAL_COVARIANCE 0.000225
#define COHERENCE_LIMIT 0.01
#define N_THREADS 1

using namespace std::chrono_literals;
typedef pcl::PointXYZRGBA RefPointType; // actual type of the points in cloud
typedef pcl::tracking::ParticleXYZRPY Particle;  // type of the points that will be used in tracking 

class BaseTracker {
private:
    float downsampleGridSize;
    float delta;
    float epsilon;

    static inline std::vector<double> initialNoiseMean = std::vector<double>(6, INITIAL_NOISE_MEAN);
    static inline std::vector<double> initialNoiseCovariance = std::vector<double>(6, INITIAL_NOISE_COVARIANCE);
    static inline std::vector<double> defaultStepCovariance = {INITIAL_COVARIANCE, INITIAL_COVARIANCE, INITIAL_COVARIANCE, 0.009, 0.009, 0.009};

    void runRANSAC(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);
    void initializeKLDFilter(float binSize, int numParticles, float delta, float epsilon);

protected:
    // Storing the 6 degrees of freedom of current object.
    Particle objPosRot;
    std::array<double, 6> dof;
        
    // Managing the threads for tracking
    std::thread threadTracking;
    std::mutex trackingMutex;

    // Segmented floor and object cloud + segmentation coefficients
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
    void setUpTracking(const std::string& modelLoc,
                       int numParticles,
                       double variance,
                       float delta,
                       float epsilon,
                       float binSizeDimensions);
    void tracking(); // TrackingThread
    void savePointCloud();

    // Not sure what this method would be for.
        // void setThreadTracking(); // SetThreadTracking

public:
    // For managing the video frames
    long frameCount = 0L;
    long frameMax;
         
    void cloudCallBack(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);
    std::array<double, 6> getDOF();
    void setDownsampleSize(float size);
    void incrementFrame();
};

class VirtualCamera : public BaseTracker {
    private: 
        std::string dataPath; 
        void setUpCameraListener(std::string videoLoc); 
        void startCameraListener(bool video, bool save); 
    public:
        VirtualCamera() {
            std::cout << "Initialised camera" << std::endl;
        }
        void setUpCamera(std::string modelLoc,
                                    int numParticles,
                                    double variance,
                                    float delta,
                                    float epsilon,
                                    float binSize,
                                    bool save,
                                    std::string resultLoc);
        void stopCameraListener(bool video);
};