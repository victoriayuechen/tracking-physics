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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#define SENSOR_FROM_CENTER_Y -0.52
#define SENSOR_FROM_CENTER_Z 1.18
#define INITIAL_NOISE_COVARIANCE 0.00001
#define INITIAL_NOISE_MEAN 0.0
#define COHERENCE_LIMIT 0.1
#define N_THREADS 1

using namespace std::chrono_literals;
typedef pcl::PointXYZRGBA RefPointType; // actual type of the points in cloud
typedef pcl::tracking::ParticleXYZRPY Particle;  // type of the points that will be used in tracking 
typedef pcl::tracking::ParticleFilterTracker<RefPointType, Particle> ParticleFilter;

class BaseTracker {
private:
    std::mutex trackingMutex;

    std::ofstream truthOutput;
    std::ofstream guessOutput;
    float downsampleGridSize;
    float delta;
    float epsilon;

    static inline std::vector<double> initialNoiseMean = std::vector<double>(6, INITIAL_NOISE_MEAN);
    static inline std::vector<double> initialNoiseCovariance = std::vector<double>(6, INITIAL_NOISE_COVARIANCE);

    void runRANSAC(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);
    void initializeKLDFilter(float binSize, int numParticles, double variance, float delta, float epsilon);

protected:
    // Saving the resulting point cloud predictions
    bool save;
    std::string outputDir;
    void savePointCloud();

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
    pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, Particle>::Ptr tracker;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getParticles();

    Eigen::Matrix4f cameraCorrection;
public:
    // For managing the video frames
    long frameCount;
    long frameMax;

    // Filtering applied at each frame
    void cloudCallBack(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);

    // Sets up the filter and algorithm parameters
    void setDownsampleSize(float size);
    void setUpTracking(const std::string& modelLoc,
                       int numParticles,
                       double variance,
                       float delta,
                       float epsilon,
                       float binSizeDimensions);
    void setMaxFrame(long maxFrame);
    void writePredictions(std::string& truthFile, std::string& guessFile);
};

class VirtualCamera : public BaseTracker {
    public:
        void initializeCamera(std::string& outputDir, long frameMax, bool save);
};