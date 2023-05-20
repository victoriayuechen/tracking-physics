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

#define INITIAL_NOISE_COVARIANCE 0.00001
#define INITIAL_NOISE_MEAN 0.0
#define COHERENCE_LIMIT 0.01
#define N_THREADS 1

using namespace std::chrono_literals;
typedef pcl::PointXYZRGBA RefPointType; // actual type of the points in cloud
typedef pcl::tracking::ParticleXYZRPY Particle;  // type of the points that will be used in tracking 
typedef pcl::tracking::ParticleFilterTracker<RefPointType, Particle> ParticleFilter;

class BaseTracker {
private:
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
    pcl::tracking::ParticleFilterTracker<RefPointType, Particle>::Ptr tracker;

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
};

class VirtualCamera : public BaseTracker {
    public:
        void incrementFrame();
        void initializeCamera(std::string& outputDir, long frameMax, bool save);
};