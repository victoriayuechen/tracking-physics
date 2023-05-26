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

#define SENSOR_FROM_CENTER_Y -0.52
#define SENSOR_FROM_CENTER_Z 1.18
#define INITIAL_NOISE_COVARIANCE 0.00001
#define INITIAL_NOISE_MEAN 0.0
#define COHERENCE_LIMIT 0.01

using namespace std::chrono_literals;
typedef pcl::PointXYZRGBA RefPointType;


class PointcloudTracker {
    private:
        double tracking_time_start;
        double tracking_time_stop;
        double preprocessing_time_start;
        double preprocessing_time_stop;
        double preprocessing_time;
        double downsampled_grid_size;
        unsigned int frame_number;
        unsigned int noisy_frames;
        bool save_video;
        bool run_tracking;
        bool preprocessing = true;
        std::string video_location;
        std::array<double, 6> array_object_position_and_rotation{};
        std::array<double, 6> array_object_position_and_rotation_temp{};
        Eigen::Matrix4f camera_correction;
        std::thread thread_tracking;
        std::mutex tracking_mutex;
        std::mutex mutex_posrot;

        pcl::PointIndices::Ptr points_on_floor;
        pcl::ModelCoefficients::Ptr floor_coefficients;
        pcl::PointCloud<RefPointType>::Ptr floor_cloudmodel;
        pcl::PointCloud<RefPointType>::Ptr objects_cloudmodel;
        pcl::SACSegmentation<RefPointType> floor_segmentation;
        pcl::ApproximateVoxelGrid<RefPointType> downsample_grid;
        pcl::tracking::ParticleXYZRPY object_position_and_rotation;
        pcl::ExtractIndices<RefPointType> separate_objects_from_floor;
        pcl::tracking::ParticleFilterTracker<RefPointType, pcl::tracking::ParticleXYZRPY>::Ptr object_tracker;
        /**
         * @brief Calculates the transformation matrix to transform the kinect coordinate system to the world coordinate system
         * @param[in] cloud pointcloud frame from the kinect of the floor of the scene
         * @return Eigen::Matrix4f Returns the transformation matrix to transform the kinect coordinate system to the world coordinate system
         */
        Eigen::Matrix4f ChangeOfBasis (const pcl::PointCloud<RefPointType>::ConstPtr &cloud);

    protected:
        /**
         * @brief Sets up object tracking parameters.
         * @param[in] object_cloudmodel_location location of the pointcloud of the object to be tracked
         */
        void SetUpObjectTracking(std::string object_cloudmodel_location, unsigned int particles, double variance, bool kld);
        /**
         * @brief Tracks the position of an object in the environment
         */
        void TrackingThread(bool save_results);
        /**
         * @brief Saves frames of point cloud to reuse later.
         * @param[in] cloud pointcloud frame from the tracker
         */
        void SavePointcloudVideo(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);
        void SetRunTracking(bool track);
        void SetThreadTracking(bool save_results);
        void SetVideoLocation(std::string location_video);
        std::string GetVideoLocation();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::stringstream ss;
        std::ofstream position_tracking_time;
        /**
         * @brief Prepares the received cloud for tracking
         * @param[in] cloud pointcloud frame from the tracker
         */
        void CloudCallback(const pcl::PointCloud<RefPointType>::ConstPtr &cloud);
        /**
         * @brief Gets the object position in XYZ coordinates and the object rotation in quaternion values
         * @return Array with object position (XYZ) and object rotation (quaternion)
         */
        std::array<double, 6> GetObjectPositionRotation();
        void IncrementFrameNumber(unsigned int frame);
        unsigned int GetFrameNumber();
        void SetDownsampleSize(double downsample_size);
        void SetSaveVideo(bool setting);
        void savePointCloud();
};

class KinectCamera : public PointcloudTracker {
    private:
        unsigned int frames_available;
        //pcl::Grabber* kinect_data_grabber;
        /**
         * @brief Sets up the OpenNI2 Grabber to grab the data from the kinect whenever data is being sent
         */
        void SetUpKinectListener(bool use_video, std::string location_video);
        /**
         * @brief Starts monitoring whether the kinect sends any data
         */
        void StartKinectListener(bool use_video, bool save_results);

    public:
        /**
         * @brief Sets up Kinect data acquisition if desired.
         */
        void SetUpKinect(std::string model3d, int particles, double variance, bool kld, bool use_video, bool save_results, std::string location_video);
        /**
         * @brief Stops monitoring whether the kinect sends any data
         */
        void StopKinectListener(bool use_video);
        unsigned int GetFramesAvailable();
};
