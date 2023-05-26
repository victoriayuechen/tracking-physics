#include "pointcloud_tracking.hpp"


void PointcloudTracker::SetUpObjectTracking(std::string object_cloudmodel_location, unsigned int particles, double variance, bool kld)
{
    // //ParticleFilterOMPTracker
    // pcl::tracking::ParticleFilterOMPTracker<RefPointType, pcl::tracking::ParticleXYZRPY>::Ptr tracker (new pcl::tracking::ParticleFilterOMPTracker<RefPointType, pcl::tracking::ParticleXYZRPY> ()); //initialize the scheduler and set the number of threads to use

    // if(kld)
    // {
    //ParticleXYZRPY consists of values x,y,z,roll,pitch,yaw, set the values for each of the six parameters
    pcl::tracking::ParticleXYZRPY bin_size;
    bin_size.x = 0.045f;
    bin_size.y = 0.045f;
    bin_size.z = 0.045f;
    bin_size.roll = 0.045f;
    bin_size.pitch = 0.045f;
    bin_size.yaw = 0.045f;

    //KLDAdaptiveParticleFilterOMPTracker tracks the PointCloud which is given by setReferenceCloud within the measured PointCloud using particle filter method.
    // The number of the particles changes adaptively based on KLD sampling [D. Fox, NIPS-01], [D.Fox, IJRR03].
    // The computation of the weights of the particles is parallelized using OpenMP.
    pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, pcl::tracking::ParticleXYZRPY>::Ptr tracker (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType, pcl::tracking::ParticleXYZRPY> (5)); //initialize the scheduler and set the number of threads to use
    //Set all parameters specific for KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum (particles); //set the maximum number of the particles to be filtered
    tracker->setDelta (0.99); //set the delta that is used in the chi-squared distribution
    tracker->setEpsilon (0.2); //set the epsilon to be used to calculate the K-L boundary (it is the threshold for the K-L boundary)
    tracker->setBinSize (bin_size); //set the bin size of the tracker
    // }

    //Set parameters for tracking
    std::vector<double> initial_noise_covariance = std::vector<double> (6, INITIAL_NOISE_COVARIANCE); //set the diagonal elements of covariance matrix of initial noise
    std::vector<double> default_initial_mean = std::vector<double> (6, INITIAL_NOISE_MEAN); //set the mean values of initial noise
    std::vector<double> default_step_covariance = std::vector<double> (6, variance); //set the diagonal elements of the covariance matrix of the step noise
    default_step_covariance[3] = variance * 40;
    default_step_covariance[4] = variance * 40;
    default_step_covariance[5] = variance * 40;

    //Setup coherence object for tracking
    //ApproxNearestPairPointCloudCoherence computes coherence between two pointclouds using the approximate nearest point pairs
    pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence (new pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>);
    //DistanceCoherence computes coherence between two points from the distance between them. The coherence is calculated by 1 / (1 + weight * d^2 )
    pcl::tracking::DistanceCoherence<RefPointType>::Ptr distance_coherence (new pcl::tracking::DistanceCoherence<RefPointType>);
    coherence->addPointCoherence (distance_coherence); //add a PointCoherence to the PointCloudCoherence
    //Octree class implements nearest neighbor search operations
    pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (COHERENCE_LIMIT));
    coherence->setSearchMethod (search); //provide a pointer to a dataset to add additional information to estimate the features for every point in the input dataset.
    coherence->setMaximumDistance (COHERENCE_LIMIT); //set maximum distance between two points to be taken into account when looking for coherence (which points to compare)

    // Set target cloud to search for during tracking
    pcl::PointCloud<RefPointType>::Ptr object_cloudmodel (new pcl::PointCloud<RefPointType>());
    if(pcl::io::loadPCDFile (object_cloudmodel_location, *object_cloudmodel) == -1)
    {
        std::cout << "File " << object_cloudmodel_location << " could not be found" << std::endl;
        exit(-1);
    }
    //prepare the model of tracker's target
    Eigen::Vector4f object_centroid;
    Eigen::Affine3f translate_model = Eigen::Affine3f::Identity ();
    pcl::compute3DCentroid<RefPointType> (*object_cloudmodel, object_centroid); //compute the 3D (X-Y-Z) centroid of a set of points and return it as a 3D vector
    translate_model.translation ().matrix () = Eigen::Vector3f (object_centroid[0], object_centroid[1], object_centroid[2]);
    pcl::transformPointCloud<RefPointType> (*object_cloudmodel, *object_cloudmodel, translate_model.inverse()); //apply an affine transform defined by an Eigen Transform

    pcl::PointCloud<RefPointType>::Ptr transformed_reference_cloud_downsampled (new pcl::PointCloud<RefPointType>);
    this->downsample_grid.setInputCloud (object_cloudmodel); //provide a pointer to the input pointcloud for ApproximateVoxelGrid
    this->downsample_grid.setLeafSize (static_cast<float> (this->downsampled_grid_size), static_cast<float> (this->downsampled_grid_size), static_cast<float> (this->downsampled_grid_size)); //set the VoxelGrid leaf size
    this->downsample_grid.filter (*transformed_reference_cloud_downsampled); //calls the filtering method and returns the filtered dataset in output

    //Set all parameters for ParticleFilterOMPTracker
    tracker->setInitialNoiseCovariance (initial_noise_covariance); //set the covariance of the initial noise. It will be used when initializing the particles.
    tracker->setInitialNoiseMean (default_initial_mean); //set the mean of the initial noise. It will be used when initializing the particles.
    tracker->setStepNoiseCovariance (default_step_covariance); //set the covariance of the step noise
    tracker->setIterationNum (1); //set the number of iterations of the particle filter
    tracker->setParticleNum (particles-500); //set the number of particles to keep track of with the tracker
    tracker->setResampleLikelihoodThr(0.0); //set the threshold to re-initialize the particles
    tracker->setUseNormal (false); //set the value of use_normal_, a flag that indicates to use normal or not, defaults to false
    tracker->setCloudCoherence (coherence); //set the PointCloudCoherence as likelihood
    tracker->setTrans (translate_model); //set the transformation from the world coordinates to the frame of the particles
    tracker->setReferenceCloud (transformed_reference_cloud_downsampled); //set the point cloud to be tracked within the environment point cloud

    //object_tracker is a ParticleFilterTracker, it tracks the PointCloud which is given by setReferenceCloud within the measured PointCloud using particle filter method.
    this->object_tracker = tracker;
};

Eigen::Matrix4f PointcloudTracker::ChangeOfBasis (const pcl::PointCloud<RefPointType>::ConstPtr &cloud)
{
    // Initialize memory for new point cloud after downsampling
    pcl::PointCloud<RefPointType>::Ptr downsampled_floor (new pcl::PointCloud<RefPointType> ());
    // Downsample the pointcloud to be used for determining the normal
    pcl::VoxelGrid<RefPointType> downsample_grid_floor;
    downsample_grid_floor.setInputCloud (cloud);
    downsample_grid_floor.setLeafSize (0.4f, 0.4f, 0.4f);
    downsample_grid_floor.filter (*downsampled_floor);

    // Create the normal estimation class, and pass the downsampled pointcloud to it
    pcl::NormalEstimationOMP<RefPointType, pcl::Normal> normal_estimation_floor;
    pcl::search::KdTree<RefPointType>::Ptr kdtree (new pcl::search::KdTree<RefPointType>);
    pcl::PointCloud<pcl::Normal>::Ptr floor_normals (new pcl::PointCloud<pcl::Normal>);
    normal_estimation_floor.setInputCloud (downsampled_floor);
    // The kdtree will be filled with the points inside the object, based on the given cloud (since no other search surface is given)
    normal_estimation_floor.setSearchMethod (kdtree);
    // Use all point neighbors in a sphere of radius 5cm to determine the normal of a point
    normal_estimation_floor.setRadiusSearch (5);
    // Compute the normals for each of the points of the cloud
    normal_estimation_floor.compute (*floor_normals);

    //For now take the normal of the point in the middle of the point cloud, this of course could be done differently but for now it suffices
    pcl::Normal floor_normal = floor_normals->at(floor_normals->size()/2);

    //Translate the local coordinates of the kinect to the world coordinate center
    Eigen::Matrix4f translation_matrix_sensor;
    translation_matrix_sensor << 1, 0, 0, 0,
            0, 1, 0, SENSOR_FROM_CENTER_Y,
            0, 0, 1, SENSOR_FROM_CENTER_Z,
            0, 0, 0, 1;

    //Change the rotation of the current point cloud based on the obtained floor normal
    Eigen::Vector3f new_z_axis = Eigen::Vector3f(0, 1, -floor_normal.normal_y / floor_normal.normal_z);
    new_z_axis.normalize();
    Eigen::Vector3f new_y_axis = Eigen::Vector3f(-floor_normal.normal_x, -floor_normal.normal_y, -floor_normal.normal_z);
    Eigen::Vector3f new_x_axis = new_y_axis.cross(new_z_axis);

    Eigen::Matrix4f rotation_matrix_sensor;
    rotation_matrix_sensor <<   new_x_axis[0],  new_y_axis[0],  new_z_axis[0],  0,
            new_x_axis[1],  new_y_axis[1],  new_z_axis[1],  0,
            new_x_axis[2],  new_y_axis[2],  new_z_axis[2],  0,
            0,              0,              0,              1;

    return translation_matrix_sensor * rotation_matrix_sensor;
};

void PointcloudTracker::CloudCallback(const pcl::PointCloud<RefPointType>::ConstPtr &cloud)
{
    // PROFILE_FUNCTION_COLOR(5);
    if(this->save_video)
    {
        this->SavePointcloudVideo(cloud);
    }

    //skip first 10 frames since they tend to be a bit noisy
    if(this->noisy_frames < 10)
    {
        this->noisy_frames++;
    }
    else
    {
        this->preprocessing_time_start = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        if(this->noisy_frames < 11)
        {
//            PROFILE_SCOPE("RANSAC");
            //RANSAC Plane Segmentation in order to filter out the floor from the object
            this->points_on_floor.reset(new pcl::PointIndices);
            this->floor_coefficients.reset(new pcl::ModelCoefficients);
            this->floor_segmentation.setOptimizeCoefficients (true);
            this->floor_segmentation.setModelType (pcl::SACMODEL_PLANE);
            this->floor_segmentation.setMethodType (pcl::SAC_RANSAC);
            this->floor_segmentation.setMaxIterations (1000);
            this->floor_segmentation.setDistanceThreshold (0.02);
            this->floor_segmentation.setInputCloud (cloud);
            this->floor_segmentation.segment (*this->points_on_floor, *this->floor_coefficients);
        }


        if(this->noisy_frames < 11)
        {
           // PROFILE_SCOPE("SEPARATE");
            //Extract the indices of the cloud that are part of the floor
            this->objects_cloudmodel.reset(new pcl::PointCloud<RefPointType>());
            this->separate_objects_from_floor.setInputCloud (cloud);
            this->separate_objects_from_floor.setIndices (this->points_on_floor);
            this->floor_cloudmodel.reset(new pcl::PointCloud<RefPointType>());
            this->separate_objects_from_floor.setNegative (false);
            this->separate_objects_from_floor.filter (*this->floor_cloudmodel);
            //Extract the indices of the cloud that are part of the object
            // this->separate_objects_from_floor.filter (*this->objects_cloudmodel);
        }

        if(true) //If statement only for the tracing to be performed properly
        {
         //   PROFILE_SCOPE("CHANGE_BASIS");
            //Transform the pointcloud to align with the desired world axes
            if(this->noisy_frames < 11)
            {
                this->camera_correction = this->ChangeOfBasis(this->floor_cloudmodel);
                this->noisy_frames++;
            }
            pcl::transformPointCloud(*cloud, *objects_cloudmodel, this->camera_correction);

            // pcl::transformPointCloud(*objects_cloudmodel, *objects_cloudmodel, camera_correction);
            pcl::PassThrough<RefPointType> pass;
            pass.setInputCloud (objects_cloudmodel);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (-2, -0.008);
            pass.filter (*objects_cloudmodel);
        }

        if(true) //If statement only for the tracing to be performed properly
        {
        //    PROFILE_SCOPE("DOWNSAMPLE");
            //Downsample the pointcloud of the world in which the object is to be tracked
            this->downsample_grid.setInputCloud (this->objects_cloudmodel);
            this->downsample_grid.filter (*this->objects_cloudmodel);
        }

        this->preprocessing_time_stop = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        this->preprocessing_time = this->preprocessing_time_stop - this->preprocessing_time_start;
        this->tracking_mutex.lock();
        this->object_tracker->setInputCloud(this->objects_cloudmodel);
        this->tracking_mutex.unlock();
        this->preprocessing = false;
    }
};

void PointcloudTracker::TrackingThread(bool save_results)
{
    double preprocessing_start;
    double preprocessing_stop;
    this->run_tracking = true;

    while(this->run_tracking)
    {
        if (this->noisy_frames < 11)
        {
           // PROFILE_SCOPE("WAITING");
            std::this_thread::sleep_for(0.03s);
        }
        else
        {
            if(this->preprocessing == false)
            {
             //   PROFILE_SCOPE("TRACKING");
                this->tracking_time_start = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
                this->tracking_mutex.lock();
                this->object_tracker->compute();

                //Save the position and rotation of the object in such a way that it can be send over the network
                this->object_position_and_rotation = this->object_tracker->getResult();
                this->preprocessing = true;
                this->tracking_mutex.unlock();
                this->tracking_time_stop = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

                this->mutex_posrot.lock();
                this->array_object_position_and_rotation[0] = -100 * this->object_position_and_rotation[0]; //x
                this->array_object_position_and_rotation[1] = -100 * this->object_position_and_rotation[1]; //y
                this->array_object_position_and_rotation[2] = -100 * this->object_position_and_rotation[2]; //z
                this->array_object_position_and_rotation[3] = this->object_position_and_rotation[3]; //r
                this->array_object_position_and_rotation[4] = this->object_position_and_rotation[4]; //p
                this->array_object_position_and_rotation[5] = this->object_position_and_rotation[5]; //y
                this->mutex_posrot.unlock();

//                if(save_results)
//                {
//                    this->position_tracking_time << (boost::format("%f %f %f %f %f %f %f\n") % (this->object_position_and_rotation.x) % (this->object_position_and_rotation.y) % (this->object_position_and_rotation.z) % this->object_position_and_rotation.roll % this->object_position_and_rotation.pitch % this->object_position_and_rotation.yaw % ((this->tracking_time_stop - this->tracking_time_start + this->preprocessing_time) * 1000)).str();
//                }
            }
        }
    }
};

std::array<double, 6> PointcloudTracker::GetObjectPositionRotation()
{
    this->mutex_posrot.lock();
    this->array_object_position_and_rotation_temp = this->array_object_position_and_rotation;
    this->mutex_posrot.unlock();

    return this->array_object_position_and_rotation_temp;
};

void PointcloudTracker::SetDownsampleSize(double downsample_size)
{
    this->downsampled_grid_size = downsample_size;
};

void PointcloudTracker::SavePointcloudVideo(const pcl::PointCloud<RefPointType>::ConstPtr &cloud)
{
    this->ss.str(std::string());
    this->ss << this->video_location << "/frame_" << this->frame_number << ".pcd";
    pcl::io::savePCDFile(this->ss.str(), *cloud, true);
    this->frame_number++;
}

void PointcloudTracker::SetRunTracking(bool track)
{
    this->run_tracking = track;
};

void PointcloudTracker::SetThreadTracking(bool save_results)
{
    this->thread_tracking = std::thread(&PointcloudTracker::TrackingThread, this, save_results);
};

void PointcloudTracker::IncrementFrameNumber(unsigned int frame)
{
    this->frame_number += frame;
};

unsigned int PointcloudTracker::GetFrameNumber()
{
    return this->frame_number;
};

void PointcloudTracker::SetSaveVideo(bool setting)
{
    this->save_video = setting;
};

void PointcloudTracker::SetVideoLocation(std::string location_video)
{
    this->video_location = location_video;
};

std::string PointcloudTracker::GetVideoLocation()
{
    return this->video_location;
};

void KinectCamera::SetUpKinectListener(bool use_video, std::string location_video)
{
    this->SetVideoLocation(location_video);
    if(use_video)
    {
        DIR *dp;
        this->frames_available = -3;
        struct dirent *ep;
        dp = opendir (this->GetVideoLocation().c_str());

        if (dp != NULL)
        {
            while (ep = readdir (dp))
            {
                this->frames_available++;
            }

            (void) closedir (dp);
        }
        else
        {
            perror ("Couldn't open the directory. Error");
        }
    }
    else
    {
//        this->kinect_data_grabber = new pcl::io::OpenNI2Grabber (); //data grabber for OpenNI 2 devices s.a. the Kinect v1
//        std::function<void (const pcl::PointCloud<RefPointType>::ConstPtr&)> function_cloud_callback = [this] (const pcl::PointCloud<RefPointType>::ConstPtr& cloud) { this->CloudCallback (cloud); }; //initialize a function for the cloud callback
//        this->kinect_data_grabber->registerCallback (function_cloud_callback); //registers a callback function/method and executes it every time the Grabber detects a new data packet (frame) after data acquisition has started
    }
};

void KinectCamera::StartKinectListener(bool use_video, bool save_results)
{
    this->SetThreadTracking(save_results);
    if(!use_video)
    {
      //  this->kinect_data_grabber->start(); //start the data acquisition
    }
};

void KinectCamera::SetUpKinect(std::string model3d, int particles, double variance, bool kld, bool use_video, bool save_results, std::string location_video)
{
    this->SetUpObjectTracking(model3d, particles, variance, kld);
  //  this->SetUpKinectListener(use_video, location_video);
    // this->StartKinectListener(use_video, save_results);
};

void KinectCamera::StopKinectListener(bool use_video)
{
    this->SetRunTracking(false);
    if(!use_video)
    {
        //this->kinect_data_grabber->stop(); //stop the data acquisition
    }
};

unsigned int KinectCamera::GetFramesAvailable()
{
    return this->frames_available;
};

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


void PointcloudTracker::savePointCloud() {
    std::cout << "==================" << std::endl;
    // Gets the particle XYZRPY (centroid of particle cloud)
    auto state = this->object_tracker->getResult();
    {
        std::cout << "Guess: " << state.x << "," << state.y << "," << state.z << ", " << state.roll << "," << state.pitch << "," << state.yaw << std::endl;

//        // Write predicted centroids to file
//        std::string out = std::to_string(state.x) + "," + std::to_string(state.y) + "," + std::to_string(state.z) + "\n";
//        this->guessOutput << out;
    }

    auto center = getCenter(this->objects_cloudmodel);
    {
        std::cout << "Truth: " << center.x << " " << center.y << " " << center.z << std::endl;

//         // Write true centroids to file
//         std::string out = std::to_string(center.x) + "," + std::to_string(center.y) + "," + std::to_string(center.z) + "\n";
//         this->truthOutput << out;
    }

//    // Write the predicted point cloud to file
//    Eigen::Affine3f transformation = this->tracker->toEigenMatrix(state);
//    pcl::PointCloud<RefPointType>::Ptr resultCloud (new pcl::PointCloud<RefPointType>);
//    pcl::transformPointCloud<RefPointType> (*(this->tracker->getReferenceCloud()), *resultCloud, transformation);
//    pcl::io::savePCDFileASCII(outputDir + std::to_string(this->frameCount) + ".pcd", *resultCloud);

}