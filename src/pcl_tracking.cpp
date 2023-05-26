#include "pcl_tracking.hpp"
#include <functional>

using namespace pcl::tracking;

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

void BaseTracker::setDownsampleSize(float size) {
    this->downsampleGridSize = size; 
}

void BaseTracker::setMaxFrame(long maxFrame) {
    this->frameMax = maxFrame;
}

void BaseTracker::downSample() {
    // Down sample the cloud
    pcl::ApproximateVoxelGrid<RefPointType> grid;
    grid.setLeafSize(downsampleGridSize, downsampleGridSize, downsampleGridSize);
    grid.setInputCloud(this->objectCloud);
    grid.filter(*this->objectCloud);
}

Eigen::Matrix4f BaseTracker::changeBasis(pcl::PointCloud<RefPointType>::Ptr &cloud) {
    pcl::PointCloud<RefPointType>::Ptr downSampleFloor(new pcl::PointCloud<RefPointType>());
    pcl::VoxelGrid<RefPointType> downSampleGrid;
    downSampleGrid.setInputCloud(cloud);
    downSampleGrid.setLeafSize(0.02f, 0.02f, 0.02f);
    downSampleGrid.filter(*downSampleFloor);
    // pcl::transformPointCloud(*cloud, *downSampleFloor, Eigen::Matrix4f::Identity());

    // Find floor normals
    pcl::NormalEstimationOMP<RefPointType, pcl::Normal> normal_estimation_floor;
    pcl::search::KdTree<RefPointType>::Ptr kdtree (new pcl::search::KdTree<RefPointType>);
    pcl::PointCloud<pcl::Normal>::Ptr floor_normals (new pcl::PointCloud<pcl::Normal>);
    normal_estimation_floor.setInputCloud (downSampleFloor);
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
}

// Creates the vector of step covariances, according to the 6 DOF
std::vector<double> getStepNoiseCovariance(double variance) {
    std::vector<double> defaultStepCovariance = std::vector<double>(6, variance);
    defaultStepCovariance[3] *= 40;
    defaultStepCovariance[4] *= 40;
    defaultStepCovariance[5] *= 40;

    return defaultStepCovariance;
}

// Initializes the KLD filter to be used
void BaseTracker::initializeKLDFilter(float binSize, int numParticles, double variance, float delta, float epsilon) {
    this->delta = delta;
    this->epsilon = epsilon;

    // Initialise KLD Filter
    KLDAdaptiveParticleFilterOMPTracker<RefPointType, Particle>::Ptr kldFilter(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, Particle> (N_THREADS));

    // Set tracker to be the KLD Filter
    this->tracker = kldFilter;
    
    // Meta-parameters for KLD sampling algorithm, dictates the properties of sampling distribution
    Particle bin;
    bin.x = bin.y = bin.z = bin.roll = bin.yaw = bin.pitch = binSize;
    this->tracker->setMaximumParticleNum(numParticles);
    this->tracker->setDelta(this->delta);
    this->tracker->setEpsilon(this->epsilon);
    this->tracker->setBinSize(bin);

    // Parameters for KLD in each iteration
    this->tracker->setTrans(Eigen::Affine3f::Identity());
    this->tracker->setIterationNum(1);
    this->tracker->setParticleNum(numParticles);
    this->tracker->setResampleLikelihoodThr(0.0);
    this->tracker->setUseNormal(false);

    // Parameters used when resampling new particles
    this->tracker->setInitialNoiseCovariance(initialNoiseCovariance);
    this->tracker->setInitialNoiseMean(initialNoiseMean);
    this->tracker->setStepNoiseCovariance(getStepNoiseCovariance(variance));
}

// Sets up the tools needed for tracking: KLD filter, downsampling, NN search
void BaseTracker::setUpTracking(const std::string& modelLoc,
                                int numParticles,
                                double variance,
                                float delta,
                                float epsilon,
                                float binSizeDimensions) {
    // Initialize the KLD Filter
    this->initializeKLDFilter(binSizeDimensions, numParticles, variance, delta, epsilon);

    // Get the initial target cloud
    this->objectCloud.reset(new pcl::PointCloud<RefPointType>());
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(modelLoc, *objectCloud) == -1) {
        std::cout << "File " << modelLoc << " could not be found" << std::endl;
        exit(-1);
    }

    // Set up coherence for point cloud tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence (new ApproxNearestPairPointCloudCoherence<RefPointType>);
    DistanceCoherence<RefPointType>::Ptr distCoherence (new DistanceCoherence<RefPointType>);
    pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (COHERENCE_LIMIT));
    coherence->setTargetCloud(this->objectCloud);
    coherence->addPointCoherence(distCoherence);
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(COHERENCE_LIMIT);

    // Prepare the model of the tracker's target
    Eigen::Vector4f objectCentroid; 
    Eigen::Affine3f translateModel = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid<RefPointType>(*this->objectCloud, objectCentroid);
    translateModel.translation().matrix() = Eigen::Vector3f(objectCentroid[0], objectCentroid[1], objectCentroid[2]);
    pcl::transformPointCloud<RefPointType>(*this->objectCloud, *this->objectCloud, translateModel.inverse());

    // Downsample the target cloud
   // downSample(); // Down sample the cloud
    pcl::ApproximateVoxelGrid<RefPointType> grid;
    grid.setLeafSize(downsampleGridSize, downsampleGridSize, downsampleGridSize);
    grid.setInputCloud(this->objectCloud);
    grid.filter(*this->objectCloud);

    // Set params for KLD Filter
    this->tracker->setCloudCoherence(coherence);
    this->tracker->setTrans(translateModel);
    this->tracker->setReferenceCloud(this->objectCloud);
}

// Removes the cloud of the floor from the cloud of the object
void BaseTracker::runRANSAC(const pcl::PointCloud<RefPointType>::ConstPtr &cloud) {
    // Identify the floor plane
    this->floorPoints.reset(new pcl::PointIndices);
    this->floorCoefficients.reset(new pcl::ModelCoefficients); 
    this->floorSegmentation.setOptimizeCoefficients(true); 
    this->floorSegmentation.setModelType(pcl::SACMODEL_PLANE);
    this->floorSegmentation.setMethodType(pcl::SAC_RANSAC);
    this->floorSegmentation.setMaxIterations(1000);
    this->floorSegmentation.setDistanceThreshold(0.02);
    this->floorSegmentation.setInputCloud(cloud);
    this->floorSegmentation.segment(*this->floorPoints, *this->floorCoefficients);

    // Separate the floor from object
    this->objectCloud.reset(new pcl::PointCloud<RefPointType>()); 
    this->floorObjectSegment.setInputCloud(cloud);
    this->floorObjectSegment.setIndices(this->floorPoints);
    this->floorCloud.reset(new pcl::PointCloud<RefPointType>()); 
    this->floorObjectSegment.setNegative(false); 
    this->floorObjectSegment.filter(*this->floorCloud);
}

void BaseTracker::cloudCallBack(const pcl::PointCloud<RefPointType>::ConstPtr &cloud) {
    // Run RANSAC for segmentation, do only once
    if (this->frameCount < 11) {
        this->frameCount++;
        this->runRANSAC(cloud);

        this->cameraCorrection = changeBasis(this->floorCloud);
        return;
    }

    this->trackingMutex.lock();
    this->objectCloud.reset(new pcl::PointCloud<RefPointType>());

    // Set the target cloud, only identity transformation needed (for now)
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    pcl::transformPointCloud(*cloud, *this->objectCloud, this->cameraCorrection);

    // Filter along a specified dimension
    pcl::PassThrough<RefPointType> pass;
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0f, -0.008f);
    pass.setInputCloud(cloud);
    pass.filter(*this->objectCloud);

    // Down sampling
    pcl::ApproximateVoxelGrid<RefPointType> grid;
    grid.setLeafSize(downsampleGridSize, downsampleGridSize, downsampleGridSize);
    grid.setInputCloud(this->objectCloud);
    grid.filter(*this->objectCloud);

    // Update the cloud being tracked 
    this->tracker->setInputCloud(this->objectCloud);
    this->tracker->compute();

    // Save the output if necessary
    if (this->save) {
        savePointCloud();
    }

    this->frameCount++;
    this->trackingMutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BaseTracker::getParticles() {
    // Save the particle cloud at this frame
    ParticleFilter::PointCloudStatePtr particles = this->tracker->getParticles();
    pcl::PointCloud<pcl::PointXYZ>::Ptr particleCloud (new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& p : *particles) {
        pcl::PointXYZ point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        particleCloud->push_back(point);
    }

    return particleCloud;
}

// Saves the point cloud
void BaseTracker::savePointCloud() {
    std::cout << " == " << this->frameCount << " == " << std::endl;

    // Gets the particle XYZRPY (centroid of particle cloud)
    Particle state = this->tracker->getResult();
    {
        std::cout << "Guess: " << state.x << "," << state.y << "," << state.z << ", " << state.roll << "," << state.pitch << "," << state.yaw << std::endl;

//        // Write predicted centroids to file
//        std::string out = std::to_string(state.x) + "," + std::to_string(state.y) + "," + std::to_string(state.z) + "\n";
//        this->guessOutput << out;
    }

    auto center = getCenter(objectCloud);
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

// Sets the files in which to write the predictions and truth values
void BaseTracker::writePredictions(std::string &truthFile, std::string &guessFile) {
    this->truthOutput.open(truthFile);
    this->guessOutput.open(guessFile);
}

void VirtualCamera::incrementFrame() {
    this->frameCount++; 
}

// Set up for finding the frames of PCD files
void VirtualCamera::initializeCamera(std::string& outputDir, long frameMax, bool save) {
    this->frameCount = 0L;
    this->frameMax = frameMax;
    this->save = save;
    this->outputDir = outputDir;
}
