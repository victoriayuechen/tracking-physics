#include "pcl_tracking.hpp"
using namespace pcl::tracking;

void BaseTracker::setDownsampleSize(float size) {
    this->downsampleGridSize = size; 
}

// Creates the vector of step covariances, according to the 6 DOF
std::vector<double> getStepNoiseCovariance(double variance) {
    double covariance = variance * variance;
    std::vector<double> defaultStepCovariance = std::vector<double>(6, covariance);
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

    // Parameters used when resampling new particles
    kldFilter->setInitialNoiseCovariance(initialNoiseCovariance);
    kldFilter->setInitialNoiseMean(initialNoiseMean);
    kldFilter->setStepNoiseCovariance(getStepNoiseCovariance(variance));

    // Meta-parameters for KLD sampling algorithm, dictates the properties of sampling distribution
    Particle bin;
    bin.x = bin.y = bin.z = bin.roll = bin.yaw = bin.pitch = binSize;

    kldFilter->setMaximumParticleNum(numParticles);
    kldFilter->setDelta(this->delta);
    kldFilter->setEpsilon(this->epsilon);
    kldFilter->setBinSize(bin);

    // Parameters for KLD in each iteration
    kldFilter->setIterationNum(1);
    kldFilter->setParticleNum(numParticles);
    kldFilter->setResampleLikelihoodThr(0.0);
    kldFilter->setUseNormal(false);

    // Set tracker to be the KLD Filter
    this->tracker = kldFilter;
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

    // Set up coherence for point cloud tracking 
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence (new ApproxNearestPairPointCloudCoherence<RefPointType>);
    DistanceCoherence<RefPointType>::Ptr distCoherence (new DistanceCoherence<RefPointType>);
    pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (COHERENCE_LIMIT));
    // This came before initializing the octree, but order probably shouldn't matter
    coherence->addPointCoherence(distCoherence);
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(COHERENCE_LIMIT); 

    // Get the initial target cloud
    pcl::PointCloud<RefPointType>::Ptr initModel (new pcl::PointCloud<RefPointType>());
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(modelLoc, *initModel) == -1) {
        std::cout << "File " << modelLoc << " could not be found" << std::endl;
        exit(-1);
    }
    this->objectCloud = initModel;

    // Prepare the model of the tracker's target
    Eigen::Vector4f objectCentroid; 
    Eigen::Affine3f translateModel = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid<RefPointType>(*this->objectCloud, objectCentroid);
    translateModel.translation().matrix() = Eigen::Vector3f(objectCentroid[0], objectCentroid[1], objectCentroid[2]);
    pcl::transformPointCloud<RefPointType>(*this->objectCloud, *this->objectCloud, translateModel.inverse());

    // Downsample the target cloud
    pcl::PointCloud<RefPointType>::Ptr transformedDownCloud(new pcl::PointCloud<RefPointType>); 
    this->downSampleGrid.setInputCloud(this->objectCloud);
    this->downSampleGrid.setLeafSize(this->downsampleGridSize, this->downsampleGridSize, this->downsampleGridSize); 
    this->downSampleGrid.filter(*transformedDownCloud); 

    // Set params for KLD Filter
    this->tracker->setCloudCoherence(coherence);
    this->tracker->setTrans(translateModel);
    this->tracker->setReferenceCloud(transformedDownCloud);
}

// Removes the cloud of the floor from the cloud of the object
void BaseTracker::runRANSAC(const pcl::PointCloud<RefPointType>::ConstPtr &cloud) {
    this->floorPoints.reset(new pcl::PointIndices);
    this->floorCoefficients.reset(new pcl::ModelCoefficients); 
    this->floorSegmentation.setOptimizeCoefficients(true); 
    this->floorSegmentation.setModelType(pcl::SACMODEL_PLANE);
    this->floorSegmentation.setMethodType(pcl::SAC_RANSAC);
    this->floorSegmentation.setMaxIterations(1000);

    this->objectCloud.reset(new pcl::PointCloud<RefPointType>()); 
    this->floorObjectSegment.setInputCloud(cloud);
    this->floorObjectSegment.setIndices(this->floorPoints);
    
    this->floorCloud.reset(new pcl::PointCloud<RefPointType>()); 
    this->floorObjectSegment.setNegative(false); 
    this->floorObjectSegment.filter(*this->floorCloud);
}

void BaseTracker::cloudCallBack(const pcl::PointCloud<RefPointType>::ConstPtr &cloud) {
//    // Run RANSAC for segmentation, do only once
//    // May be unnecessary if no floor plane exists
//    if (frameCount < 2) {
//        this->runRANSAC(cloud);
//    }

    // Set the target cloud, only identity transformation needed (for now)
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    pcl::transformPointCloud(*cloud, *this->objectCloud, identity);

    // Filter along a specified dimension
    // TODO: define specific bounding box in which this object exists
    pcl::PassThrough<RefPointType> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-10, 10.0); // how to determine these limits?
    pass.setInputCloud(this->objectCloud);
    pass.filter(*this->objectCloud);

    // Down sampling
    this->downSampleGrid.setInputCloud(this->objectCloud);
    this->downSampleGrid.filter(*this->objectCloud);

    // Update the cloud being tracked 
    this->tracker->setInputCloud(this->objectCloud);

    // Save the output if necessary
    if (this->save) {
        savePointCloud();
    }

    this->frameCount++;
}

// Saves the point cloud
void BaseTracker::savePointCloud() {
    Particle state = this->tracker->getResult();
    {
        std::cout << "Frame " << this->frameCount << " : " << state.x << " " << state.y << " " << state.z << std::endl;
    }

   //pcl::io::savePCDFileASCII(outputDir + std::to_string(this->frameCount) + ".pcd", *this->objectCloud);
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
