#include "pcl_tracking.hpp"

using namespace pcl::tracking;

void BaseTracker::setDownsampleSize(float size) {
    this->downsampleGridSize = size; 
}

void BaseTracker::initializeKLDFilter(float binSize, int numParticles, float delta, float epsilon) {
    this->delta = delta;
    this->epsilon = epsilon;

    // Initialise KLD Filter
    KLDAdaptiveParticleFilterOMPTracker<RefPointType, Particle>::Ptr kldFilter (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, Particle> (N_THREADS));

    // Parameters when resampling new particles
    kldFilter->setInitialNoiseCovariance(initialNoiseCovariance);
    kldFilter->setInitialNoiseMean(initialNoiseMean);
    kldFilter->setStepNoiseCovariance(defaultStepCovariance);

    // Meta-parameters for KLD sampling algorithm
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

void BaseTracker::setUpTracking(const std::string& modelLoc,
                                int numParticles,
                                double variance,
                                float delta,
                                float epsilon,
                                float binSizeDimensions) {
    // Initialize the KLD Filter
    this->initializeKLDFilter(binSizeDimensions, numParticles, delta, epsilon);

    // Set up coherence for point cloud tracking 
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence (new ApproxNearestPairPointCloudCoherence<RefPointType>);
    DistanceCoherence<RefPointType>::Ptr distCoherence (new DistanceCoherence<RefPointType>);
    coherence->addPointCoherence(distCoherence); 

    // Octree class for nearest neighbour search
    pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (COHERENCE_LIMIT)); 
    coherence->setSearchMethod(search); 
    coherence->setMaximumDistance(COHERENCE_LIMIT); 

    // Get target cloud
    pcl::PointCloud<RefPointType>::Ptr objectModel (new pcl::PointCloud<RefPointType>());
    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(modelLoc, *objectModel) == -1) {
        std::cout << "File " << modelLoc << " could not be found" << std::endl;
        exit(-1);
    }
    this->objectCloud = objectModel;

    // Prepare the model of the tracker's target
    Eigen::Vector4f objectCentroid; 
    Eigen::Affine3f translateModel = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid<RefPointType>(*objectModel, objectCentroid); 
    translateModel.translation().matrix() = Eigen::Vector3f(objectCentroid[0], objectCentroid[1], objectCentroid[2]);
    pcl::transformPointCloud<RefPointType>(*objectModel, *objectModel, translateModel.inverse()); 

    // Downsample the target cloud
    pcl::PointCloud<RefPointType>::Ptr transformedDownCloud(new pcl::PointCloud<RefPointType>); 
    this->downSampleGrid.setInputCloud(objectModel); 
    this->downSampleGrid.setLeafSize(this->downsampleGridSize, this->downsampleGridSize, this->downsampleGridSize); 
    this->downSampleGrid.filter(*transformedDownCloud); 

    // Set params for KLD Filter 
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
    // Ignore first 10 frames because of noise 
    if (this->frameCount < 10) {
        this->frameCount++;
        return;
    }
//    // Run RANSAC for segmentation, only if fewer than 11 frames
//    // May be unnecessary if no floor plane exists
//    if (frameCount < 11) {
//        this->runRANSAC(cloud);
//    }

    // Set the target cloud, no transformation needed in this case
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    pcl::transformPointCloud(*cloud, *this->objectCloud, identity);

    // Filter along a specified dimension,
    // This limits what is kept for tracking, removes corner points we don't care about.      
    pcl::PassThrough<RefPointType> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 10.0); // how to determine these limits?
    pass.setInputCloud(this->objectCloud);
    pass.filter(*this->objectCloud);

    // Down sampling
    this->downSampleGrid.setInputCloud(this->objectCloud); 
    this->downSampleGrid.filter(*this->objectCloud);

    // Update the cloud being tracked 
    this->trackingMutex.lock();
    this->tracker->setInputCloud(this->objectCloud); 
    this->trackingMutex.unlock();
    this->frameCount++;
}

void BaseTracker::tracking() {
    if (this->frameCount < 11) {
        std::this_thread::sleep_for(0.03s); 
    }

    // Compute and store the XYZRPY of the cloud 
    this->objPosRot = this->tracker->getResult();
    this->trackingMutex.lock();
    this->dof[0] = objPosRot[0]; 
    this->dof[1] = objPosRot[1]; 
    this->dof[2] = objPosRot[2]; 
    this->dof[3] = objPosRot[3]; 
    this->dof[4] = objPosRot[4]; 
    this->dof[5] = objPosRot[5]; 
    this->trackingMutex.unlock(); 
}

std::array<double, 6> BaseTracker::getDOF() {
    return this->dof; 
}

void BaseTracker::savePointCloud() {
    pcl::io::savePCDFileASCII("pcl_frame_" + std::to_string(this->frameCount) + ".pcd", *this->objectCloud);
}

void BaseTracker::incrementFrame() {
    this->frameCount++; 
}

// Set up for finding the frames of PCD files
void VirtualCamera::setUpCameraListener(std::string videoLoc) {
    this->frameCount = 0L; 
    this->dataPath = videoLoc; 
} 

void VirtualCamera::startCameraListener(bool video, bool save) {
    std::cout << "Start camera listener" << std::endl; 
    std::cout << "Add saving flag here later." << std::endl; 
}

void VirtualCamera::setUpCamera(std::string modelLoc,
                                int numParticles,
                                double variance,
                                float delta,
                                float epsilon,
                                float binSize,
                                bool save,
                                std::string resultLoc)
                                {
    this->setUpTracking(modelLoc, numParticles, variance, delta, epsilon, binSize);

    this->setUpCameraListener(resultLoc); 
    // useless call
    this->startCameraListener(true, true); 
}

void VirtualCamera::stopCameraListener(bool video) {
    std::cout << "Stopped camera listener (remove this method later) " << std::endl;
}