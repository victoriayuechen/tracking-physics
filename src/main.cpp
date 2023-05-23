#include "pcl_tracking.hpp"

int main() {
    // Variables used for tracker
    float downSampleSize = 0.0002f;
    int particleCount = 10000;
    double variance = 0.015;
    float delta = 0.99f;
    float epsilon = 0.02f;
    float binSize = 0.001f;
    bool save = false;
    std::string resultDir = "../data/output/output_cloud_";

    // Maximum number of frames that are processed
    long maxFrames = 50;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(resultDir, maxFrames, save);
    tracker.setDownsampleSize(downSampleSize);
    tracker.setUpTracking("../data/frame_0.pcd", particleCount, variance, delta, epsilon, binSize);
    tracker.incrementFrame();

    // Initialize time variables to 0
    tracker.setupTime = 0.0;
    tracker.trackingTime = 0.0;

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ("../data/frame_" + std::to_string(tracker.frameCount) + ".pcd", *cloud) == -1) {
            PCL_ERROR ("Could read PCD file \n"); 
            return -1; 
        }
        tracker.cloudCallBack(cloud);
    }

    // Print elapsed tracking time for now. Might be nicer to save to csv file later.
    printf ("Setup time: %f ms, Tracking time: %f ms, Total time: %f ms for %d particles\n", tracker.setupTime, tracker.trackingTime, tracker.setupTime + tracker.trackingTime, particleCount);

    return 0;
}