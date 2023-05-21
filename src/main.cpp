#include "pcl_tracking.hpp"

int main() {
    // Variables used for tracker
    float downSampleSize = 0.0002f;
    int particleCount = 50;
    double variance = 0.015;
    float delta = 0.99f;
    float epsilon = 0.02f;
    float binSize = 0.001f;
    bool save = false;
    std::string resultDir = "../data/output/output_cloud_";

    // Maximum number of frames that are processed
    long maxFrames = 20;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(resultDir, maxFrames, save);
    tracker.setDownsampleSize(downSampleSize);
    tracker.setUpTracking("../data/frame_0.pcd", particleCount, variance, delta, epsilon, binSize);
    tracker.incrementFrame();

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ("../data/frame_" + std::to_string(tracker.frameCount) + ".pcd", *cloud) == -1) {
            PCL_ERROR ("Could read PCD file \n"); 
            return -1; 
        }
        tracker.cloudCallBack(cloud);
    }

    return 0;
}