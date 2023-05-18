#include "pcl_tracking.hpp"

int main() {
    // Variables used for tracker
    float downSampleSize = 0.02f;
    int particleCount = 50;
    double variance = 0.01;
    float delta = 0.99f;
    float epsilon = 0.02f;
    float binSize = 0.045f;
    bool save = false;
    std::string resultDir = "data/output";

    // Maximum number of frames that are processed
    long maxFrames = 20;
    VirtualCamera tracker = VirtualCamera(); 
    tracker.setDownsampleSize(downSampleSize);
    tracker.setUpCamera("../data/frame_0.pcd", particleCount, variance, delta, epsilon, binSize, save, resultDir);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ("../data/frame_" + std::to_string(tracker.frameCount) + ".pcd", *cloud) == -1) {
            PCL_ERROR ("Could read PCD file \n"); 
            return -1; 
        }
        std::cout << "Iteration in while loop " << tracker.frameCount << std::endl;
        tracker.cloudCallBack(cloud); 
    }

    return 0;
}