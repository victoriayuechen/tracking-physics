#include "pcl_tracking.hpp"

int main() {
    // Variables used for tracker
    float downSampleSize = 0.0002f;
    int particleCount = 2000;
    double variance = 0.00225;
    float delta = 0.99f;
    float epsilon = 0.02f;
    float binSize = 0.045f;
    bool save = true;
    std::string resultDir = "../data/output/output_cloud_";
    std::string truthFileName = "../analysis/truth.txt";
    std::string guessFileName = "../analysis/guess.txt";

    // Maximum number of frames that are processed
    long maxFrames = 100;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(resultDir, maxFrames, save);
    tracker.setDownsampleSize(downSampleSize);
    tracker.setUpTracking("../translation/frame_0.pcd", particleCount, variance, delta, epsilon, binSize);
    tracker.writePredictions(truthFileName, guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ("../translation/frame_" + std::to_string(tracker.frameCount) + ".pcd", *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
            return -1; 
        }
        tracker.cloudCallBack(cloud);
    }

    return 0;
}