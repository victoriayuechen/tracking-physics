#include "tracker/pcl_tracking.hpp"

int main() {
    // Parameters for the filter
    FilterParams params = {
            0.0,
            700,
            0.025,
            0.99,
            0.02,
            0.1,
            0.1,
            false
    };

    //  File directories to use
    bool save = true;
    std::string experiment = "sin-wave/";
    std::string targetModel = "bunny";
    std::string truthFileName = "../analysis/results/truth-noDS1-bunny.txt";
    std::string guessFileName = "../analysis/results/guess-noDS1-bunny.txt";

    // Maximum number of frames that are processed
    long maxFrames = 1000;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.setUpTracking("../tests/" + experiment + targetModel + "/frame_0.pcd", params);
    tracker.writePredictions(truthFileName, guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = "../tests/" + experiment + targetModel + "/frame_";

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ((fileNames + std::to_string(tracker.frameCount) + ".pcd"), *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
            return -1; 
        }
        tracker.cloudCallBack(cloud);
    }

    return 0;
}