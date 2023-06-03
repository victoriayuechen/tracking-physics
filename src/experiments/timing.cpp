#include <chrono>
#include "../tracker/pcl_tracking.hpp"

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
    bool save = false;
    std::string resultDir = ".."; // not used as of now
    std::string experiment = "translation/";
    std::string targetModel = "suzanne";
    std::ofstream timingFile;
    timingFile.open("../analysis/results/timing-ds-5.txt");

    // Maximum number of frames that are processed
    long maxFrames = 1000;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.initializeKLDFilter(params);
    tracker.setUpTracking("../tests/" + experiment + targetModel + "/frame_0.pcd");

    // Load all frames
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = "../tests/" + experiment + targetModel + "/frame_";

    std::chrono::high_resolution_clock::time_point startTime, endTime;

    while (tracker.frameCount < maxFrames) {
        std::cout << "=== Frame " << tracker.frameCount << ": ===" << std::endl;

        if (pcl::io::loadPCDFile<RefPointType> ((fileNames + std::to_string(tracker.frameCount) + ".pcd"), *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
            return -1;
        }
        // Time the tracking
        startTime = std::chrono::high_resolution_clock::now();
        tracker.cloudCallBack(cloud);
        endTime = std::chrono::high_resolution_clock::now();

        // Save timing in (ms)
        auto elapsedSeconds = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        std::string out = std::to_string(elapsedSeconds) + "\n";
        timingFile << out;
    }

    return 0;
}