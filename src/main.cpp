#include <chrono>
#include "pcl_tracking.hpp"

int main() {
// Variables used for tracker
    float downSampleSize = 0.10f;
    int particleCount = 1000;
    double variance = 0.025;
    float delta = 0.99f;
    float epsilon = 0.02f;
    float binSize = 0.1f;
    bool save = false;

    //  File directories to use
    std::string resultDir = ".."; // not used as of now
    std::string experiment = "translation/";
    std::string targetModel = "suzanne";
    std::string truthFileName = "../analysis/results/truth-exp4-suzanne.txt";
    std::string guessFileName = "../analysis/results/guess-exp4-suzanne-p1.txt";
    std::ofstream timingFile;
    timingFile.open("../analysis/results/timing-ds-5.txt"); 

    // Maximum number of frames that are processed
    long maxFrames = 1000;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(resultDir, maxFrames, save);
    tracker.setDownsampleSize(downSampleSize);
    tracker.setUpTracking("../tests/" + experiment + targetModel + "/frame_0.pcd", particleCount, variance, delta, epsilon, binSize);
    tracker.writePredictions(truthFileName, guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = "../tests/" + experiment + targetModel + "/frame_";

    std::chrono::high_resolution_clock::time_point startTime, endTime;

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ((fileNames + std::to_string(tracker.frameCount) + ".pcd"), *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
            return -1; 
        }
        startTime = std::chrono::high_resolution_clock::now();

        tracker.cloudCallBack(cloud);

        endTime = std::chrono::high_resolution_clock::now();
        auto elapsedSeconds = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        std::string out = std::to_string(elapsedSeconds) + "\n";
        timingFile << out;
    }

    return 0;
}