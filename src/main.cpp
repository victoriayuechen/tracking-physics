#include "pcl_tracking.hpp"

int main() {
    // Variables used for tracker
    float downSampleSize = 0.02f;
    int particleCount = 700;
    // Initial variance = 0.01
    double variance = 0.01;
    float delta = 0.99f;
    float epsilon = 0.02f;
    float binSize = 0.1f;
    bool save = true;

    //  File directories to use
    std::string resultDir = ".."; // not used as of now
    std::string experiment = "translation";
    std::string targetModel = "monke";
    std::string truthFileName = "../analysis/results/truth-monke.txt";
    std::string guessFileName = "../analysis/results/guess-monke.txt";

    // Maximum number of frames that are processed
    long maxFrames = 1000;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(resultDir, maxFrames, save);
    tracker.setDownsampleSize(downSampleSize);
    //tracker.setUpTracking("../tests/" + experiment + targetModel + "/frame_0.pcd", particleCount, variance, delta, epsilon, binSize);
    tracker.setUpTracking("../data/frame_0.pcd", particleCount, variance, delta, epsilon, binSize);
    tracker.writePredictions(truthFileName, guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    //std::string fileNames = "../tests/" + experiment + targetModel + "/frame_";
    std::string fileNames = "../data/frame_";

    // Initialize time variables to 0
    tracker.setupTime = 0.0;
    tracker.trackingTime = 0.0;

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ((fileNames + std::to_string(tracker.frameCount) + ".pcd"), *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
            return -1; 
        }
        tracker.cloudCallBack(cloud);
    }

    // Print elapsed tracking time for now. Might be nicer to save to csv file later.
    printf ("Setup time: %f ms, Tracking time: %f ms, Total time: %f ms for %d particles and %ld frames\n", tracker.setupTime, tracker.trackingTime, tracker.setupTime + tracker.trackingTime, particleCount, maxFrames);
    printf ("That is an average of %f ms per frame\n", (tracker.setupTime + tracker.trackingTime) / maxFrames);

    return 0;
}