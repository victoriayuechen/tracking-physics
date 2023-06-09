#include "tracker/pcl_tracking.hpp"

int main() {
    // Parameters for the filter
    FilterParams params = {
            0.02,
            2000,
            0.00999,
            0.99,
            0.02,
            0.1,
            0.10,
            false
    };

    //  File directories to use
    bool save = true;
    std::string experiment = "small-test";
    std::string targetModel = "";
    std::string truthFileName = "../analysis/full-model/truth-partial.txt";
    std::string guessFileName = "../analysis/full-model/guess-partial-100frames.txt";

    // Maximum number of frames that are processed
    long maxFrames = 100;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.initializeKLDFilter(params);
    tracker.setUpTracking("../tests/suzanne2000.pcd");
    tracker.writePredictions(guessFileName);

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
    printf ("Setup time: %f ms, Tracking time: %f ms, Total time: %f ms for %d particles and %ld frames\n", tracker.setupTime, tracker.trackingTime, tracker.setupTime + tracker.trackingTime, params.particleCount, maxFrames);
    printf ("That is an average of %f ms per frame\n", (tracker.setupTime + tracker.trackingTime) / maxFrames);

    return 0;
}