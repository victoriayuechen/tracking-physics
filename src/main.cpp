#include "tracker/pcl_tracking.hpp"

int run_experiment(double variance, int numParticles, float downSampleLevel, int count) {
    // Parameters for the filter
    FilterParams params = {
            downSampleLevel,
            numParticles,
            variance,
            0.99,           // delta
            0.02,           // epslion
            0.1,            // bin size
            0.10,           // coherence limit 
            true            // downsample or not 
    };

    // File directories to use
    bool save = true;
    std::string experiment = "../experiments";
    std::string targetType = "/translation";
    std::string guessFileName = "../analysis/experiment-full/translation-guess-" + std::to_string(count) + ".txt";

    // Maximum number of frames that are processed
    long maxFrames = 900;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.initializeKLDFilter(params);
    tracker.setUpTracking("../experiments/suzanne.pcd");
    tracker.writePredictions(guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = experiment + targetType + "/frame_"; 

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ((fileNames + std::to_string(tracker.frameCount) + ".pcd"), *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
            return -1; 
        }
        tracker.cloudCallBack(cloud);
    }

    return 0;
}

int batchExperiment() {
    std::vector<double> variance = { 0.00025, 0.0025, 0.025, 0.10 };
    std::vector<int> numParticles = { 500, 1000, 2000, 3000 };
    std::vector<float> downSampleLevel = { 0.0f, 0.02f, 0.05f, 0.1f };
    int count = 1;

    for (auto const& v : variance) {
        run_experiment(v, 2000, 0.0, count);
        std::cout << "Completed experiment " << count << " / 12" << std::endl;
        count++;
    }

    for (auto const& n : numParticles) {
        run_experiment(0.0025, n, 0.0, count);
        std::cout << "Completed experiment " << count << " / 12" << std::endl;
        count++;
    }

    for (auto const& d : downSampleLevel) {
        run_experiment(0.0025, 2000, d, count);
        std::cout << "Completed experiment " << count << " / 12" << std::endl;
        count++;
    }

    return 0; 
}

int main() {
    batchExperiment();
    // run_experiment(0.0025, 2000, 0.0f, 222);
}