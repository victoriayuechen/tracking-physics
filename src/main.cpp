#include <chrono>
#include "tracker/pcl_tracking.hpp"

int time_experiments(int numParticles, int count) {
    // Parameters for the filter
    FilterParams params = {
            0.02f,
            numParticles,
            0.005,
            0.99,           // delta
            0.02,           // epslion
            0.1,            // bin size
            0.10,           // coherence limit 
            true           // downsample or not 
    };

    //  File directories to use
    bool save = false;
    std::ofstream timingFile; 
    timingFile.open( "../analysis/experiment-full/timing-" + std::to_string(count) + ".txt"); 

    // Maximum number of frames that are processed
    long maxFrames = 600;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.initializeKLDFilter(params);
    tracker.setUpTracking("../tests/suzanne2000.pcd");

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = "../experiments/translation/frame_";
    std::chrono::high_resolution_clock::time_point startTime, endTime;


    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<RefPointType> ((fileNames + std::to_string(tracker.frameCount) + ".pcd"), *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
            return -1; 
        }
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
            true           // downsample or not 
    };

    //  File directories to use
    bool save = true;
    std::string guessFileName = "../analysis/experiment-full/translation-" + std::to_string(count) + ".txt";

    // Maximum number of frames that are processed
    long maxFrames = 600;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.initializeKLDFilter(params);
    tracker.setUpTracking("../tests/suzanne2000.pcd");
    tracker.writePredictions(guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = "../experiments/translation/frame_";

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
    std::vector<double> variance = { 0.08 };
    std::vector<int> numParticles = { 300, 800, 1000, 1500, 2000, 3000 };
    std::vector<float> downSampleLevel = { 0.01f, 0.02f, 0.06f, 0.1f };
    int count = 1;

    for (int n : numParticles) {
        time_experiments(n, count); 
        std::cout << "Completed experiment " << count << " / 6" << std::endl; 
        count++;  
    }

    // for (double v : variance) {
    //     run_experiment(v, 2000, 0.02f, count);
    //     std::cout << "Completed experiment " << count << " / 12" << std::endl;
    //     count++;
    // }

    // for (int n : numParticles) {
    //     run_experiment(0.008, n, 0.02f, count);
    //     std::cout << "Completed experiment " << count << " / 12" << std::endl;
    //     count++;
    // }

    // for (float d : downSampleLevel) {
    //     run_experiment(0.08, 2000, d, count);
    //     std::cout << "Completed experiment " << count << " / 12" << std::endl;
    //     count++;
    // }

    return 0; 
}

int main() {
    // batchExperiment();
    run_experiment(0.005, 2000, 0.02f, 222);
}