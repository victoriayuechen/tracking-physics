#include <chrono>
#include "tracker/pcl_tracking.hpp"

int time_experiments(int numParticles, int res, int count, int run) {
    double variance = 0.005; 
    if (res == 2) {
        variance = 0.008; 
    }
    // Parameters for the filter
    FilterParams params = {
            0.02f,
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
    std::ofstream timingFile; 
    std::string guessFileName = "../analysis/experiment-full/runs/" + std::to_string(run) + "-res" + std::to_string(res) + "-" + std::to_string(count) + ".txt"; 
    std::string timeFileName = "../analysis/experiment-full/runs/" + std::to_string(run) + "-res" + std::to_string(res) + "-timing-" + std::to_string(count) + ".txt"; 
    timingFile.open(timeFileName); 

    // Maximum number of frames that are processed
    long maxFrames = 500;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.initializeKLDFilter(params);
    tracker.setUpTracking("../tests/suzanne2000.pcd");
    tracker.writePredictions(guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = "../experiments/res" + std::to_string(res) +"/frame_";
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
    std::string guessFileName = "../analysis/experiment-full/sinwave-" + std::to_string(count) + ".txt";

    // Maximum number of frames that are processed
    long maxFrames = 900;

    // Start up the tracker
    VirtualCamera tracker = VirtualCamera();
    tracker.initializeCamera(maxFrames, save);
    tracker.initializeKLDFilter(params);
    tracker.setUpTracking("../tests/suzanne2000.pcd");
    tracker.writePredictions(guessFileName);

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
    std::string fileNames = "../experiments/sin/frame_";

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
    std::vector<double> variance = { 0.001, 0.005, 0.05, 0.08 };
    std::vector<int> numParticles = { 500, 1000, 2000, 3000 };
    std::vector<int> partCount = { 100, 600 ,1000, 1500, 2000 }; // 100, 600 ,1000, 1500, 2000 
    std::vector<float> downSampleLevel = { 0.01f, 0.02f, 0.06f, 0.1f };
    // int count = 1;

    // for (int n : partCount) { 
    //     time_experiments(n, count); 
    //     std::cout << "Completed experiment " << count << " / 5" << std::endl; 
    //     count++;
    // }

    // For all of the possible camera resolutions
    for (int r = 3; r <= 5; r++) {
        for (int i = 1; i <= 5; i++) {
            int count = 1; 
            // Analyse the time and accuracy given a specific particle count
            for (int n : partCount) {
                // Run 1 
                // time_experiments(int numParticles, int res, int count, int run) 
                time_experiments(n, i, count, r); 
                std::cout << "Resolution " << i << ", completed " << count << " / 5" << std::endl; 
                count++;
            }
        }
    }

    // for (double v : variance) {
    //     run_experiment(v, 2000, 0.02f, count);
    //     std::cout << "Completed experiment " << count << " / 12" << std::endl;
    //     count++;
    // }

    // for (int n : numParticles) {
    //     run_experiment(0.08, n, 0.02f, count);
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
    batchExperiment();
    // run_experiment(0.08, 100, 0.02f, 100);
    // time_experiments(600, 2); 
}