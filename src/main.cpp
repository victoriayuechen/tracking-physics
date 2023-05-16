#include "pcl_tracking.hpp"

int main() {
    // define a bunch of variables 

    // Maximum number of frames that are processed
    long maxFrames = 40; 
    VirtualCamera tracker = new VirtualCamera(); 

    // Load all frames 
    pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);

    while (tracker.frameCount < maxFrames) {
        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/frame_" + std::to_string(tracker.frameCount) + ".pcd", *cloud) == -1) {
            PCL_ERROR ("Could read PCD file \n"); 
            return -1; 
        }
        tracker.cloudCallBack(cloud); 
        tracker.incrementFrame(); 
    }

}