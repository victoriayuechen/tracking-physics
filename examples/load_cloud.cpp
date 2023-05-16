#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/frame_" + std::to_string(0) + ".pcd", *cloud) == -1) {
        PCL_ERROR ("Could read PCD file \n"); 
        return -1; 
    }

    std::cout << "Loaded cloud" << std::endl; 

    for (const auto& p : *cloud) {
        std::cout << "p: " << p.x << " " << p.y << " " << p.z << std::endl; 
    } 
    
    return 0; 
}