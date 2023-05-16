#include "imu.hpp"
#include <array>

std::array<double, 3> IMU::getPosition() {
    return this->position; 
}

Quaternion IMU::getRotation() {
    return this->rotation; 
}