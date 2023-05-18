#include <array>

struct Quaternion {
    double r; 
    double i; 
    double j; 
    double k; 
}; 

class IMU {
    private: 
        std::array<double, 3> position; 
        Quaternion rotation; 
    public:
        std::array<double, 3> getPosition();
        Quaternion getRotation(); 
}; 