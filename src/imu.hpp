#include <array>

struct Quaternion {
    
}; 

class IMU {
    private: 
        std::array<double, 3> position; 
        Quaternion rotation; 
    public:
        std::array<double, 3> getPosition();
        Quaternion getRotation(); 
}; 