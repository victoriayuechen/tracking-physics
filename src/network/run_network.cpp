#include "communication.hpp"

int main() {
    Communicator communicator = Communicator();
    FilterParams params = {
            0.0,
            700,
            0.025,
            0.99,
            0.02,
            0.1,
            0.1,
            false
    };

    communicator.setUpCommunication();
    communicator.initializeFilter(params);
    communicator.run();

    // Continue until Enter is clicked
    std::cin.ignore();

    communicator.stop();

    return 0;
}