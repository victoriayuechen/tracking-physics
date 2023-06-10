#include "communication.hpp"

int main() {
    Communicator communicator = Communicator();
    FilterParams params = {
            0.0,
            2000,
            0.2,
            0.99,
            0.02,
            0.1,
            0.1,
            false
    };
    std::string guessFileName = "../analysis/network-exp/guess-1.txt";

    communicator.setUpCommunication();
    communicator.initializeFilter(params, "../tests/suzanne2000.pcd", guessFileName);
    communicator.run();

    // Continue until Enter is clicked
    std::cin.ignore();

    communicator.stop();

    return 0;
}