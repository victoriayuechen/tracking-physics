#include "communication.hpp"

// Connects the communicator to the specified port and gives the target address
bool Updator::setUpConnection(int myPort, int otherPort) {
    // Create own socket
    this->mySocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    // Specify own address and port
    this->my_address.sin_family = AF_INET;
    this->my_address.sin_addr.s_addr = inet_addr(myIP);
    this->my_address.sin_port = htons(myPort);

    // Bind port and own address for receiving/sending
    if ((bind(mySocket, (struct sockaddr*)&this->my_address, sizeof(this->my_address))) < 0) {
        std::cout << "Binding own port and address failed" << std::endl;
        return false;
    }

    // Specify address and port for target (send or receive)
    this->other_address.sin_family = AF_INET;
    this->other_address.sin_addr.s_addr = inet_addr(unity);
    this->other_address.sin_port = htons(otherPort);

    return true;
}

void Updator::endConnection() const {
    shutdown(this->mySocket, SHUT_RDWR);
}

void Communicator::setUpCommunication() {
    bool cloudSetUp = cloudGrabber.setUpConnection(PCD_PCL, PCD_UNITY);
    bool posSetUp = posUpdator.setUpConnection(POS_PCL, POS_UNITY);

    if (!cloudSetUp || !posSetUp) {
        std::cout << "Communication set up failed!" << std::endl;
    }
}

void Communicator::sendPosUpdates() {
    socklen_t targetAddrLen = sizeof(this->posUpdator.other_address);

    while (this->running) {
        auto xyzrpyUpdate = this->camera.getResult();
        char message[MESSAGE_SIZE];
        xyzrpyUpdate.copy(message, strlen(xyzrpyUpdate.c_str()));

        if ((sendto(this->posUpdator.mySocket, message, strlen(message), 0, (struct sockaddr*)&this->posUpdator.other_address, targetAddrLen)) < 0) {
            std::cout << "Didn't send! " << std::endl;
        }
    }
}

void Communicator::getNewPCD() {
    socklen_t targetAddrLen = sizeof(this->cloudGrabber.other_address);
    std::cout << "in getting new PCD" << std::endl; 
    
    while (this->running) {
        memset(this->buffer, 0, sizeof(this->buffer));
        pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);

        if ((recvfrom(this->cloudGrabber.mySocket, this->buffer, sizeof(this->buffer) - 1, 0, (struct sockaddr*)&this->cloudGrabber.other_address, &targetAddrLen)) < 0) {
            std::cerr << "Could not receive the full point cloud successfully" << std::endl;
        }

        // Write PCD to file and continue tracking
        std::string fileName = "../data/playback/frame_" + std::to_string(this->camera.frameCount) + ".pcd";
        std::ofstream pcdFile = std::ofstream(fileName);
        pcdFile << buffer;
        pcdFile.flush(); // TODO: check if there is a better way

        std::cout << "New Message" << std::endl; 

        if (pcl::io::loadPCDFile<RefPointType> (fileName, *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
        }

        this->camera.cloudCallBack(cloud);
    }
}

bool Communicator::initializeFilter(FilterParams &params, std::string initModel, std::string guessFile) {
    // Initialize the camera used for tracking
    this->camera.initializeKLDFilter(params);
    this->camera.setUpTracking(initModel);
    this->camera.writePredictions(guessFile);
    this->camera.save = true;

    this->running = true;

    return true;
}

void Communicator::run() {
   std::thread getClouds = std::thread(&Communicator::getNewPCD, this);
   std::thread sendUpdates = std::thread(&Communicator::sendPosUpdates, this);

   getClouds.detach();
   sendUpdates.detach();
}

void Communicator::stop() {
    this->running = false;
    this->cloudGrabber.endConnection();
    this->posUpdator.endConnection();
}

