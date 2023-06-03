#include "communication.hpp"

// Given the buffer, creates a point cloud and saves it to file
// Returns the name of the file that contains the first scan of the cloud
std::string makePointCloud(char* buffer) {

}

// Connects the communicator to the specified port and gives the target address
bool Communicator::setUpConnection(int port, std::string& ipAddress) {
    // Create socket
    this->server_fd = socket(AF_LOCAL, SOCK_STREAM, 0);
    if (this->server_fd < 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        return false;
    }

    // Set options for the socket
    if(!setsockopt(this->server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &this->enableOpt, sizeof(int))) {
        std::cerr << "Setting options for socket failed" << std::endl;
        return false;
    }

    // Bind the socket to the port
    this->address.sin_family = AF_INET;
    this->address.sin_addr.s_addr = inet_addr(ipAddress.c_str());
    this->address.sin_port = htons(port);
    if (bind(this->server_fd, (const struct sockaddr *)&(this->address), sizeof(this->address)) < 0) {
        std::cerr << "Binding to port failed" << std::endl;
        return false;
    }

    this->running = true;

    return true;
}

void Communicator::startConnection(FilterParams& params) {
    this->camera.initializeKLDFilter(params);

    // Start listening to the port
    listen(this->server_fd, 2);

    if ((this->unity_fd = accept(server_fd, (struct sockaddr*)&this->address, (socklen_t*)sizeof(this->address))) < 0) {
        std::cout << "Couldn't accept incoming connections" << std::endl;
        exit(EXIT_FAILURE);
    }

    while (this->running) {
        if (this->messageCount < 1) {
            // Use the first frame (correct initial state) to set up tracking
            read(this->unity_fd, this->buffer, BUFFER_SIZE);
            std::string initModel = makePointCloud(this->buffer);
            this->camera.setUpTracking(initModel);

            this->messageCount++;
        }

        // Load cloud into file, or load in the cloud completely
        read(this->unity_fd, this->buffer, BUFFER_SIZE);
        std::string modelLoc = makePointCloud(this->buffer);
        pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);

        if (pcl::io::loadPCDFile<RefPointType> (modelLoc, *cloud) == -1) {
            PCL_ERROR ("Could not read PCD file \n");
        }
        this->camera.cloudCallBack(cloud);

        // TODO: Make prediction for the position for the next cloud

        this->messageCount++;
    }
}

void Communicator::endConnection() {
    this->running = false;
    shutdown(this->server_fd, SHUT_RDWR);
}