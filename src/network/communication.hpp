#include <iostream>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "../tracker/pcl_tracking.hpp"
#define BUFFER_SIZE 4096

// Listens to physics engine for incoming PCD and sends
// updated predictions of the point cloud tracker
class Communicator {
private:
    int server_fd, unity_fd;
    int enableOpt = 1;
    int messageCount = 0;
    char buffer[BUFFER_SIZE] = { 0 };

    bool running;

    struct sockaddr_in address;
    VirtualCamera camera;
public:
    bool setUpConnection(int port, std::string& ipAddress);
    void startConnection(FilterParams& params);
    void endConnection();
};
