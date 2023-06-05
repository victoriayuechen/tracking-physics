#include <iostream>
#include <fstream>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "../tracker/pcl_tracking.hpp"
#define BUFFER_SIZE 4096
#define MESSAGE_SIZE 200

// PCL sends updates here
#define POS_PCL 8080
// Unity grabs updates from here
#define POS_UNITY 8081
// PCL grabs clouds from here
#define PCD_PCL 8082
// Unity sends clouds here
#define PCD_UNITY 8083
// The IP used by all
#define localhost "127.0.0.1"


/**
 * Listens to physics engine for incoming PCD and sends
 * updated predictions of the point cloud tracker
 */
class Updator {
public:
    int mySocket;
    struct sockaddr_in other_address, my_address;
    bool setUpConnection(int myPort, int otherPort);
    void endConnection() const;
};


class Communicator {
private:
    VirtualCamera camera;
    Updator cloudGrabber;
    Updator posUpdator;
    bool running;
    char buffer[BUFFER_SIZE];
public:
    void setUpCommunication();
    void sendPosUpdates();
    void getNewPCD();
    bool initializeFilter(FilterParams& params);
    void run();
    void stop();
};