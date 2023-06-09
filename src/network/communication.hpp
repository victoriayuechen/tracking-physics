#include <iostream>
#include <fstream>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "../tracker/pcl_tracking.hpp"

// BUFFER is for receiving PCDs and MESSAGE is for sending position updates
#define BUFFER_SIZE 40000
#define MESSAGE_SIZE 20

// PCL sends updates here
#define POS_PCL 8080
// Unity grabs updates from here
#define POS_UNITY 8080
// PCL grabs clouds from here
#define PCD_PCL 8081
// Unity sends clouds here
#define PCD_UNITY 8081
// The IP used by all
#define myIP "172.22.78.131"
// The IP used by windows
#define unity "172.22.64.1"

/**
 * Class for setting up communication endpoint with Unity
 * Has own port and the target port through which it can send/receive data
 * using the UDP protocol.
 */
class Updator {
public:
    int mySocket;
    struct sockaddr_in other_address, my_address;
    bool setUpConnection(int myPort, int otherPort);
    void endConnection() const;
};

/**
 * Responsible for communication loop with Unity for tracking.
 * Takes PCDs from the Unity engine and tracks it
 * Two channels for receiving PCDs and for sending position updates
 */
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
    bool initializeFilter(FilterParams &params, std::string initModel, std::string guessFile);
    void run();
    void stop();
};