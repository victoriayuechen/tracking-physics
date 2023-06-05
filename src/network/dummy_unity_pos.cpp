#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {
    // Create a socket
    int unitySocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (unitySocket == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return -1;
    }

    // Server address (Address of PCL)
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = inet_addr("127.0.0.1");  // Connect to localhost
    serverAddress.sin_port = htons(8080);                   // Use the same port as the server

    // Bind the socket to a specific address and port (Address of Unity)
    sockaddr_in unityAddress{};
    unityAddress.sin_family = AF_INET;
    unityAddress.sin_addr.s_addr = inet_addr("127.0.0.1"); // Bind to any available network interface
    unityAddress.sin_port = htons(8081);

    if (bind(unitySocket, (struct sockaddr*)&unityAddress, sizeof(unityAddress)) == -1) {
        std::cout << "Failed to bind socket." << std::endl;
        return -1;
    }

    // Receive response from server
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    socklen_t serverAddressLength = sizeof(serverAddress);

    std::string fakeData = "# .PCD v0.7 - Point Cloud Data file format\n"
                           "VERSION 0.7\n"
                           "FIELDS x y z rgb\n"
                           "SIZE 4 4 4 4\n"
                           "TYPE F F F F\n"
                           "COUNT 1 1 1 1\n"
                           "WIDTH 5\n"
                           "HEIGHT 1\n"
                           "VIEWPOINT 0 0 0 1 0 0 0\n"
                           "POINTS 5\n"
                           "DATA ascii\n"
                           "0.35222197 -0.15188313 -0.10639524 1.0\n"
                           "-0.3974061 -0.47310591 0.29260206 1.0\n"
                           "-0.73189831 0.66710472 0.44130373 1.0\n"
                           "-0.73476553 0.85458088 -0.036173344 1.0\n"
                           "-0.46070004 -0.2774682 -0.91676188 1.0";

    // Make it have sequential behaviour first
    int i = 0;

//    // Send initial frame (for set up)
//    char messageBack[strlen(fakeData.c_str())];
//    fakeData.copy(messageBack,strlen(fakeData.c_str()));
//
//    if ((sendto(clientSocket, messageBack, strlen(messageBack), 0, (struct sockaddr*)&serverAddress, serverAddressLength)) < 0) {
//        std::cout << "Didn't send! " << std::endl;
//    }
//
//    i++;

    while (i < 50) {
        // Receive updates from PCL
        int bytesRead = recvfrom(unitySocket, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&serverAddress, &serverAddressLength);
        if (bytesRead == -1) {
            std::cout << "Failed to receive data from server." << std::endl;
            return -1;
        }
        std::cout << "Received from PCL tracking: " << buffer << std::endl;

        // Send the point cloud
//        char messageBack[strlen(fakeData.c_str())];
//        fakeData.copy(messageBack,strlen(fakeData.c_str()));
//
//        if ((sendto(clientSocket, messageBack, strlen(messageBack), 0, (struct sockaddr*)&serverAddress, serverAddressLength)) < 0) {
//            std::cout << "Didn't send! " << std::endl;
//        }
        i++;
    }


    // Close the socket
    close(unitySocket);

    return 0;
}
