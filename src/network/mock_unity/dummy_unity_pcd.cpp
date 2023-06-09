#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>

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
    serverAddress.sin_addr.s_addr = inet_addr("172.18.128.1");  // Connect to localhost
    serverAddress.sin_port = htons(8080);                   // Use the same port as the server

    // Bind the socket to a specific address and port (Address of Unity)
    sockaddr_in unityAddress{};
    unityAddress.sin_family = AF_INET;
    unityAddress.sin_addr.s_addr = inet_addr("172.18.135.177"); // Bind to any available network interface
    unityAddress.sin_port = htons(8080);

    if (bind(unitySocket, (struct sockaddr*)&unityAddress, sizeof(unityAddress)) == -1) {
        std::cout << "Failed to bind socket." << std::endl;
        return -1;
    }

    // Receive response from server
    socklen_t serverAddressLength = sizeof(serverAddress);

    std::ifstream inputCloud("../data/test.txt");
    std::string inputContents((std::istreambuf_iterator<char>(inputCloud)),
                             std::istreambuf_iterator<char>());
    auto cloudData = inputContents.c_str();

    auto message = "hello from pcl";

    // Keep sending frames
    while (true) {
        if ((sendto(unitySocket, message, strlen(message), 0, (struct sockaddr*)&serverAddress, serverAddressLength)) <= 0) {
            std::cout << "Couldn't send to PCL tracking program! " << std::endl;
            break;
        }
    }

    // Close the socket
    close(unitySocket);

    return 0;
}
