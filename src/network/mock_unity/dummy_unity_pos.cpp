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

    // Keep trying to receive position data
    while (true) {
        // Receive updates from PCL
        int bytesRead = recvfrom(unitySocket, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&serverAddress, &serverAddressLength);
        if (bytesRead == -1) {
            std::cout << "Failed to receive data from server." << std::endl;
            return -1;
        }
        std::cout << "Received from PCL tracking: " << buffer << std::endl;
    }

    return 0;
}
