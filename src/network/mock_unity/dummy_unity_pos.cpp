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
    serverAddress.sin_addr.s_addr = inet_addr("172.18.128.1");  // Connect to localhost
    serverAddress.sin_port = htons(8081);                   // Use the same port as the server

    // Bind the socket to a specific address and port (Address of Unity)
    sockaddr_in unityAddress{};
    unityAddress.sin_family = AF_INET;
//    unityAddress.sin_addr.s_addr = inet_addr("172.18.135.177"); // Bind to any available network interface
    unityAddress.sin_port = htons(8081);

    if (bind(unitySocket, (struct sockaddr*)&unityAddress, sizeof(unityAddress)) == -1) {
        std::cout << "Failed to bind socket." << std::endl;
        return -1;
    }

    // Receive response from server
    char buffer[40000];
    memset(buffer, 0, sizeof(buffer));
    socklen_t serverAddressLength = sizeof(serverAddress);

    // Keep trying to receive position data
    int i = 0;
    while (i < 3) {
        // Receive updates from PCL
        int bytesRead = recvfrom(unitySocket, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&serverAddress, &serverAddressLength);
        if (bytesRead == -1) {
            std::cout << "Failed to receive data from server." << std::endl;
            return -1;
        }

        char senderIP[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(serverAddress.sin_addr), senderIP, INET_ADDRSTRLEN);
        std::string senderAddress = senderIP;

        // Process the received data
        std::cout << "Received data from sender at " << senderAddress << ": " << std::string(buffer, bytesRead) << std::endl;

        std::cout << "Received from Windows: " << buffer << std::endl;
        i++;
    }

    return 0;
}
