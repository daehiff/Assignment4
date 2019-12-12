#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include "opencv.hpp"
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

#include "img_processing.cpp"

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "4000"
#define CAMERA_ID 0

using namespace std;
using namespace cv;

int sendCommand(char *sendbuf, SOCKET &ClientSocket) {
    cout << "Sending Command: " << sendbuf << endl;
    int SendResult = 0;
    int ReceiveResult = 0;
    char recvbuf[512];
    int counter = 0;
    int RoundThreshold = 1000;

    //Send Command to Robot Arm
    SendResult = send(ClientSocket, sendbuf, strlen(sendbuf) + 1, 0);

    if (SendResult == SOCKET_ERROR) {
        cout << "send failed with error: " << WSAGetLastError() << endl;
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

    Sleep(100);

    do {
        ReceiveResult = recv(ClientSocket, recvbuf, 512, 0);
        counter++;
    } while (ReceiveResult == 0 && counter < RoundThreshold);

    if (counter > RoundThreshold) {
        cout << "Respond Time Out" << endl;
        return 1;
    } else {
        if (!strcmp(recvbuf, "ERR")) {
            cout << "Invalid Command" << endl;
            return 1;
        }
    }

    return 0;
}

int __cdecl main(void) {
    WSADATA wsaData;
    int iResult;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    int recvbuflen = DEFAULT_BUFLEN;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind(ListenSocket, result->ai_addr, (int) result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // No longer need server socket
    closesocket(ListenSocket);

    //========== Add your code below ==========//

    // === 1. Read the camera frames and open a window to show it.
    // === 2. Segment the object(s) and calculate the centroid(s) and principle angle(s).
    // === 3. Use prespective transform to calculate the desired pose of the arm.



    // [note] David will include his magic✨🦄 code here to get
    //        from camera frame to the world frame



    // === 4. Move the arm to the grasping pose by sendCommand() function.

    // The following lines give an example of how to send a command.
    // You can find commends in "Robot Arm Manual.pdf"
    char command[] = "GOHOME";
    sendCommand(command, ClientSocket);



    // 5. Control the gripper to grasp the object.
    // The following lines give an example of how to control the gripper.
    char closeGripper[] = "OUTPUT 48 ON";
    sendCommand(closeGripper, ClientSocket);
    Sleep(1000);
    char openGripper[] = "OUTPUT 48 OFF";
    sendCommand(openGripper, ClientSocket);

    //========== Add your code above ==========//

    system("pause");

    // shutdown the connection since we're done
    iResult = shutdown(ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();

    return 0;
}
