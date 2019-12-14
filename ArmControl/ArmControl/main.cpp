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
#define GRIPPER_CLOSE 0
#define GRIPPER_OPEN 1

using namespace std;
using namespace cv;

struct Pose {
    int x;
    int y;
    int z;
    int degX;
    int degY; 
    int degZ;

    Pose(int x, int y, int z, int degX, int degY, int degZ): x(x), y(y), z(z), degX(degX), degY(degY), degZ(degZ) { };
    static Pose from_centroid(tuple<double, double, double> centroid, int z) {
        return Pose(get<0>centroid, get<1>centroid, z, get<2>centroid, 0, 180);
    }
};

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

int moveArm(SOCKET ClientSocket, Pose pose) {
  degX = pose.x + 90; // the rotation in robot coords and camera coords is off by 90 degrees
  cout << "Moving robot to pose: [" << pose.x << ", \t" << pose.y << ", \t" << pose.z
    << ", \t" << degX << "deg, \t" << pose.degY << "deg, \t" << pose.degZ << "]" << endl;

  char* commandPtr = new char[100];
  sprintf(commandPtr, "MOVP %d %d %d %d %d %d", pose.x, pose.y, pose.z, degX, pose.degY, pose.degZ);

  char command[100];
  strcpy(command, commandPtr);

  cout << "Moving arm with command: " << command << endl;
  sendCommand(command, ClientSocket);

  return 0;
}

void controlGripper(SOCKET ClientSocket, int action) {
    char closeCmd[] = "OUTPUT 48 ON";
    char openCmd[] = "OUTPUT 48 OFF";
    sendCommand(action == GRIPPER_CLOSE ? closeCmd : openCmd, ClientSocket);
    Sleep(500);
}

void goHome(SOCKET ClientSocket) {
    char cmd[] = "GOHOME";
    sendCommand(cmd, ClientSocket);
}

int __cdecl main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "usage: hw4 <camera_id> [ <scaling> ]";
        return 1;
    }

    // Read camera parameters from file
    cout << "Reading camera parameters..." << endl;
    ifstream paramsFile;
    paramsFile.open("camparams.txt");
    if (!paramsFile) { cerr << "camparams.txt file is missing."; return 1; }
    double f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3;
    paramsFile >> f_x >> f_y >> c_x >> c_y >> k_1 >> k_2 >> p_1 >> p_2 >> k_3;

    // Set camera distortion parameters
    Mat cameraMat = (Mat_<double>(3, 3) << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(5, 1) << k_1, k_2, p_1, p_2, k_3);

    // ---- Robot connetion code

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


    // ---- Robot connected 

    // Connect to camera
    cout << "Connecting to camera..." << endl;
    VideoCapture camera = init_camera(atoi(argv[1]));
    waitKey(2000);
    cout << "Connected." << endl;

    // Get scaling
    Vec2 scaling = (argc >= 3) ? Vec2(-atof(argv[2]),atof(argv[2])) : measure_scaling(&camera);
    cout << "Scaling factor is [" << scaling.x << ", " << scaling.y << "]" << endl;

    // Read known points
    int known_points_count;
    Vec2 *known_points_cam, *known_points_world;
    read_known_points(&known_points_count, known_points_cam, known_points_world);

    Vec2 camera_orig_world = estimate_origin(known_points_cam, known_points_world, 
        known_points_count, scaling);

    cout << "Assuming the camera origin [0, 0] is at [" <<
        camera_orig_world.x << ", " << camera_orig_world.y << "] in robot space." << endl;

    perform_sanity_check(known_points_count, known_points_cam, known_points_world, scaling, camera_orig_world);

    Mat frame = get_camera_frame(camera);

    // Undistort & convert the image
    Mat work_image = undistort_camera_frame(frame, cameraMat, distCoeffs);
    cvtColor(work_image, work_image, COLOR_BGR2GRAY);

    // Get centroids
    vector<tuple<double, double, double>> centroids;
    get_centroids(work_image, &centroids);

    // Display debug information on the screen
    cvtColor(work_image, work_image, COLOR_GRAY2RGB);
    display_points(work_image, centroids, scaling, camera_orig_world);
    imshow("Results", work_image);
    waitKey(500);

    // Read which centroid to pick up
    int pickup_id;
    cout << "Select an object to pick up:" << endl;
    cin >> pickup_id;

    if (pickup_id < 0 || pickup_id >= centroids.size()) {
        cerr << pickup_id << " is out of the range of objects." << endl;
        return 1;
    }

    cout >> "Picking up block ID " >> pickup_id >> endl;

    // ------- Robot logic

    goHome();
    controlGripper(ClientSocket, GRIPPER_OPEN);
    moveArm(ClientSocket, Pose.from_centroid(centroids[pickup_id], -210);
    controlGripper(ClientSocket, GRIPPER_CLOSE);
    Sleep(1000);
    controlGripper(ClientSocket, GRIPPER_OPEN);
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
