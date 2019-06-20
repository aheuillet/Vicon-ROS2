#include "../../DataStreamSDK/DataStreamClient.h"
#include "utils.hpp"
#include <iostream>
#include <list>
#include <csignal>
#include <string>
#include <cstdio>

using namespace ViconDataStreamSDK::CPP;

Client MyClient;

void KeyboardInterruptHandler(int s)
{
    printf("%d \n", s); //To stop compiler from complaining
    std::cout << "Exiting now..." << std::endl;
    MyClient.Disconnect();
    exit(1);
}

int main()
{
    //Getting parameters from config file
    std::string hostname = GetParam("./settings.cfg", "hostname");
    int buffer_size = std::stoi(GetParam("./settings.cfg", "buffer_size").c_str());
    int camera_index = std::stoi(GetParam("./settings.cfg", "camera_index").c_str());
    int subject_index = std::stoi(GetParam("./settings.cfg", "subject_index").c_str());

    std::list<Position> Positions;
    Output_GetCentroidPosition CentroidPosition;
    Output_GetDeviceCount GDC;
    Output_GetDeviceName GDN;
    Position CurrentPosition;
    std::string device_name;
    std::string device_type;
    Output_Connect Output = MyClient.Connect(hostname.c_str());
    std::cerr << "Result: " + std::to_string(Output.Result) << std::endl;
    if (Output.Result != Result::Success)
    {
        std::cerr << "Error: Could not connect to compatible DataStream server, exiting now..." << std::endl;
        exit(1);
    }
    MyClient.EnableDebugData();
    MyClient.EnableGreyscaleData();
    Output_GetFrame Frame = MyClient.GetFrame();

    MyClient.SetBufferSize(buffer_size);
    MyClient.SetStreamMode(StreamMode::ClientPull);
    Output_GetSubjectName GSN = MyClient.GetSubjectName(subject_index);
    String subject_name = GSN.SubjectName;
    Output_GetCameraName GCN = MyClient.GetCameraName(camera_index);
    std::cout << subject_name << std::endl;
    std::cout << GCN.CameraName << std::endl;

    //Installing a SIGINT signal handler to interrupt loop
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = KeyboardInterruptHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    while (true)
    {
        /* CentroidPosition = MyClient.GetCentroidPosition(GCN.CameraName, 0);
        CurrentPosition = {*CentroidPosition.CentroidPosition, CentroidPosition.Radius};
        std::cout << CurrentPosition.toString() << std::endl;
        Positions.push_back(CurrentPosition); */

        Output_GetGreyscaleBlobCount Output = MyClient.GetGreyscaleBlobCount(GCN.CameraName);
        std::cout << Output.BlobCount << std::endl;

        Frame = MyClient.GetFrame();
    }

    return 0;
}
