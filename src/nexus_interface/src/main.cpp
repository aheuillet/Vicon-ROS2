#include "../../DataStreamSDK/DataStreamClient.h"
#include "utils.cpp"
#include <iostream>
#include <list>
#include <csignal>

using namespace ViconDataStreamSDK::CPP;

struct Position
{
    double position_point[2];
    double radius;
} typedef Position;

void KeyboardInterruptHandler(int s)
{
    printf("%d \n", s); //To stop compiler from complaining
    std::cout << "Exiting now..." << std::endl;
    exit(1);
}

int main()
{
    //Getting parameters from config file
    string hostname = GetParam("../settings.cfg", "localhost");
    int buffer_size = std::stoi(GetParam("../settings.cfg", "buffer_size").c_str());
    int camera_index = std::stoi(GetParam("../settings.cfg", "camera_index").c_str());
    int subject_demo = std::stoi(GetParam("../settings.cfg", "subject_demo").c_str());

    Client MyClient;
    std::list<Position> Positions;
    Output_GetCentroidPosition CentroidPosition;
    Output_GetDeviceCount GDC;
    Output_GetDeviceName GDN;
    Position CurrentPosition;
    string device_name;
    string device_type;
    Output_Connect Output = MyClient.Connect("localhost");
    if (Output.Result != Result::Success)
    {
        std::cerr << "Error: Could not connect to compatible DataStream server, exiting now..." << std::endl;
        exit(1);
    }
    //MyClient.EnableCentroidData();
    MyClient.EnableDeviceData();
    Output_GetFrame Frame = MyClient.GetFrame();

    MyClient.SetBufferSize(2);
    MyClient.SetStreamMode(StreamMode::ClientPull);
    Output_GetSubjectName GSN = MyClient.GetSubjectName(0);
    String subject_name = GSN.SubjectName;
    Output_GetCameraName GCN = MyClient.GetCameraName(0);

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
        Positions.push_back(CurrentPosition); */
        GDC = MyClient.GetDeviceCount();
        if (GDC.DeviceCount == 0) 
        {
            std::cerr << "No device detected on this setup, exiting now" << std::endl;
            exit(1);
        }
        for (size_t i = 0; i < GDC.DeviceCount; i++)
        {
           GDN = MyClient.GetDeviceName(i);
           device_name = GDN.DeviceName;
           device_type = GDN.DeviceType;
           std::cout << "Device Index: " + to_string(i) + " Device Name: " + device_name + " Device Type: " + device_type;
        }
        

        Frame = MyClient.GetFrame();
    }

    return 0;
}
