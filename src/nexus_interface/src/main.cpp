#include "../../DataStreamSDK/DataStreamClient.h"
#include "utils.hpp"
#include <iostream>
#include <list>
#include <csignal>
#include <string>

#define output_stream std::cout

using namespace ViconDataStreamSDK::CPP;

Client MyClient;

void KeyboardInterruptHandler(int s)
{
    std::cout << s << std::endl; //To stop compiler from complaining
    std::cout << "Exiting now..." << std::endl;
    MyClient.Disconnect();
    exit(1);
}

int main()
{
    //Getting parameters from config file
    std::string hostname = GetParam("hostname");
    int buffer_size = std::stoi(GetParam("buffer_size").c_str());
    int camera_index = std::stoi(GetParam("camera_index").c_str());
    int subject_index = std::stoi(GetParam("subject_index").c_str());
    std::string topic_name = GetParam("topic");
    std::string log_file = GetParam("log_file");

    std::list<Position> Positions;
    Position CurrentPosition;
    Output_Connect Output = MyClient.Connect(hostname.c_str());
    std::cerr << "Result: " + std::to_string(Output.Result) << std::endl;
    if (Output.Result != Result::Success)
    {
        std::cerr << "Error: Could not connect to compatible DataStream server, exiting now..." << std::endl;
        exit(1);
    }
    MyClient.EnableDebugData();
    MyClient.EnableSegmentData();
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
        // Count the number of subjects
        unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
        output_stream << "Subjects (" << SubjectCount << "):" << std::endl;
        for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
        {
            output_stream << "  Subject #" << SubjectIndex << std::endl;

            // Get the subject name
            std::string SubjectName = MyClient.GetSubjectName(SubjectIndex).SubjectName;
            output_stream << "    Name: " << SubjectName << std::endl;

            // Get the root segment
            std::string RootSegment = MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
            output_stream << "    Root Segment: " << RootSegment << std::endl;

            // Count the number of segments
            unsigned int SegmentCount = MyClient.GetSegmentCount(SubjectName).SegmentCount;
            output_stream << "    Segments (" << SegmentCount << "):" << std::endl;
            for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
            {
                output_stream << "      Segment #" << SegmentIndex << std::endl;

                // Get the segment name
                std::string SegmentName = MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
                output_stream << "        Name: " << SegmentName << std::endl;

                // Get the segment parent
                std::string SegmentParentName = MyClient.GetSegmentParentName(SubjectName, SegmentName).SegmentName;
                output_stream << "        Parent: " << SegmentParentName << std::endl;

                // Get the segment's children
                unsigned int ChildCount = MyClient.GetSegmentChildCount(SubjectName, SegmentName).SegmentCount;
                output_stream << "     Children (" << ChildCount << "):" << std::endl;
                for (unsigned int ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
                {
                    std::string ChildName = MyClient.GetSegmentChildName(SubjectName, SegmentName, ChildIndex).SegmentName;
                    output_stream << "       " << ChildName << std::endl;
                }

                Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
                    MyClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);
                output_stream << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[0] << ", "
                              << _Output_GetSegmentGlobalTranslation.Translation[1] << ", "
                              << _Output_GetSegmentGlobalTranslation.Translation[2] << ") "
                              << std::endl;
                for (size_t i = 0; i < POSITION_NUMBER; i++)
                {
                    CurrentPosition.translation[i] = _Output_GetSegmentGlobalTranslation.Translation[i];
                }
            }
            Frame = MyClient.GetFrame();
        }

        return 0;
    }
}
