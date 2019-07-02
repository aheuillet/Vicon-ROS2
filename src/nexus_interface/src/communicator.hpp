#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "../../DataStreamSDK/DataStreamClient.h"
#include "logging.hpp"
#include "utils.hpp"
#include <iostream>
#include <list>
#include <csignal>
#include <string>
#include <unistd.h>

using namespace std;

class Communicator
{
private:
    ViconDataStreamSDK::CPP::Client MyClient;
    string hostname;
    int buffer_size;
    int camera_index;
    int subject_index;
    string topic_name;
    list<Position> Positions;
    Position CurrentPosition;
    bool running = false;
    string Adapt(const bool i_Value) const;
    string Adapt(const ViconDataStreamSDK::CPP::TimecodeStandard::Enum i_Standard) const;
    string Adapt(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction) const;

public:
    Communicator(/* args */);
    bool Connect();
    bool Disconnect();
    bool IsConnected() const;
    void FrameGetter();
    string GetHostName() const;
    ~Communicator();
};

#endif // COMMUNICATOR_HPP
