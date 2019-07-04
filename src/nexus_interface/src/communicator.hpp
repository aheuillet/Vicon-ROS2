#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "../../DataStreamSDK/DataStreamClient.h"
#include "logging.hpp"
#include "utils.hpp"
#include "publisher.hpp"
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
    unsigned int buffer_size;
    unsigned int target_subject_index;
    string topic_name;
    string segments;
    list<Position> Positions;
    Position CurrentPosition;
    Publisher *pub;
    bool running = false;
    string Adapt(const bool i_Value) const;
    string Adapt(const ViconDataStreamSDK::CPP::TimecodeStandard::Enum i_Standard) const;
    string Adapt(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction) const;
    void GetParams();

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
