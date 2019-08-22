#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "../../../DataStreamSDK/DataStreamClient.h"
#include "logging.hpp"
#include "utils.hpp"
#include "publisher.hpp"
#include <iostream>
#include <list>
#include <csignal>
#include <string>
#include <unistd.h>

using namespace std;

/// This class holds the capability to connect to a DataStream server and retrieve segment information.
/// This data can then be passed to the Publisher class and published to a ROS topic.
class Communicator
{
private:
    ViconDataStreamSDK::CPP::Client MyClient;
    string hostname;
    unsigned int buffer_size;
    unsigned int target_subject_index;
    string topic_name;
    list<string> segments;
    list<Position> Positions;
    Position CurrentPosition;
    Publisher *pub;
    bool running = false;

    /// This function translates a boolean value to its string equivalent.
    string Adapt(const bool i_Value) const;

    /// This function translates a Vicon TimeCode value to its string equivalent.
    string Adapt(const ViconDataStreamSDK::CPP::TimecodeStandard::Enum i_Standard) const;

    /// This functions translates a Vicon Direction value to its string equivalent.
    string Adapt(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction) const;

    /// This handler retrieves the parameter values (defined by the user in the "./settings.cfg" file)
    /// and set their corresponding class attributes to them.   
    void GetParams();

public:
    Communicator();

    /// Initialises the connection to the DataStream server using the settings retrieved
    /// with GetParams().
    bool Connect();

    /// Stops the current connection to a DataStream server (if any).
    bool Disconnect();

    /// Returns a boolean value whether a connection to a DataStream server has been established.
    bool IsConnected() const;

    /// Tests if a given segment name matches at least one of the selected body part names.
    bool IsSegmentValid(string test_segment) const;

    /// Main loop that request frames from the currently connected DataStream server and send the 
    /// received segment data to the Publisher class.
    void FrameGetter();

    /// Returns the hostname of the currently connected DataStream server.
    string GetHostName() const;
    ~Communicator();
};

#endif // COMMUNICATOR_HPP
