#include "communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

string Communicator::Adapt(const bool i_Value) const
{
    return i_Value ? "True" : "False";
}

string Communicator::Adapt(const TimecodeStandard::Enum i_Standard) const
{
    switch (i_Standard)
    {
    default:
    case TimecodeStandard::None:
        return "0";
    case TimecodeStandard::PAL:
        return "1";
    case TimecodeStandard::NTSC:
        return "2";
    case TimecodeStandard::NTSCDrop:
        return "3";
    case TimecodeStandard::Film:
        return "4";
    case TimecodeStandard::NTSCFilm:
        return "5";
    case TimecodeStandard::ATSC:
        return "6";
    }
}

string Communicator::Adapt(const Direction::Enum i_Direction) const
{
    switch (i_Direction)
    {
    case Direction::Forward:
        return "Forward";
    case Direction::Backward:
        return "Backward";
    case Direction::Left:
        return "Left";
    case Direction::Right:
        return "Right";
    case Direction::Up:
        return "Up";
    case Direction::Down:
        return "Down";
    default:
        return "Unknown";
    }
}

string Adapt(const DeviceType::Enum i_DeviceType)
{
    switch (i_DeviceType)
    {
    case DeviceType::ForcePlate:
        return "ForcePlate";
    case DeviceType::Unknown:
    default:
        return "Unknown";
    }
}

string Adapt(const Unit::Enum i_Unit)
{
    switch (i_Unit)
    {
    case Unit::Meter:
        return "Meter";
    case Unit::Volt:
        return "Volt";
    case Unit::NewtonMeter:
        return "NewtonMeter";
    case Unit::Newton:
        return "Newton";
    case Unit::Kilogram:
        return "Kilogram";
    case Unit::Second:
        return "Second";
    case Unit::Ampere:
        return "Ampere";
    case Unit::Kelvin:
        return "Kelvin";
    case Unit::Mole:
        return "Mole";
    case Unit::Candela:
        return "Candela";
    case Unit::Radian:
        return "Radian";
    case Unit::Steradian:
        return "Steradian";
    case Unit::MeterSquared:
        return "MeterSquared";
    case Unit::MeterCubed:
        return "MeterCubed";
    case Unit::MeterPerSecond:
        return "MeterPerSecond";
    case Unit::MeterPerSecondSquared:
        return "MeterPerSecondSquared";
    case Unit::RadianPerSecond:
        return "RadianPerSecond";
    case Unit::RadianPerSecondSquared:
        return "RadianPerSecondSquared";
    case Unit::Hertz:
        return "Hertz";
    case Unit::Joule:
        return "Joule";
    case Unit::Watt:
        return "Watt";
    case Unit::Pascal:
        return "Pascal";
    case Unit::Lumen:
        return "Lumen";
    case Unit::Lux:
        return "Lux";
    case Unit::Coulomb:
        return "Coulomb";
    case Unit::Ohm:
        return "Ohm";
    case Unit::Farad:
        return "Farad";
    case Unit::Weber:
        return "Weber";
    case Unit::Tesla:
        return "Tesla";
    case Unit::Henry:
        return "Henry";
    case Unit::Siemens:
        return "Siemens";
    case Unit::Becquerel:
        return "Becquerel";
    case Unit::Gray:
        return "Gray";
    case Unit::Sievert:
        return "Sievert";
    case Unit::Katal:
        return "Katal";

    case Unit::Unknown:
    default:
        return "Unknown";
    }
}

Communicator::Communicator(/* args */)
{
    const char* const* argv = NULL;
    rclcpp::init(0, argv);
    pub = NULL;
}

void Communicator::GetParams() 
{
    hostname = GetParam("hostname");
    buffer_size = stoi(GetParam("buffer_size").c_str());
    target_subject_index = stoi(GetParam("subject_index").c_str());
    topic_name = GetParam("topic");
    string segments_str = GetParam("segments");
    size_t semicolon = segments_str.find(";");
    while (semicolon != string::npos)
    {
        segments.push_back(segments_str.substr(0, semicolon));
        segments_str.erase(0, semicolon + 1);
        semicolon = segments_str.find(";");
    }
}

bool Communicator::Connect()
{
    GetParams();
    string msg = "Connecting to " + hostname + " ...";
    cout << msg << endl;
    Log(msg, INFO);
    int counter = 0;
    while (!MyClient.IsConnected().Connected)
    {
        bool ok = (MyClient.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            msg = "Connect failed...";
            cout << msg << endl;
            Log(msg, WARNING);
            counter++;
        }
        if (counter >= 3)
        {
            msg = "Aborting attempt to connect to " + hostname;
            Log(msg, ERROR);
            return false;
        }
        sleep(1);
    }
    msg = "Connection successfully established with " + hostname;
    cout << msg << endl;
    Log(msg, INFO);
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableMarkerRayData();
    MyClient.EnableDeviceData();
    MyClient.EnableDebugData();
    MyClient.EnableCentroidData();
    Log("Enabling datatypes for transfer", INFO);

    MyClient.SetStreamMode(StreamMode::ClientPull);
    Log("Setting Stream mode to ClientPull", INFO);

    MyClient.SetAxisMapping(Direction::Forward,
                            Direction::Up,
                            Direction::Left); //TODO: Y-up by default but to be included in settings
    Log("Setting up Axis", INFO);

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    Log("Axis Mapping: X-" + Adapt(_Output_GetAxisMapping.XAxis) + " Y-" + Adapt(_Output_GetAxisMapping.YAxis) + " Z-" + Adapt(_Output_GetAxisMapping.ZAxis), INFO);

    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    Log("Version: " + to_string(_Output_GetVersion.Major) + "." + to_string(_Output_GetVersion.Minor) + "." + to_string(_Output_GetVersion.Point), INFO);

    MyClient.SetBufferSize(buffer_size);
    Log("Setting client buffer size to " + to_string(buffer_size), INFO);

    if (!pub)
        pub = new Publisher(topic_name);

    running = true;
    return true;
}

bool Communicator::Disconnect()
{
    running = false;
    sleep(1);
    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    MyClient.DisableCentroidData();
    Log("Disconnecting from " + hostname + "...", INFO);
    MyClient.Disconnect();
    Log("Successfully disconnected", INFO);
    if (!MyClient.IsConnected().Connected)
        return true;
    return false;
}

void Communicator::FrameGetter()
{
    while (running)
    {
        Log("Fetching new frame...", INFO);
        MyClient.GetFrame();

        Output_GetFrameNumber FrameNumber = MyClient.GetFrameNumber();
        Log("Frame number: " + to_string(FrameNumber.FrameNumber), INFO);

        Output_GetFrameRate Rate = MyClient.GetFrameRate();
        Log("Frame Rate: " + to_string(Rate.FrameRateHz) + " Hz", INFO);

        Output_GetTimecode _Output_GetTimecode = MyClient.GetTimecode();

        string msg = "Last frame timecode: " + to_string(_Output_GetTimecode.Hours) + "h " + to_string(_Output_GetTimecode.Minutes) + "m " + to_string(_Output_GetTimecode.Seconds) + "s " + to_string(_Output_GetTimecode.Frames) + "f " + to_string(_Output_GetTimecode.SubFrame) + "sf " + Adapt(_Output_GetTimecode.FieldFlag) + " " + Adapt(_Output_GetTimecode.Standard) + " " + to_string(_Output_GetTimecode.SubFramesPerFrame) + " " + to_string(_Output_GetTimecode.UserBits);
        Log(msg, INFO);

        Output_GetLatencyTotal Latency = MyClient.GetLatencyTotal();
        Log("Latency: " + to_string(Latency.Total), INFO);

        unsigned int CameraCount = MyClient.GetCameraCount().CameraCount;
        Log("Number of cameras: " + to_string(CameraCount), INFO);

        for (unsigned int CameraIndex = 0; CameraIndex < CameraCount; ++CameraIndex)
        {
            const string CameraName = MyClient.GetCameraName(CameraIndex).CameraName;

            Log("Camera #" + to_string(CameraIndex) + ":" + '\n' + "    Name: " + CameraName + '\n', INFO);
        }

        unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
        msg = SubjectCount + " subject(s) detected";
        Log(msg, INFO);
        
        for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
        {
            if (SubjectIndex == target_subject_index) 
            {  
                // Get the subject name
                string SubjectName = MyClient.GetSubjectName(SubjectIndex).SubjectName;
                msg = "Subject " + to_string(SubjectIndex) + " is: " + SubjectName;
                Log(msg, INFO);

                // Count the number of segments
                unsigned int SegmentCount = MyClient.GetSegmentCount(SubjectName).SegmentCount;

                // Get the subject root segment
                string RootSegmentName = MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
                CurrentRootSegment.name = RootSegmentName;
                CurrentRootSegment.subject_name = SubjectName;
                msg = "Subject root segment is : " + RootSegmentName;
                Log(msg, INFO);

                for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
                {
                    // Get the segment name
                    string SegmentName = MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
                    msg = "Found segment: " + SegmentName;
                    Log(msg, INFO);

                    if (IsSegmentValid(SegmentName)) 
                    {
                        Output_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation =
                        MyClient.GetSegmentLocalTranslation(SubjectName, SegmentName);
                        Output_GetSegmentLocalRotationQuaternion _Output_GetSegmentLocalRotationQuaternion = 
                        MyClient.GetSegmentLocalRotationQuaternion(SubjectName, SegmentName);
                        for (size_t i = 0; i < 4; i++)
                        {
                            if (i < 3)
                                CurrentPosition.translation[i] = _Output_GetSegmentLocalTranslation.Translation[i];
                            CurrentPosition.rotation[i] = _Output_GetSegmentLocalRotationQuaternion.Rotation[i];
                        }
                        CurrentPosition.segment_name = SegmentName;
                        CurrentPosition.subject_name = SubjectName;
                        CurrentPosition.translation_type = "Local";
                        CurrentPosition.frame_number = FrameNumber.FrameNumber;
                        msg = "Publishing segment " + SegmentName + " from subject " + SubjectName + " with translation type Local";
                        Log(msg, INFO);
                        pub->PublishRootSegment(CurrentRootSegment);
                        pub->PublishPosition(CurrentPosition);
                    }

                    else
                    {
                        Log("Unfit segment, skipping...", WARNING);
                    }
                    
                }
            }
        }
    }
}

bool Communicator::IsConnected() const
{
    return MyClient.IsConnected().Connected;
}

bool Communicator::IsSegmentValid(string test_segment) const 
{
    for (string segment : segments)
    {
        if (ci_find_substr(test_segment, segment))
        {
            return true;
        }
    }
    return false;
}

string Communicator::GetHostName() const
{
    return hostname;
}

Communicator::~Communicator()
{
    delete pub;
}