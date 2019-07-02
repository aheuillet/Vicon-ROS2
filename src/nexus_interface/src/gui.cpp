#include "gui.hpp"

wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_MENU(ID_Start,   MyFrame::OnStart)
    EVT_BUTTON(BUTTON_Start, MyFrame::OnStart)
    EVT_BUTTON(BUTTON_Stop, MyFrame::OnStop)
    EVT_MENU(wxID_PREFERENCES, MyFrame::OnSettings)
    EVT_MENU(wxID_EXIT,  MyFrame::OnExit)
    EVT_MENU(wxID_ABOUT, MyFrame::OnAbout)
wxEND_EVENT_TABLE()

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{
    NewLog();
    MyFrame *frame = new MyFrame( "ROS2VICON", wxPoint(50, 50), wxSize(450, 340) );
    Log("Creating GUI...", INFO);
    frame->Show( true );
    return true;
}

MyFrame::MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
        : wxFrame(NULL, wxID_ANY, title, pos, size)
{
    menuCommand = new wxMenu;
    buttonsSizer = new wxBoxSizer(wxVERTICAL);
    start_button = new wxButton(this, BUTTON_Start, _T("Start"));
    stop_button = new wxButton(this, BUTTON_Stop, _T("Stop"));
    stop_button->Enable(false);
    buttonsSizer->AddStretchSpacer(1);
    buttonsSizer->Add(start_button, 0, wxEXPAND|wxALIGN_LEFT|wxALL);
    buttonsSizer->Add(stop_button, 0, wxEXPAND|wxALIGN_RIGHT|wxALL);
    buttonsSizer->AddStretchSpacer(1);
    menuCommand->Append(ID_Start, "&Start\tCtrl-S",
                     "Help string shown in status bar for this menu item");
    menuCommand->AppendSeparator();
    menuCommand->Append(wxID_PREFERENCES, "&Settings\tCtrl-D", "Allows you to tweak the transmission settings");
    menuCommand->AppendSeparator();
    menuCommand->Append(wxID_EXIT);
    menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    menuBar = new wxMenuBar;
    menuBar->Append( menuCommand, "&Command" );
    menuBar->Append( menuHelp, "&Help" );
    SetSizer(buttonsSizer);
    SetMenuBar( menuBar );
    CreateStatusBar();
    SetStatusText( "Welcome to ROS2VICON!" );
}

void MyFrame::OnExit(wxCommandEvent& event)
{
    Close( true );
}

void MyFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox( "This free software was made using WxWidgets 3.0.4, ROS2 Dashing Diademata and Vicon DataStream 1.8",
                  "About ROS2VICON 1.0", wxOK | wxICON_INFORMATION );
}

void MyFrame::OnStart(wxCommandEvent& event)
{
    Log("Stopping streaming to ROS...", INFO);
    start_button->Enable(false);
    stop_button->Enable(true);
    std::string status_msg = "Connecting to " + client.GetHostName();
    SetStatusText(status_msg.c_str());
    if(client.Connect()) 
    {
        status_msg = "Connected to " + client.GetHostName();
        main_loop = thread(&Communicator::FrameGetter, ref(client));
    }
    else 
    {
        status_msg = "Failed to connect to " + client.GetHostName();
        start_button->Enable(true);
        stop_button->Enable(false);
    } 
    SetStatusText(status_msg.c_str());
}

void MyFrame::OnStop(wxCommandEvent& event)
{
    Log("Stopping streaming to ROS...", INFO);
    start_button->Enable(true);
    stop_button->Enable(false);
    client.Disconnect();
    main_loop.join();
    string status_msg = "Disconnected from " + client.GetHostName();
    SetStatusText(status_msg.c_str());
}

void MyFrame::OnSettings(wxCommandEvent& event) 
{
    if (!editor) 
    {
        editor = new wxPreferencesEditor("Settings");
        editor->AddPage(new PrefPage());
    }
    editor->Show(this);
}

wxWindow* PrefPage::CreateWindow(wxWindow *parent)
        { return new PrefPagePanel(parent); }

PrefPagePanel::PrefPagePanel(wxWindow *parent) : wxPanel(parent)
{
    current_config = GetConfigLines();
    wxSizer *sizer = new wxGridSizer(4);
    for (ConfigLine &line : current_config)
    {
        wxTextCtrl *text = new wxTextCtrl(this, wxID_ANY);
        wxStaticText *label = new wxStaticText(this, wxID_ANY, (line.name + ":").c_str());
        text->SetLabel(line.name.c_str());
        text->SetValue(line.value.c_str());

        sizer->Add(label, wxSizerFlags().Border());
        sizer->Add(text, wxSizerFlags().Border());

        text->Bind(wxEVT_TEXT, [=](wxCommandEvent &) {
            for (ConfigLine &line : current_config)
            {
                if (line.name == text->GetLabel())
                    line.value = text->GetValue();
            }
            UpdateSettings();
        });
        parameters.push_back(text);
    }
    SetSizer(sizer);
}

void PrefPagePanel::UpdateSettings() const
{
    WriteConfigLines(current_config);
}