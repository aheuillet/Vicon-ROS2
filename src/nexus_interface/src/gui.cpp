#include "gui.hpp"

wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_MENU(ID_Start,   MyFrame::OnStart)
    EVT_BUTTON(BUTTON_Start, MyFrame::OnStart)
    EVT_BUTTON(BUTTON_Stop, MyFrame::OnStop)
    EVT_MENU(wxID_EXIT,  MyFrame::OnExit)
    EVT_MENU(wxID_ABOUT, MyFrame::OnAbout)
wxEND_EVENT_TABLE()

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{
    MyFrame *frame = new MyFrame( "ROS2VICON", wxPoint(50, 50), wxSize(450, 340) );
    frame->Show( true );
    return true;
}

MyFrame::MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
        : wxFrame(NULL, wxID_ANY, title, pos, size)
{
    menuCommand = new wxMenu;
    buttonsSizer = new wxBoxSizer(wxVERTICAL);
    buttonsSizer->AddStretchSpacer(1);
    buttonsSizer->Add(new wxButton(this, BUTTON_Start, _T("Start")), 0, wxEXPAND|wxALIGN_LEFT|wxALL);
    buttonsSizer->Add(new wxButton(this, BUTTON_Stop, _T("Stop")), 0, wxEXPAND|wxALIGN_RIGHT|wxALL);
    buttonsSizer->AddStretchSpacer(1);
    menuCommand->Append(ID_Start, "&Start\tCtrl-S",
                     "Help string shown in status bar for this menu item");
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
                  "About ROS2VICON", wxOK | wxICON_INFORMATION );
}

void MyFrame::OnStart(wxCommandEvent& event)
{
    wxLogMessage("Starting streaming to ROS");
}

void MyFrame::OnStop(wxCommandEvent& event)
{
    wxLogMessage("Stopping streaming to ROS");
}