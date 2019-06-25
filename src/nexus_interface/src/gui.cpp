// wxWidgets "Hello world" Program
// For compilers that support precompilation, includes "wx/wx.h".
#include <wx/wx.h>

class MyApp: public wxApp
{
public:
    virtual bool OnInit();
};

class MyFrame: public wxFrame
{
public:
    MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size);
private:
    void OnStart(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    wxDECLARE_EVENT_TABLE();
};

enum
{
    ID_Start = 1, ID_Stop = 2, BUTTON_Start = 3
};

wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_MENU(ID_Start,   MyFrame::OnStart)
    EVT_BUTTON(BUTTON_Start, MyFrame::OnStart)
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
    wxMenu *menuCommand = new wxMenu;
    wxButton *startButton = new wxButton(this, BUTTON_Start, _T("Start"), wxDefaultPosition, wxDefaultSize, 0);
    menuCommand->Append(ID_Start, "&Start\tCtrl-S",
                     "Help string shown in status bar for this menu item");
    menuCommand->AppendSeparator();
    menuCommand->Append(wxID_EXIT);
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append( menuCommand, "&Command" );
    menuBar->Append( menuHelp, "&Help" );
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