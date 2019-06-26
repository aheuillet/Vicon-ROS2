#ifndef GUI_HPP
#define GUI_HPP

#include <wx/wxprec.h>
  #ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

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
    wxMenu *menuCommand;
    wxSizer *buttonsSizer;
    wxMenu *menuHelp;
    wxMenuBar *menuBar;
    void OnStart(wxCommandEvent& event);
    void OnStop(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    wxDECLARE_EVENT_TABLE();
};

enum
{
    ID_Start = 1, BUTTON_Stop = 2, BUTTON_Start = 3
};

#endif