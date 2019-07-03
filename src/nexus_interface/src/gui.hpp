#ifndef GUI_HPP
#define GUI_HPP

#define wxUSE_TEXTCTRL true
#define wxUSE_GUI true

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif
#include "wx/preferences.h"
#include "wx/app.h"
#include "wx/statusbr.h"
#include "wx/config.h"
#include "wx/panel.h"
#include "wx/scopedptr.h"
#include "wx/menu.h"
#include "wx/checkbox.h"
#include "wx/stattext.h"
#include "wx/sizer.h"
#include "wx/artprov.h"
#include "wx/frame.h"
#include "wx/textctrl.h"
#include "wx/checklst.h"
#include "communicator.hpp"
#include <thread>

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};

class MyFrame : public wxFrame
{
public:
    MyFrame(const wxString &title, const wxPoint &pos, const wxSize &size);

private:
    Communicator client;
    std::thread main_loop;
    wxMenu *menuCommand;
    wxSizer *buttonsSizer;
    wxMenu *menuHelp;
    wxMenuBar *menuBar;
    wxButton *start_button;
    wxButton *stop_button;
    wxPreferencesEditor *editor;
    void OnStart(wxCommandEvent &event);
    void OnStop(wxCommandEvent &event);
    void OnExit(wxCommandEvent &event);
    void OnAbout(wxCommandEvent &event);
    void OnSettings(wxCommandEvent &event);
    wxDECLARE_EVENT_TABLE();
};

class PrefPagePanel : public wxPanel
{
public:
    PrefPagePanel(wxWindow *parent);

private:
    std::list<wxTextCtrl *> parameters;
    void UpdateSettings() const;
    list<ConfigLine> current_config;
};

class PrefPage : public wxStockPreferencesPage
{
public:
    PrefPage() : wxStockPreferencesPage(Kind_General) {}
    virtual wxWindow *CreateWindow(wxWindow *parent);
};

enum
{
    ID_Start = 1,
    BUTTON_Stop = 2,
    BUTTON_Start = 3
};

#endif