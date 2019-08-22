///////////////////////////////////////////////////////////////////////////////////////////////
/// This file holds all GUI related classes and functions.
///////////////////////////////////////////////////////////////////////////////////////////////

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

/// This class holds the GUI main window and most of the logic behind.
class MyFrame : public wxFrame
{
public:
    /// Main window constructor.
    /// title: window title.
    /// pos: window position in pixels (on the screen).
    /// size: window size in pixels.
    MyFrame(const wxString &title, const wxPoint &pos, const wxSize &size);
    ~MyFrame();

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

    /// Defines the app's behaviour when the 'Start button clicked' event is fired.
    void OnStart(wxCommandEvent &event);

    /// Defines the app's behaviour when the 'Stop button clicked' event is fired.
    void OnStop(wxCommandEvent &event);

    /// Defines the app's behaviour when the 'Exit button clicked' event is fired.
    void OnExit(wxCommandEvent &event);

    /// Defines the app's behaviour when the 'About button clicked' event is fired.
    void OnAbout(wxCommandEvent &event);

    /// Defines the app's behaviour when the 'Settings button clicked' event is fired.
    void OnSettings(wxCommandEvent &event);

    wxDECLARE_EVENT_TABLE();
};

/// Main container class for the app's GUI. Holds the main frame.
class MyApp : public wxApp
{
public:
    virtual bool OnInit();
    ~MyApp();

private:
    MyFrame *frame;
};

/// Holds the graphical settings editor panel and its logic.
class PrefPagePanel : public wxPanel
{
public:
    PrefPagePanel(wxWindow *parent);
    ~PrefPagePanel();

private:
    /// Reads the values entered by the user in the GUI and writes them in the config file.
    void UpdateSettings() const;

    /// List holding the config values read from user input.
    list<ConfigLine> current_config;
};

/// Holds the graphical settings editor window. To be used in conjunction
/// with PrefPagePanel class.
class PrefPage : public wxStockPreferencesPage
{
public:
    PrefPage() : wxStockPreferencesPage(Kind_General) {}
    virtual wxWindow *CreateWindow(wxWindow *parent);
};

/// Represents the ids corresponding to GUI buttons (in order to associate them with actions).
enum
{
    ID_Start = 1,
    BUTTON_Stop = 2,
    BUTTON_Start = 3
};

#endif