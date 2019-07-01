#ifndef GUI_HPP
#define GUI_HPP

#define wxUSE_TEXTCTRL true
#define wxUSE_GUI true

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif
#include <wx/preferences.h>
#include <wx/panel.h>
#include <wx/frame.h>
#include <wx/textctrl.h>
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
    wxPreferencesEditor *editor;
    void OnStart(wxCommandEvent &event);
    void OnStop(wxCommandEvent &event);
    void OnExit(wxCommandEvent &event);
    void OnAbout(wxCommandEvent &event);
    void OnSettings(wxCommandEvent &event);
    wxDECLARE_EVENT_TABLE();
};

class PrefPage : public wxPanel
{
public:
    PrefPage(wxWindow *parent);

private:
    std::list<wxTextCtrl *> parameters;
    void UpdateSettings() const;
    list<ConfigLine> current_config;
    void ChangeUsedCtrl(wxCommandEvent& e);
};

PrefPage::PrefPage(wxWindow *parent) : wxPanel(parent)
{
    current_config = GetConfigLines();
    wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    for (ConfigLine &line : current_config)
    {
        wxTextCtrl *text = new wxTextCtrl(this, wxID_ANY);
        text->SetLabel(line.name.c_str());
        text->SetValue(line.value.c_str());

        sizer->Add(text, wxSizerFlags().Border());
        parameters.push_back(text);
    }
    SetSizerAndFit(sizer);
}

void PrefPage::UpdateSettings() const 
{
    WriteConfigLines(current_config);
}

void PrefPage::ChangeUsedCtrl(wxCommandEvent& e) 
{
    
}

enum
{
    ID_Start = 1,
    BUTTON_Stop = 2,
    BUTTON_Start = 3
};

#endif