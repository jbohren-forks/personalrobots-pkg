///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Eric Berger
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include "wx/wx.h"
#include "wxstring.h"
#include "ros/node.h"
#include "logfile.h"
#include "time.h"

class Topic{
public:
  Topic(string name, string type){this->name = name; this->type = type; checkBox = NULL; logfile = NULL;};
  string name;
  string type;
  wxCheckBox *checkBox;
  Logfile *logfile;
};

class LoggerNode : public ros::node{
public:
  LoggerNode();
  void start_logging(string prefix);
  void stop_logging();
  vector<Topic *> topics;
  void dummy_cb(void) {};
};

void LoggerNode::start_logging(string prefix){
  ros::Time start = ros::Time::now();
  time_t t = time(NULL);
  struct tm *tms = localtime(&t);
  char datetime[1024];
  snprintf(datetime, sizeof(datetime), "-%d-%02d-%02d-%02d-%02d-%02d",
      tms->tm_year + 1900, tms->tm_mon+1, tms->tm_mday, tms->tm_hour, tms->tm_min, tms->tm_sec);
  string directory_name = prefix + string(datetime);
  mkdir(directory_name.c_str(), 0755);
  for(vector<Topic *>::iterator it = topics.begin(); it != topics.end(); it++){
    if((*it)->checkBox->IsChecked()){
      printf("Logging topic %s\n", (*it)->name.c_str());
      (*it)->logfile = new LogfileCapture(directory_name, (*it)->name, start);
      printf("Created logfile.\n");
      subscribe((*it)->name.c_str(), *(*it)->logfile, &LoggerNode::dummy_cb);
      printf("Subscribed.\n");
    }
  }
}

void LoggerNode::stop_logging(){
  printf("Stopping all logging.\n");
  
  for(vector<Topic *>::iterator it = topics.begin(); it != topics.end(); it++){
    if((*it)->logfile){	
      printf("Stopping loggin on topic %s\n", (*it)->name.c_str());
      unsubscribe((*it)->name.c_str());
      delete((*it)->logfile);
      (*it)->logfile = NULL;
    }
  }
}

LoggerNode::LoggerNode()
: ros::node("logger"){
  vector<pair<string, string> > raw_topics;
  get_published_topics(&raw_topics);
  vector<pair<string, string> >::iterator it;
  for(it = raw_topics.begin(); it != raw_topics.end(); it++){
    topics.push_back(new Topic(it->first, it->second));
  }
}

LoggerNode *logger;

class LoggerApp : public wxApp{
public:
  virtual bool OnInit();
  virtual int OnQuit();
};

class LoggerFrame : public wxFrame
{
public:
  LoggerFrame(const wxChar *title, wxPoint pos=wxDefaultPosition, wxSize size=wxDefaultSize);
  void start_logging(wxCommandEvent& WXUNUSED(event));
  void stop_logging(wxCommandEvent& WXUNUSED(event));

  wxButton *startButton;
  wxButton *stopButton;
  wxTextCtrl *prefixBox;
};

IMPLEMENT_APP(LoggerApp)

bool LoggerApp::OnInit()
{
  char **argv = new char*[wxApp::argc]; //This memory duplicates argv, so is never deallocated
  for(int i = 0; i < wxApp::argc; i++){
    argv[i] = strdup((wxString(wxApp::argv[i]).mb_str(wxConvUTF8))); //This memory duplicates argv, so is never deallocated
  }
  ros::init(wxApp::argc, argv);

  logger = new LoggerNode();
  LoggerFrame *frame = new LoggerFrame(_T("Logger"));
  frame->Show(TRUE);
  SetTopWindow(frame);
  return TRUE;
}

int LoggerApp::OnQuit(){
  ros::fini();
  delete(logger);
  return(0);
}

LoggerFrame::LoggerFrame(const wxChar *title, wxPoint pos, wxSize size)
: wxFrame ((wxFrame*)NULL, wxID_ANY, title, pos, size)
{
  CreateStatusBar();
  SetStatusText(_T("Select topics to log"));

  wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
  vector<Topic *>::iterator it;
  for(it = logger->topics.begin(); it != logger->topics.end(); it++){
    (*it)->checkBox = new wxCheckBox(this, wxID_ANY, std2wx(((*it)->name + "(" + (*it)->type + ")")));
    sizer->Add((*it)->checkBox);
  }

  wxPanel *p = new wxPanel(this, wxID_ANY);
  sizer->Add(p);
  wxSizer *sizer2 = new wxBoxSizer(wxHORIZONTAL);
  startButton = new wxButton(p, wxID_OK, _T("Start logging"));
  stopButton = new wxButton(p, wxID_STOP, _T("Stop logging"));
  sizer2->Add(startButton);
  sizer2->Add(stopButton);
  p->SetSizer(sizer2);
  sizer2->Fit(p);

  stopButton->Disable();

  Connect(wxID_OK, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LoggerFrame::start_logging)); 
  Connect(wxID_STOP, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LoggerFrame::stop_logging));

  wxPanel *prefix_panel = new wxPanel(this, wxID_ANY);
  wxSizer *prefix_sizer = new wxBoxSizer(wxHORIZONTAL);
  prefixBox = new wxTextCtrl(prefix_panel, wxID_ANY);
  prefixBox->SetValue(std2wx("logfile"));
  wxStaticText *prefixLabel = new wxStaticText(prefix_panel, wxID_ANY, std2wx("Logfile prefix"));
  prefix_sizer->Add(prefixLabel, 0, wxCENTER);
  prefix_sizer->Add(prefixBox, 0, wxCENTER);
  prefix_panel->SetSizer(prefix_sizer);
  prefix_sizer->Fit(prefix_panel);

  sizer->Add(prefix_panel);

  SetSizer(sizer);
  sizer->Fit(this);
}

void LoggerFrame::start_logging(wxCommandEvent& WXUNUSED(event)){
  logger->start_logging(string(wx2std(prefixBox->GetValue())));
  SetStatusText(_T("Logging..."));
  startButton->Disable();
  stopButton->Enable();
  for(vector<Topic *>::iterator it = logger->topics.begin(); it != logger->topics.end(); it++){
    (*it)->checkBox->Disable();
    }
}

void LoggerFrame::stop_logging(wxCommandEvent& WXUNUSED(event)){
  logger->stop_logging();
  SetStatusText(_T("Select topics to log"));
  startButton->Enable();
  stopButton->Disable();
  for(vector<Topic *>::iterator it = logger->topics.begin(); it != logger->topics.end(); it++){
    (*it)->checkBox->Enable();
  }
}

//LoggerFrame::~LoggerFrame()
//{
//}
