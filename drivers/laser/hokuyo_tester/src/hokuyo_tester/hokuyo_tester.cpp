/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "hokuyo_tester.h"
#include "urg_laser.h"
#include <wx/dcclient.h>
#include <math.h>
#include <wx/app.h>

void HokuyoThread::OnExit()
{
  tester->DoStopScan();
  tester->thread = NULL;
}

void *HokuyoThread::Entry()
{
  if (!tester->DoStartScan())
    return NULL;

  while (1)
  {
    if ( TestDestroy() )
      return NULL;

    if (!tester->DoHandleScan())
      return NULL;

    Yield();
  }

  return NULL;
}


HokuyoTester::HokuyoTester( wxWindow* parent )
:
  GenHokuyoTester( parent ), view_scale(50), view_x(0), view_y(0), thread(NULL)
{
  urg_mutex = new wxMutex();
  log_mutex = new wxMutex();

  log = new wxLogTextCtrl(logText);

  wxLog::SetActiveTarget(log);

  wxBoxSizer* gl_sizer;
  gl_sizer = new wxBoxSizer( wxHORIZONTAL );

  int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};

  gl = new wxGLCanvas(visPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, wxT("LaserViewer"), args);

  gl_sizer->Add(gl, 1,  wxALL | wxEXPAND, 0);

  visPanel->SetSizer( gl_sizer );

  m_init = false;
  
  gl->Connect( wxEVT_PAINT, wxPaintEventHandler( HokuyoTester::OnPaint ), NULL, this );
  gl->Connect( wxEVT_ERASE_BACKGROUND, wxEraseEventHandler( HokuyoTester::OnEraseBackground ), NULL, this );
  gl->Connect( wxEVT_SIZE, wxSizeEventHandler( HokuyoTester::OnSize ), NULL, this );

  disconnectButton->Disable();

  scan.num_readings = 0;
}


void
HokuyoTester::OnConnect( wxCommandEvent& event )
{
    try
    {
      urg.open((const char*)(port->GetValue().mb_str(wxConvUTF8)));

      log_mutex->Lock();
      wxLogMessage(_T("Connection successful"));
      log_mutex->Unlock();

      urg.laser_on();

      disconnectButton->Enable();
      connectButton->Disable();

    } catch (URG::exception& e) {
      log_mutex->Lock();
      wxLogMessage(_T("Could not connect to Hokuyo: ") + wxString::FromAscii(e.what()));
      log_mutex->Unlock();
    }
}

void
HokuyoTester::OnDisconnect( wxCommandEvent& event )
{
  try
  {
    urg.close();

    log_mutex->Lock();
    wxLogMessage(_T("Disconnection successful"));
    log_mutex->Unlock();;

    disconnectButton->Disable();
    connectButton->Enable();

  } catch (URG::exception& e) {
    log_mutex->Lock();
    wxLogMessage(_T("Could not disconnect from Hokuyo: ") + wxString::FromAscii(e.what()));
    log_mutex->Unlock();
  }
}

void
HokuyoTester::OnScan( wxCommandEvent& event )
{
  if (scanButton->GetValue())
  {
    if (thread == NULL)
    {
      thread = new HokuyoThread(this);
      thread->Create();
      thread->Run();
    }
  } else {
    if (thread != NULL)
    {
      thread->Delete();
    }
  }
}

bool
HokuyoTester::DoStartScan()
{
  try
  {
    urg_mutex->Lock();
    urg.request_scans(true, -M_PI/2.0, M_PI/2.0, 1, 1, 0, -1);    
    urg_mutex->Unlock();

    log_mutex->Lock();
    wxLogMessage(_T("Started scanning."));
    log_mutex->Unlock();
  } catch (URG::exception& e) {
    urg_mutex->Unlock();

    log_mutex->Lock();
    wxLogMessage(_T("Could not start scanning: ") + wxString::FromAscii(e.what()));
    log_mutex->Unlock();
    scanButton->SetValue(false);
    return false;
  }
  return true;
}

bool
HokuyoTester::DoStopScan()
{
  try
  {
    urg_mutex->Lock();
    urg.stop_scanning();
    urg_mutex->Unlock();

    log_mutex->Lock();
    wxLogMessage(_T("Stopped scanning."));
    log_mutex->Unlock();
  } catch (URG::exception& e) {
    urg_mutex->Unlock();

    log_mutex->Lock();
    wxLogMessage(_T("Error while stoppign scanning: ") + wxString::FromAscii(e.what()));
    log_mutex->Unlock();
    return false;
  }
  return true;
}

bool
HokuyoTester::DoHandleScan()
{
  try
  {
    urg_mutex->Lock();
    urg.service_scan(&scan, 1000);
    urg_mutex->Unlock();

    wxPaintEvent* e = new wxPaintEvent();
    gl->AddPendingEvent(*e);

  } catch (URG::timeout_exception& e) {
    urg_mutex->Unlock();
    return false;
  }
  return true;
}

void HokuyoTester::InitGL()
{
  gl->SetCurrent();

  int w,h;
  visPanel->GetClientSize(&w, &h);
  int m = (w < h) ? w : h;
  glViewport((GLint)((w-m)/2), (GLint)((h-m)/2), (GLint) m, (GLint) m);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-m/2, m/2, -m/2, m/2, -1, 1);
    glMatrixMode(GL_MODELVIEW);

}

void HokuyoTester::OnSize(wxSizeEvent& event)
{
    // this is also necessary to update the context on some platforms
    gl->OnSize(event);

    // set GL viewport (not called by wxGLCanvas::OnSize on all platforms...)
    int w, h;
    visPanel->GetClientSize(&w, &h);
    int m = (w < h) ? w : h;

    gl->SetCurrent();
    glViewport((GLint)((w-m)/2), (GLint)((h-m)/2), (GLint) m, (GLint) m);
}

void HokuyoTester::OnPaint( wxPaintEvent& WXUNUSED(event) )
{
    Render();
}

void HokuyoTester::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
  // Do nothing, to avoid flashing.
}

void HokuyoTester::Render()
{
    wxPaintDC dc(gl);

    gl->SetCurrent();

    // Init OpenGL once, but after SetCurrent
    if (!m_init)
    {
        InitGL();
        m_init = true;
    }

    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glScalef(view_scale, view_scale, view_scale);
    glTranslatef(view_x, view_y, 0);

    glPushMatrix();
    bool intensity_avail = false;
    double min_intensity = 1e9, max_intensity = -1e9;

    glPointSize(4.0);

    urg_mutex->Lock();
    for (size_t i = 0; i < scan.num_readings; i++)
    {
      glColor3f(0.1, 0.3, 0.1);
      glBegin(GL_LINES);
      glVertex2f(0,0);
      const double ang = scan.config.min_angle + scan.config.ang_increment * i;
      const double lx = scan.ranges[i] * cos(ang);
      const double ly = scan.ranges[i] * sin(ang);
      glVertex2f(lx, ly);
      glEnd();
    }
    urg_mutex->Unlock();
    glPopMatrix();

    glFlush();
    gl->SwapBuffers();
}
