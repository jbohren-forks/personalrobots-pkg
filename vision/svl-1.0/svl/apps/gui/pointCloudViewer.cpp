/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2008, Stephen Gould
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    pointCloudViewer.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#include <winsock2.h>
#include "win32/dirent.h"
#else
#include <dirent.h>
#endif

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <limits>
#include <iostream>

#include <GL/glu.h>

#include "wx/wx.h"
#include "wx/glcanvas.h"
#include "wx/utils.h"
#include "wx/wxprec.h"
#include "wx/cmdline.h"
#include "wx/aboutdlg.h"

#include "svlBase.h"
#include "svlVision.h"

#include "pointCloudViewer.h"

using namespace std;

#define NOT_IMPLEMENTED_YET wxMessageBox(_T("Functionality not implementet yet."),\
					 _T("Error"), wxOK | wxICON_EXCLAMATION, this);

// Global Variables and Tables -------------------------------------------------

MainWindow *gMainWindow = NULL;

static const wxCmdLineEntryDesc COMMAND_LINE_DESCRIPTION[] =
{
    //{ wxCMD_LINE_SWITCH, "v", "verbose", "verbose" },

    { wxCMD_LINE_OPTION, "p", "pointcloud", "point cloud" },

    { wxCMD_LINE_NONE }
};

// Event Tables ----------------------------------------------------------------

BEGIN_EVENT_TABLE(PointCloudCanvas, wxGLCanvas)
    EVT_ERASE_BACKGROUND(PointCloudCanvas::on_erase_background)
    EVT_SIZE(PointCloudCanvas::on_size)
    EVT_PAINT(PointCloudCanvas::on_paint)
    EVT_CHAR(PointCloudCanvas::on_key)
    EVT_MOUSE_EVENTS(PointCloudCanvas::on_mouse)
END_EVENT_TABLE()

BEGIN_EVENT_TABLE(MainWindow, wxFrame)
	EVT_CHAR(MainWindow::on_key)
    EVT_MENU(FILE_NEW, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN_POINTCLOUD, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN_TRAJECTORY, MainWindow::on_file_menu)
    EVT_MENU(FILE_SAVE_VIEW, MainWindow::on_file_menu)
    EVT_MENU(FILE_IMPORT_POINTCLOUD, MainWindow::on_file_menu)
    EVT_MENU(FILE_EXPORT_POINTCLOUD, MainWindow::on_file_menu)
    EVT_MENU(FILE_EXIT, MainWindow::on_file_menu)
    EVT_MENU(OPTIONS_MAX_POINTS, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_COLOUR_NONE, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_COLOUR_HEIGHT, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_COLOUR_NORMAL, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_COLOUR_BACKPROJECTED, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_COLOUR_WEIGHT, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_COLOUR_DEPTH, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_SHOWGROUNDPLANE, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_POINTSIZE, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_IMPORTPOINTCOLOUR, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_IMPORTDATAWIDTH, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_LEFTHANDEDCOORDINATES, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_RIGHTHANDEDCOORDINATES, MainWindow::on_options_menu)
    EVT_MENU(HELP_KEYBOARD, MainWindow::on_help_menu)
    EVT_MENU(HELP_ABOUT, MainWindow::on_help_menu)
    EVT_CLOSE(MainWindow::on_close)
END_EVENT_TABLE()

// PointCloudCanvas Implementation ---------------------------------------------

int CANVAS_ARGS[] = { WX_GL_DOUBLEBUFFER, 0 };

PointCloudCanvas::PointCloudCanvas(wxWindow *parent, wxWindowID id,
    const wxPoint &pos, const wxSize &size, long style, const wxString &name) :
    wxGLCanvas(parent, (wxGLCanvas *)NULL, id, pos, size, 
        style | wxFULL_REPAINT_ON_RESIZE, 
        name, CANVAS_ARGS),
    wndWidth(1), wndHeight(1), lastMousePoint(0, 0),
    pointColouring(COLOUR_NONE), pointSize(1.0),
    importPointColour(0.0, 0.0, 1.0), importDataWidth(3),
    cameraUp(0.0, -1.0, 0.0), cameraTarget(0.0, 0.0, 0.0), 
    cameraTheta(M_PI), cameraPhi(0.0), cameraRho(2.0),
    bRightHandedCoordinates(true), bShowGroundPlane(false), data(1000000)
{
    updateCameraPosition();
}

PointCloudCanvas::~PointCloudCanvas()
{
  //do nothing
}

void PointCloudCanvas::on_erase_background(wxEraseEvent &event)
{
    // do nothing (and avoid flicker)
}

void PointCloudCanvas::on_paint(wxPaintEvent &WXUNUSED(event))
{
    wxPaintDC dc(this); // needed for MS Windows
    render();
}

void PointCloudCanvas::on_size(wxSizeEvent &event)
{
    wxGLCanvas::OnSize(event);
    GetClientSize(&wndWidth, &wndHeight);
    if (GetContext()) {
	set_view();
    }
}

void MainWindow::on_key(wxKeyEvent & event)
{
    canvas->on_key(event);
}

void PointCloudCanvas::on_key(wxKeyEvent &event)
{
    const double delta = 0.05;

    bool ctrlDown = event.GetModifiers() & wxMOD_CONTROL;
    ctrlDown = event.ControlDown();

    switch (event.m_keyCode) {
    case WXK_ESCAPE:
        this->GetParent()->Close();
	break;
    case 'a':
	cameraTarget.y += cameraUp.y * delta;
	break;
    case 'z':
	cameraTarget.y -= cameraUp.y * delta;
	break;
    case WXK_UP:
    case 'i':
	cameraTarget.x -= delta * cameraRho * sin(cameraTheta);
	cameraTarget.z -= delta * cameraRho * cos(cameraTheta);
        break;
    case WXK_DOWN:
    case 'k':
	cameraTarget.x += delta * cameraRho * sin(cameraTheta);
	cameraTarget.z += delta * cameraRho * cos(cameraTheta);
        break;
    case WXK_LEFT:
    case 'j':
	cameraTarget.x -= delta * cameraUp.y * cameraRho * cos(cameraTheta);
	cameraTarget.z += delta * cameraUp.y * cameraRho * sin(cameraTheta);
        break;
    case WXK_RIGHT:
    case 'l':
	cameraTarget.x += delta * cameraUp.y * cameraRho * cos(cameraTheta);
	cameraTarget.z -= delta * cameraUp.y * cameraRho * sin(cameraTheta);
        break;
    case 'x':
	cameraRho = 2.0;
	cameraTheta = M_PI;
	cameraPhi = 0.0;
	if (bRightHandedCoordinates) {
	    cameraUp = svlPoint3d(0.0, -1.0, 0.0);
	} else {
	    cameraUp = svlPoint3d(0.0, 1.0, 0.0);
	}
	cameraTarget.y = 0.0;
	break;

    case 'M': // magic key
        {
            wxDirDialog dlg(this, _T("Directory containing files to import:"), _T(""),
                wxDD_DIR_MUST_EXIST | wxDD_CHANGE_DIR);
            if (dlg.ShowModal() == wxID_OK) {
                DIR *dir = opendir(dlg.GetPath().c_str());
                assert(dir != NULL);
                struct dirent *e = readdir(dir);
                while (e != NULL) {
                    if (strstr(e->d_name, ".txt") != NULL) {
                        if (strncmp(e->d_name, "cloud", 5)) {
                            e = readdir(dir);
                            continue;
                        }
                        string filename = string(dlg.GetPath().c_str()) + string("/") + string(e->d_name);
                        data.clear();
                        data.import(filename.c_str(), true);
                        //data.read(filename.c_str());
                        this->Refresh(false);
                        this->Update();            
                        filename = filename.substr(0, filename.size() - 3) + string("bmp");
                        saveImage(filename.c_str());
                        
                    }
                    e = readdir(dir);
                }
                closedir(dir);
            }
        }
        break;

    default:
      event.Skip();
    }

    updateCameraPosition();

    // refresh view
    this->Refresh(false);
    this->Update();
}

void PointCloudCanvas::on_mouse(wxMouseEvent &event)
{
    int dx = event.m_x - lastMousePoint.x;
    int dy = event.m_y - lastMousePoint.y;

    lastMousePoint = wxPoint(event.m_x, event.m_y);

    ((MainWindow *)GetParent())->SetStatusText("");

    if (event.LeftIsDown() && event.Dragging())	{
	cameraTheta -= 0.01 * cameraUp.y * dx;
	if (bRightHandedCoordinates)
	    cameraPhi += 0.01 * dy;
	else cameraPhi -= 0.01 * dy;
	updateCameraPosition(true);
	this->Refresh(false);
	this->Update();
    } else if (event.RightIsDown() && event.Dragging()) {
	cameraRho *= (1.0 + 0.01 * dy);
	updateCameraPosition(true);
	this->Refresh(false);
	this->Update();
    }

}

void PointCloudCanvas::saveImage(const char *filename)
{
    wxBitmap bmp(wndWidth, wndHeight);
    wxClientDC dc(this);
    render();

    wxMemoryDC memDC;
    memDC.SelectObject(bmp);
    memDC.Blit(0, 0, wndWidth, wndHeight, &dc, 0, 0);
    bmp.SaveFile(filename, wxBITMAP_TYPE_BMP);
}

void PointCloudCanvas::updateCameraPosition(bool bUpdateStatusText)
{
    if (bRightHandedCoordinates) {
	cameraUp = svlPoint3d(0.0, -1.0, 0.0);
    } else {
	cameraUp = svlPoint3d(0.0, 1.0, 0.0);
    }

    cameraPosition.x = cameraTarget.x + cameraRho * cos(cameraPhi) * sin(cameraTheta);
    cameraPosition.y = cameraTarget.y + cameraUp.y * cameraRho * sin(cameraPhi);
    cameraPosition.z = cameraTarget.z + cameraRho * cos(cameraPhi) * cos(cameraTheta);

    if (bUpdateStatusText) {
	((MainWindow *)GetParent())->SetStatusText(
            wxString::Format("Looking at <%.2f, %.2f, %.2f> from %.2f m", 
                cameraTarget.x, cameraTarget.y, cameraTarget.z, cameraRho));
    }
}

void PointCloudCanvas::set_view()
{
    SetCurrent();
    glViewport(0, 0, (GLint)wndWidth, (GLint)wndHeight);
    double aspect_ratio = (double)wndWidth / wndHeight;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, aspect_ratio, 0.1, 50.0);
    glMatrixMode(GL_MODELVIEW);
}

void PointCloudCanvas::render()
{
    static bool initialized = false;
    if (!initialized) {
        set_view();
        initialized = true;
    }

    // clear buffer
    SetCurrent();

    //TODO [IG]-- move this to some initialization place
    glEnable(GL_DEPTH_TEST);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // save current transformation
    glPushMatrix();
    gluLookAt(cameraPosition.x, cameraPosition.y, cameraPosition.z,
	      cameraTarget.x, cameraTarget.y, cameraTarget.z,
	      cameraUp.x, cameraUp.y, cameraUp.z);

    // draw ground plane
    if (bShowGroundPlane) {
	glLineWidth(1.0);
	glColor3f(0.0, 0.5, 0.0);
	glBegin(GL_LINES);
	for (double i = -10.0; i < 10.0; i += 0.25) {
	    glVertex3f(-10.0, 0, i);
	    glVertex3f(10, 0, i);	
	    glVertex3f(i, 0, -10);
	    glVertex3f(i, 0, 10);	
	}
	glEnd();
    }

    // draw point cloud
    // TO DO [SG]: maybe use vertex arrays to speed this up? Or list since it doesn't change.

    glColor3f(0.0, 0.0, 1.0);
    glPointSize(pointSize);
    glBegin(GL_POINTS);
    for (unsigned i = 0; i < data.pointCloud.size(); i++) {
        if (pointColouring == COLOUR_BY_HEIGHT) {
            glColor3f((data.pointCloud[i].y + 2.0)/4.0, 0.0, 1.0 - (data.pointCloud[i].y + 2.0)/4.0);
            //glColor3f(1.0 - fabs(data.pointCloud[i].x - cameraPosition.x)/10.0,
            //    1.0 - fabs(data.pointCloud[i].y - cameraPosition.y)/10.0,
            //    1.0 - fabs(data.pointCloud[i].z - cameraPosition.z)/10.0);
        } else if (pointColouring == COLOUR_BY_NORMAL) {
            //glColor3f((data.pointNormals[i].y + 1.0)/2.0, 1.0 - (data.pointNormals[i].y + 1.0)/2.0, 0.5);
            glColor3f((data.pointNormals[i].x + 1.0)/2.0, (data.pointNormals[i].y + 1.0)/2.0, 0.5);
        } else if (pointColouring == COLOUR_BY_BACKPROJECTION) {
            if (data.pointColours[i] == 0.0) {
                continue;
            }
            glColor3f(data.pointColours[i].x, data.pointColours[i].y, data.pointColours[i].z);
        } else if (pointColouring == COLOUR_BY_WEIGHT) {
            if (!data.pointWeights.empty()) {
                double w = (data.pointWeights[i] - minWeight) / (maxWeight - minWeight + 1.0e-6); 
                glColor3f(w, 0.0, 1.0 - w);
            } else glColor3f(0.0,0.0,1.0);
        } else if (pointColouring == COLOUR_BY_DEPTH) {       
            double z = (data.pointCloud[i].z - minDepth) / (maxDepth - minDepth + 1.0e-6); 
            glColor3f(z, 0.0, 1.0 - z);
        }
        glVertex3f(data.pointCloud[i].x, data.pointCloud[i].y, data.pointCloud[i].z);
    }
    glEnd();

    // draw robot path
    glColor3f(0.5, 0.8, 0.2);
    glLineWidth(1.0);
    glBegin(GL_LINE_STRIP);
    for (unsigned i = 0; i < data.robotPath.size(); i++) {
    	glVertex3f(data.robotPath[i].x, data.robotPath[i].y, data.robotPath[i].z);
    }
    glEnd();

    // draw look-at point
    glLineWidth(1.0);
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(cameraTarget.x, cameraTarget.y, cameraTarget.z - 0.05);
    glVertex3f(cameraTarget.x, cameraTarget.y, cameraTarget.z + 0.05);
    glVertex3f(cameraTarget.x - 0.05, cameraTarget.y, cameraTarget.z);
    glVertex3f(cameraTarget.x + 0.05, cameraTarget.y, cameraTarget.z);
    glColor3f(1.0, 1.0, 0.0);
    glVertex3f(cameraTarget.x, cameraTarget.y - 0.05, cameraTarget.z);
    glVertex3f(cameraTarget.x, cameraTarget.y + 0.05, cameraTarget.z);
    glEnd();

    // restore previous transformation
    glPopMatrix();

    // draw and swap buffers
    glFlush();
    SwapBuffers();
}

void PointCloudCanvas::updateRanges()
{
    pair<float, float> depthRange = data.getRange(Z_COORDINATE);
    minDepth = depthRange.first;
    maxDepth = depthRange.second;
    
    if (data.pointWeights.size()) {
        pair<double, double> weightRange = range(data.pointWeights);
        minWeight = weightRange.first;
        maxWeight = weightRange.second;
    }
}

// MainWindow Implementation ---------------------------------------------------

MainWindow::MainWindow(wxWindow* parent, wxWindowID id, const wxString& title,
    const wxPoint& pos, const wxSize& size, long style) : 
    wxFrame(parent, id, title, pos, size, style)
{
    SetMinSize(wxSize(320, 240));

    wxMenu *file_menu = new wxMenu;
    wxMenu *options_menu = new wxMenu;
    wxMenu *options_coord_submenu = new wxMenu;
    wxMenu *help_menu = new wxMenu;
    file_menu->Append(FILE_NEW, _T("&New\tCtrl-N"), _T("Clear object labels"));
    file_menu->Append(FILE_OPEN_POINTCLOUD, _T("&Open Point Cloud...\tCtrl-O"), _T("Open 3d point cloud file (with normal and colour data)"));
    file_menu->Append(FILE_OPEN_TRAJECTORY, _T("Open &Trajectory..."), _T("Open robot trajectory log"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_SAVE_VIEW, _T("Save &View..."), _T("Save image of current view"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_IMPORT_POINTCLOUD, _T("&Import Point Cloud...\tCtrl-I"), _T("Open 3d point cloud file (with points only)"));
    file_menu->Append(FILE_EXPORT_POINTCLOUD, _T("&Export Point Cloud...\tCtrl-E"), _T("Save 3d point cloud (with points only)"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_EXIT, _T("E&xit\tAlt-X"), _T("Exit this program"));
    options_menu->Append(OPTIONS_MAX_POINTS, _T("Set Maximum &Points..."), _T("Set maximum points in point cloud"));
    options_menu->AppendSeparator();
    options_menu->AppendRadioItem(OPTIONS_COLOUR_NONE, _T("Colour &None"));
    options_menu->AppendRadioItem(OPTIONS_COLOUR_HEIGHT, _T("Colour by &Height"));
    options_menu->AppendRadioItem(OPTIONS_COLOUR_NORMAL, _T("Colour by &Normals"));
    options_menu->AppendRadioItem(OPTIONS_COLOUR_BACKPROJECTED, _T("Colour by &Backprojection"));
    options_menu->AppendRadioItem(OPTIONS_COLOUR_WEIGHT, _T("Colour by &Weight"));
    options_menu->AppendRadioItem(OPTIONS_COLOUR_DEPTH, _T("Colour by &Depth"));
    options_menu->AppendSeparator();
    options_menu->AppendCheckItem(OPTIONS_SHOWGROUNDPLANE, _T("Show &Ground Plane"));
    options_menu->AppendSeparator();
    options_menu->Append(OPTIONS_POINTSIZE, _T("Set Point &Size..."), _T("Set rendering point size"));
    options_menu->Append(OPTIONS_IMPORTPOINTCOLOUR, _T("Set Import &Colour..."), _T("Set colour for imported points"));
    options_menu->Append(OPTIONS_IMPORTDATAWIDTH, _T("Set Import Data Width..."), _T("Set data width for imported points clouds"));
    options_menu->AppendSeparator();
    options_coord_submenu->AppendRadioItem(OPTIONS_RIGHTHANDEDCOORDINATES, _T("&Right-handed Coordinates"));
    options_coord_submenu->AppendRadioItem(OPTIONS_LEFTHANDEDCOORDINATES, _T("&Left-handed Coordinates"));
    options_menu->Append(wxID_ANY, _T("&Coordinates"), options_coord_submenu, _T("Select coordinate system"));
    help_menu->Append(HELP_KEYBOARD, _T("&Keyboard"), _T("Show keyboard shortcuts"));
    help_menu->Append(HELP_ABOUT, _T("&About...\tF1"), _T("Show about dialog"));

    wxMenuBar *menu_bar = new wxMenuBar();
    menu_bar->Append(file_menu, _T("&File"));
    menu_bar->Append(options_menu, _T("&Options"));
    menu_bar->Append(help_menu, _T("&Help"));
    SetMenuBar(menu_bar);

    CreateStatusBar();

    // setup 3d point cloud viewer canvas
    canvas = new PointCloudCanvas(this, wxID_ANY);
    canvas->SetFocus();
}

MainWindow::~MainWindow()
{
    delete canvas;
}

void MainWindow::on_file_menu(wxCommandEvent& event)
{
    if (event.GetId() == FILE_NEW) {
        canvas->data.clear();
    } else if (event.GetId() == FILE_OPEN_POINTCLOUD) {
        wxFileDialog dlg(this, _T("Choose point cloud file"), _T(""),
	    _T(""), _T("*.*"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
            canvas->data.read(dlg.GetPath().c_str());
            canvas->updateRanges();
            SetStatusText(wxString::Format("%d points loaded", canvas->data.pointCloud.size()));
        }
    } else if (event.GetId() == FILE_OPEN_TRAJECTORY) {
        NOT_IMPLEMENTED_YET;
    } else if (event.GetId() == FILE_SAVE_VIEW) {
        wxFileDialog dlg(this, _T("Save image of scene as:"), _T(""), 
	    _T(""), _T("BMP files (*.bmp)|*.bmp"), wxSAVE | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
	    canvas->saveImage(dlg.GetPath().c_str());
        }
    } else if (event.GetId() == FILE_IMPORT_POINTCLOUD) {
        wxFileDialog dlg(this, _T("Choose point cloud file"), _T(""),
	    _T(""), _T("*.*"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
	    unsigned numPoints = canvas->data.pointColours.size();
            canvas->data.import(dlg.GetPath().c_str(), true, canvas->importDataWidth);
	    while (numPoints < canvas->data.pointColours.size()) {
		canvas->data.pointColours[numPoints] = canvas->importPointColour;
		numPoints++;
	    }
            canvas->updateRanges();
            SetStatusText(wxString::Format("%d points imported", canvas->data.pointCloud.size()));
        }
    } else if (event.GetId() == FILE_EXPORT_POINTCLOUD) {
        wxFileDialog dlg(this, _T("Export point cloud as:"), _T(""), 
	    _T(""), _T("*.*"), wxSAVE | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
            canvas->data.write(dlg.GetPath().c_str(), true);
            SetStatusText(wxString::Format("%d points exported", canvas->data.pointCloud.size()));
        }
    } else if (event.GetId() == FILE_EXIT) {
        Close(true);
    }

    canvas->Refresh(false);
    canvas->Update();
}

void MainWindow::on_options_menu(wxCommandEvent& event)
{
    if (event.GetId() == OPTIONS_MAX_POINTS) {
        wxTextEntryDialog dlg(this, _T("Enter maximum number of points in 3d point cloud:"),
            _T("Maximum Points"), wxString::Format("%d", canvas->data.getMaxPoints()));
        if (dlg.ShowModal() == wxID_OK) {
            canvas->data.setMaxPoints(atoi(dlg.GetValue().c_str()));
        }
    } else if (event.GetId() == OPTIONS_COLOUR_NONE) {
        canvas->pointColouring = COLOUR_NONE;
    } else if (event.GetId() == OPTIONS_COLOUR_HEIGHT) {
        canvas->pointColouring = COLOUR_BY_HEIGHT;
    } else if (event.GetId() == OPTIONS_COLOUR_NORMAL) {
        canvas->pointColouring = COLOUR_BY_NORMAL;
    } else if (event.GetId() == OPTIONS_COLOUR_BACKPROJECTED) {
        canvas->pointColouring = COLOUR_BY_BACKPROJECTION;
    } else if (event.GetId() == OPTIONS_COLOUR_WEIGHT) {
        canvas->pointColouring = COLOUR_BY_WEIGHT;
    } else if (event.GetId() == OPTIONS_COLOUR_DEPTH) {
	canvas->pointColouring = COLOUR_BY_DEPTH;
    } else if (event.GetId() == OPTIONS_SHOWGROUNDPLANE) {
	canvas->bShowGroundPlane = event.IsChecked();
    } else if (event.GetId() == OPTIONS_POINTSIZE) {
	wxTextEntryDialog dlg(this, _T("Enter point size for point cloud:"),
	    _T("Point Size"), wxString::Format("%f", canvas->pointSize));
        if (dlg.ShowModal() == wxID_OK) {
            canvas->pointSize = atof(dlg.GetValue().c_str());
            if (canvas->pointSize < 1.0)
                canvas->pointSize = 1.0;
        }
    } else if (event.GetId() == OPTIONS_IMPORTPOINTCOLOUR) {
	wxTextEntryDialog dlg(this, _T("Enter colour for imported points:"),
	    _T("Point Colour"), wxString::Format("%f %f %f", canvas->importPointColour.x,
		canvas->importPointColour.y, canvas->importPointColour.z));
        if (dlg.ShowModal() == wxID_OK) {
	    vector<double> v;
	    parseString(dlg.GetValue().c_str(), v);
	    if (v.size() == 3) {
		canvas->importPointColour = svlPoint3d(v[0], v[1], v[2]);
	    }
        }       
    } else if (event.GetId() == OPTIONS_IMPORTDATAWIDTH) {
	wxTextEntryDialog dlg(this, _T("Enter data width for imported points. "
                "First three components will be imported."),
	    _T("Import Width"), wxString::Format("%d", canvas->importDataWidth));
        if (dlg.ShowModal() == wxID_OK) {
            canvas->importDataWidth = atoi(dlg.GetValue().c_str());
            if (canvas->importDataWidth < 3) canvas->importDataWidth = 3;
        }       
    } else if (event.GetId() == OPTIONS_LEFTHANDEDCOORDINATES) {
	canvas->bRightHandedCoordinates = false;
	canvas->updateCameraPosition();
    } else if (event.GetId() == OPTIONS_RIGHTHANDEDCOORDINATES) {
	canvas->bRightHandedCoordinates = true;
	canvas->updateCameraPosition();
    } 

    canvas->Refresh(false);
    canvas->Update();
}

void MainWindow::on_help_menu(wxCommandEvent& event)
{
    if (event.GetId() == HELP_KEYBOARD) {
        wxMessageBox(_T("Use the mouse to control pan, tilt and zoom of camera. "
			"Keys (a,z,i,j,k,l) control view location. "
			"<esc> quits application."),
            _T("Keyboard Help"), wxOK | wxICON_INFORMATION, this);
    } else if (event.GetId() == HELP_ABOUT) {
        wxAboutDialogInfo info;
        info.SetName(_("STAIR's 3d Point Cloud Viewer"));
        info.SetVersion(_("0.3"));
        info.SetDescription(_("This program allows you to view 3D point clouds."));
        info.SetCopyright(_T("(C) 2007-2008 Staphen Gould <sgould@stanford.edu>"));

        wxAboutBox(info);
    }
}

void MainWindow::on_close(wxCloseEvent& event)
{
    // not implemented yet
    event.Skip();
}

// PointCloudViewerApp Implementation ------------------------------------------

bool PointCloudViewerApp::OnInit()
{
    // setup main window
    gMainWindow = new MainWindow(NULL, wxID_ANY, wxT("STAIR 3d Point Cloud Viewer GUI"),
        wxDefaultPosition, wxSize(640, 480));
    SetTopWindow(gMainWindow);
    gMainWindow->Show();
    gMainWindow->SetFocus();

    // call base class for command-line options
    wxApp::OnInit();

    return true;
}

void PointCloudViewerApp::OnInitCmdLine(wxCmdLineParser& parser)
{
    parser.SetDesc(COMMAND_LINE_DESCRIPTION);
    if (parser.Parse(true)) {
	exit(1);
    }

    wxString str;
    if (parser.Found("p", &str)) {
	cerr << "Loading point cloud...";
	gMainWindow->canvas->data.read(str.c_str());
	cerr << "done" << endl;
    }
}

int PointCloudViewerApp::OnExit()
{
    return 0;
}

IMPLEMENT_APP(PointCloudViewerApp)


