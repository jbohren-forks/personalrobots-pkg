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
** FILENAME:    pointCloudViewer.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  GUI for displaying 3d point clouds.
**
*****************************************************************************/

#pragma once

#include "wx/wx.h"
#include "wx/glcanvas.h"
#include "wx/utils.h"

#include "svlBase.h"
#include "svlVision.h"

// wxWidgets Event Constants --------------------------------------------------

enum
{
    FILE_NEW = wxID_HIGHEST,
    FILE_OPEN_POINTCLOUD = wxID_HIGHEST + 4,
    FILE_OPEN_TRAJECTORY = wxID_HIGHEST + 6,
    FILE_SAVE_VIEW = wxID_HIGHEST + 7,
    FILE_IMPORT_POINTCLOUD = wxID_HIGHEST + 10,
    FILE_EXPORT_POINTCLOUD = wxID_HIGHEST + 11,
    FILE_EXIT = wxID_EXIT,
    
    OPTIONS_MAX_POINTS = wxID_HIGHEST + 200,
    OPTIONS_COLOUR_NONE = wxID_HIGHEST + 210,
    OPTIONS_COLOUR_HEIGHT = wxID_HIGHEST + 211,
    OPTIONS_COLOUR_NORMAL = wxID_HIGHEST + 212,
    OPTIONS_COLOUR_BACKPROJECTED = wxID_HIGHEST + 213,
    OPTIONS_COLOUR_DEPTH = wxID_HIGHEST + 214,
    OPTIONS_COLOUR_WEIGHT = wxID_HIGHEST + 215,
    OPTIONS_POINTSIZE = wxID_HIGHEST + 220,
    OPTIONS_IMPORTPOINTCOLOUR = wxID_HIGHEST + 221,
    OPTIONS_IMPORTDATAWIDTH = wxID_HIGHEST + 222,
    OPTIONS_SHOWGROUNDPLANE = wxID_HIGHEST + 230,    
    OPTIONS_LEFTHANDEDCOORDINATES = wxID_HIGHEST + 240,
    OPTIONS_RIGHTHANDEDCOORDINATES = wxID_HIGHEST + 241,
        
    HELP_KEYBOARD = wxID_HIGHEST + 900,
    HELP_ABOUT = wxID_ABOUT,
};

// PointCloudCanvas Class -----------------------------------------------------

typedef enum {
    COLOUR_NONE, COLOUR_BY_HEIGHT, COLOUR_BY_NORMAL, COLOUR_BY_BACKPROJECTION,
    COLOUR_BY_DEPTH, COLOUR_BY_WEIGHT
} TPointColouring;

class PointCloudCanvas : public wxGLCanvas
{
 public:
    PointCloudCanvas(wxWindow *parent, wxWindowID id = wxID_ANY,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize &size = wxDefaultSize,
        long style = 0, const wxString &name = _T("PointCloudCanvas"));
    ~PointCloudCanvas();

    void on_erase_background(wxEraseEvent &event);
    void on_paint(wxPaintEvent &event);
    void on_size(wxSizeEvent &event);
    void on_key(wxKeyEvent &event);
    void on_mouse(wxMouseEvent &event);

    void saveImage(const char *filename);
    void updateCameraPosition(bool bUpdateStatusText = false);

    void updateRanges();

protected:
    void set_view();
    void render();

public:
    svlPointCloudData data;

    TPointColouring pointColouring;
    double pointSize;
    svlPoint3d importPointColour;
    int importDataWidth;
    bool bRightHandedCoordinates;
    bool bShowGroundPlane;

protected:
    double minDepth, maxDepth;
    double minWeight, maxWeight;

    int wndWidth, wndHeight;
    wxPoint lastMousePoint;

    svlPoint3d cameraUp;
    svlPoint3d cameraPosition;
    svlPoint3d cameraTarget;
    double cameraTheta;     // pan angle
    double cameraPhi;       // tilt angle
    double cameraRho;       // distance
    
    DECLARE_EVENT_TABLE()    
};

// MainWindow Class -----------------------------------------------------------

class PointCloudViewerApp;

class MainWindow : public wxFrame
{
 friend class PointCloudViewerApp;
 public:
    MainWindow(wxWindow* parent,
	  wxWindowID id,
	  const wxString& title,
	  const wxPoint& pos = wxDefaultPosition,
	  const wxSize& size = wxDefaultSize,
	  long style = wxDEFAULT_FRAME_STYLE | wxSUNKEN_BORDER);
    ~MainWindow();
 
    void on_key(wxKeyEvent &event);
    void on_file_menu(wxCommandEvent& event);
    void on_options_menu(wxCommandEvent& event);
    void on_help_menu(wxCommandEvent& event);
    void on_close(wxCloseEvent& event);
    
 protected:
    PointCloudCanvas *canvas;
 
    DECLARE_EVENT_TABLE()
};

// PointCloudViewer Application -----------------------------------------------

class PointCloudViewerApp : public wxApp
{
 public:
    bool OnInit();
    void OnInitCmdLine(wxCmdLineParser& parser);
    int OnExit();
};

// Global Variables -----------------------------------------------------------

extern MainWindow *gMainWindow;


