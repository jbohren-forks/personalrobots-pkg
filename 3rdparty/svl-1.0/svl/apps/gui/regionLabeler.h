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
** FILENAME:    regionLabeler.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  GUI for displaying and labeling regions in images.
**
*****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <map>

#include "wx/wx.h"
#include "wx/glcanvas.h"
#include "wx/utils.h"

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

// wxWidgets Event Constants --------------------------------------------------

enum
{
    FILE_NEW = wxID_HIGHEST,
    FILE_OPEN = wxID_HIGHEST + 1,
    FILE_OPEN_IMAGE = wxID_HIGHEST + 2,
    FILE_SAVE = wxID_HIGHEST + 10,
    FILE_SAVEAS = wxID_HIGHEST + 11,
    FILE_IMPORT_REGIONDEFS = wxID_HIGHEST + 20,
    FILE_EXIT = wxID_EXIT,

    EDIT_POLYGON_MODE = wxID_HIGHEST + 200,
    EDIT_BRUSH_MODE = wxID_HIGHEST + 201,
    EDIT_FILL_MODE = wxID_HIGHEST + 202,
    EDIT_CHANGE_LABEL = wxID_HIGHEST + 210,
    EDIT_UNDO = wxID_HIGHEST + 220,

    OPTIONS_GRID = wxID_HIGHEST + 300,
    OPTIONS_VIEW_IMAGE = wxID_HIGHEST + 310,
    OPTIONS_VIEW_REGIONS = wxID_HIGHEST + 311,
    OPTIONS_VIEW_EDGE_OVERLAY = wxID_HIGHEST + 312,
    OPTIONS_VIEW_REGIONS_OVERLAY = wxID_HIGHEST + 313,

    OPTIONS_DEFINE_LABELS = wxID_HIGHEST + 320,

    TOOLBAR_BASE = wxID_HIGHEST + 1000,

    HELP_ABOUT = wxID_ABOUT
};

// Region Definitions ---------------------------------------------------------

typedef struct _TRegionDef {
    std::string name;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
} TRegionDef;

// Mouse Modes ----------------------------------------------------------------

typedef enum {
    MM_NONE, MM_REGION
} TMouseMode;

// View Modes -----------------------------------------------------------------

typedef enum {
    VM_IMAGE, VM_REGIONS, VM_EDGE_OVERLAY, VM_REGION_OVERLAY
} TViewMode;

// Drawing Modes --------------------------------------------------------------

typedef enum {
    DM_POLYGON, DM_BRUSH, DM_FILL
} TDrawMode;

// MainCanvas Class -----------------------------------------------------------
// This is required under Linux because wxFrame doesn't get keyboard focus
// without a child window.

class MainCanvas : public wxWindow
{
 public:
    MainCanvas(wxWindow *parent,
	       wxWindowID id = wxID_ANY,
	       const wxPoint& pos = wxDefaultPosition,
	       const wxSize& size = wxDefaultSize,
	       long style = wxDEFAULT_FRAME_STYLE | wxSUNKEN_BORDER | wxWANTS_CHARS,
	       const wxString& name = wxPanelNameStr);
    ~MainCanvas();

    void on_erase_background(wxEraseEvent &event);
    void on_paint(wxPaintEvent &event);
    void on_size(wxSizeEvent &event);
    void on_key(wxKeyEvent &event);
    void on_mouse(wxMouseEvent &event);

    void drawGrid(bool b) { _bDrawGrid = b; this->Update(); this->Refresh(); }
    void setDrawMode(TDrawMode m);
    void setViewMode(TViewMode m);
    void setActiveLabel(int lbl);

    bool openImage(const char *filename);
    void newRegions();
    bool openRegions(const char *filename);
    bool saveRegions(const char *filename);
    bool openRegionDefs(const char *filename);

    void undo();

 protected:
    void undoableAction();

    void updateImageBuffer();
    void updateStatusBar();
    void updateToolBar();

    IplImage *_image;
    CvMat *_regions;
    unsigned char *_imageBuffer;

    int _activeLabel;
    TDrawMode _drawMode;
    TViewMode _viewMode;
    bool _bDrawGrid;
    TMouseMode _mouseMode;
    wxPoint _lastMousePoint;
    std::vector<CvPoint> _points;

    std::map<int, TRegionDef> _regionDefinitions;

    CvMat *_undoRegions;
    
    DECLARE_EVENT_TABLE()
};

// MainWindow Class -----------------------------------------------------------

class RegionLabelerApp;

class MainWindow : public wxFrame
{
 friend class RegionLabelerApp;

 public:
    MainWindow(wxWindow* parent,
	  wxWindowID id,
	  const wxString& title,
	  const wxPoint& pos = wxDefaultPosition,
	  const wxSize& size = wxDefaultSize,
	  long style = wxDEFAULT_FRAME_STYLE | wxSUNKEN_BORDER | wxWANTS_CHARS);
    ~MainWindow();

    // event callbacks
    void on_file_menu(wxCommandEvent& event);
    void on_edit_menu(wxCommandEvent& event);
    void on_options_menu(wxCommandEvent& event);
    void on_help_menu(wxCommandEvent& event);
    void on_toolbar(wxCommandEvent& event);
    void on_close(wxCloseEvent& event);

 protected:
    MainCanvas *_canvas;
    std::string _regionsFilename;

    DECLARE_EVENT_TABLE()
};

// RegionLabeler Application ---------------------------------------------------

class RegionLabelerApp : public wxApp
{
 public:
    bool OnInit();
    void OnInitCmdLine(wxCmdLineParser& parser);
    int OnExit();
};

// Global Variables -----------------------------------------------------------

extern MainWindow *gMainWindow;


