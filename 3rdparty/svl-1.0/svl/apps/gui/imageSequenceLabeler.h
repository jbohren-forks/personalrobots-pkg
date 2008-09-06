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
** FILENAME:    imageSequenceLabeler.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  GUI for displaying and labeling objects in images. Uses OpenCV for decoding
**  images and video files, and wxWidgets for everything else.
**
*****************************************************************************/

#pragma once

#include "wx/wx.h"
#include "wx/glcanvas.h"
#include "wx/utils.h"

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "svlBase.h"
#include "svlVision.h"

// wxWidgets Event Constants --------------------------------------------------

enum
{
    FILE_NEW = wxID_HIGHEST,
    FILE_OPEN = wxID_HIGHEST + 1,
    FILE_OPEN_SEQUENCE = wxID_HIGHEST + 2,
    FILE_OPEN_DIRECTORY = wxID_HIGHEST + 3,
    FILE_OPEN_VIDEO = wxID_HIGHEST + 4,
    FILE_SAVE = wxID_HIGHEST + 10,
    FILE_SAVEAS = wxID_HIGHEST + 11,
    FILE_IMPORT = wxID_HIGHEST + 13,
    FILE_EXIT = wxID_EXIT,
    
    EDIT_FIND = wxID_HIGHEST + 100,
    EDIT_GOTO_FRAME = wxID_HIGHEST + 102,
    EDIT_GOTO_EMPTY = wxID_HIGHEST + 103,
        
    OPTIONS_UNDISTORT = wxID_HIGHEST + 200,
    OPTIONS_LOADCALIBRATION = wxID_HIGHEST + 201,
    OPTIONS_GRID = wxID_HIGHEST + 210,
    OPTIONS_CENTROID_MODE = wxID_HIGHEST + 215,
    OPTIONS_DEFAULT_NAME = wxID_HIGHEST + 230,
    OPTIONS_DEFAULT_SIZE = wxID_HIGHEST + 240,

    HELP_OBJECT_STATS = wxID_HIGHEST + 900,
    HELP_KEYBOARD = wxID_HIGHEST + 910,
    HELP_ABOUT = wxID_ABOUT,

    PLAY_TIMER_ID = wxID_HIGHEST + 1000
};

// Mouse Modes ----------------------------------------------------------------

typedef enum {
    MM_NONE, MM_MOVE, MM_SIZE_CORNER, MM_SIZE_WIDTH, MM_SIZE_HEIGHT
} TMouseMode;

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
    void on_timer(wxTimerEvent& event);

    void undistort(bool b);
    void drawGrid(bool b) { _bDrawGrid = b; this->Update(); this->Refresh(); }
    void centroidMode(bool b) { _bCentroidMode = b; }
    void setDefaultObjectName(std::string s) { _defaultObjectName = s; }
    void setDefaultObjectSize(std::string s);
    string getDefaultObjectSize() const;
    bool calibrate(const char *filename);

    void clearObjects();
    bool loadObjects(const char *filename);
    bool saveObjects(const char *filename) const;
    void importObjects(const char *filename);

    bool openSequence(const char *filename);
    bool openDirectory(const char *directory);
    bool openVideo(const char *filename);
    void closeSequence();

    bool gotoFrame(unsigned index);
    bool gotoEmpty();
    bool nextFrame();
    int getFrameIndex() const { return _index; }

    void findActiveObject();
    svlObject2dFrame *getCurrentFrame(int offset = 0);
    map<string, int> objectCounts();

 protected:
    int _index;
    svlObject2dSequence _objects;
    std::string _defaultObjectName;
    std::pair<int, int> _defaultObjectSize;

    unsigned char *_imageData;
    wxImage _image;
    double _scaleX, _scaleY;

    bool _bDrawGrid;
    int _activeObject;
    wxPoint _mouseDownPoint;
    TMouseMode _mouseMode;
    bool _bCentroidMode;

    svlImageSequence _imageSequence;
    wxTimer _playTimer;

    svlCameraIntrinsics _intrinsics;
    bool _bUndistort;

    DECLARE_EVENT_TABLE()
};

// MainWindow Class -----------------------------------------------------------

class ImageLabelerApp;

class MainWindow : public wxFrame
{
 friend class ImageLabelerApp;

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
    void on_close(wxCloseEvent& event);

 protected:
    MainCanvas *_canvas;
    std::string _objectFilename;

    DECLARE_EVENT_TABLE()
};

// ImageLabeler Application ---------------------------------------------------

class ImageLabelerApp : public wxApp
{
 public:
    bool OnInit();
    void OnInitCmdLine(wxCmdLineParser& parser);
    int OnExit();
};

// Global Variables -----------------------------------------------------------

extern MainWindow *gMainWindow;


