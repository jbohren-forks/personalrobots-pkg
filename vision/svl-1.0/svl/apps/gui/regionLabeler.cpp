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
** FILENAME:    regionLabeler.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
** In POLYGON mode, left-click places point on contour, double-click
** completes the contour.
** In PAINT mode, left-click paints region.
** In FILL mode, left-click nominated region to fill.
**
** Keyboard:
**  TAB cycles through labels
**  'v' cycles through views
**  'd' cycles through drawing modes (hold ctrl in brush mode for fill)
**
** TODO:
**  2. change size of brush in brush mode
**
*****************************************************************************/

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#undef max
#undef min
#endif

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <deque>

#include "wx/wx.h"
#include "wx/utils.h"
#include "wx/wxprec.h"
#include "wx/cmdline.h"
#include "wx/aboutdlg.h"

#include "xmlParser/xmlParser.h"

#include "regionLabeler.h"

using namespace std;

#ifndef CV_LOAD_IMAGE_GRAYSCALE
#define CV_LOAD_IMAGE_GRAYSCALE 0
#define CV_LOAD_IMAGE_COLOR 1
#endif

#define NOT_IMPLEMENTED_YET wxMessageBox(_T("Functionality not implementet yet."),\
	_T("Error"), wxOK | wxICON_EXCLAMATION, this);

// Global Variables and Tables -------------------------------------------------

MainWindow *gMainWindow = NULL;

static const wxCmdLineEntryDesc COMMAND_LINE_DESCRIPTION[] =
{
    { wxCMD_LINE_OPTION, "d", "definitions", "region definitions" },
    { wxCMD_LINE_OPTION, "i", "image", "image" },
    { wxCMD_LINE_OPTION, "r", "regions", "region labels" },

    { wxCMD_LINE_NONE }
};

// Event Tables ----------------------------------------------------------------

BEGIN_EVENT_TABLE(MainCanvas, wxWindow)
    EVT_ERASE_BACKGROUND(MainCanvas::on_erase_background)
    EVT_SIZE(MainCanvas::on_size)
    EVT_PAINT(MainCanvas::on_paint)
    EVT_CHAR(MainCanvas::on_key)
    EVT_MOUSE_EVENTS(MainCanvas::on_mouse)
END_EVENT_TABLE()

BEGIN_EVENT_TABLE(MainWindow, wxFrame)
    EVT_CLOSE(MainWindow::on_close)

    EVT_MENU(FILE_NEW, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN_IMAGE, MainWindow::on_file_menu)
    EVT_MENU(FILE_SAVE, MainWindow::on_file_menu)
    EVT_MENU(FILE_SAVEAS, MainWindow::on_file_menu)
    EVT_MENU(FILE_IMPORT_REGIONDEFS, MainWindow::on_file_menu)
    EVT_MENU(FILE_EXIT, MainWindow::on_file_menu)

    EVT_MENU(EDIT_POLYGON_MODE, MainWindow::on_edit_menu)
    EVT_MENU(EDIT_BRUSH_MODE, MainWindow::on_edit_menu)
    EVT_MENU(EDIT_FILL_MODE, MainWindow::on_edit_menu)
    EVT_MENU(EDIT_CHANGE_LABEL, MainWindow::on_edit_menu)
    EVT_MENU(EDIT_UNDO, MainWindow::on_edit_menu)

    EVT_MENU(OPTIONS_GRID, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_VIEW_IMAGE, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_VIEW_REGIONS, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_VIEW_EDGE_OVERLAY, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_VIEW_REGIONS_OVERLAY, MainWindow::on_options_menu)

    EVT_TOOL_RANGE(TOOLBAR_BASE, TOOLBAR_BASE + 1000, MainWindow::on_toolbar)

    EVT_MENU(HELP_ABOUT, MainWindow::on_help_menu)
END_EVENT_TABLE()

// MainCanvas Implementation ---------------------------------------------------

MainCanvas::MainCanvas(wxWindow *parent, wxWindowID id,const wxPoint& pos,
    const wxSize& size, long style, const wxString& name) :
    _image(NULL), _regions(NULL), _imageBuffer(NULL), _activeLabel(-1),
    _drawMode(DM_BRUSH), _viewMode(VM_IMAGE), _bDrawGrid(false),
    _mouseMode(MM_NONE), _undoRegions(NULL),
    wxWindow(parent, id, pos, size, style, name)
{
    SetMinSize(wxSize(320, 240));

    // set initial definitions
    _regionDefinitions[0].name = "background";
    _regionDefinitions[0].red = 0x00;
    _regionDefinitions[0].green = 0x00;
    _regionDefinitions[0].blue = 0x00;

    _regionDefinitions[1].name = "foreground";
    _regionDefinitions[1].red = 0xff;
    _regionDefinitions[1].green = 0x00;
    _regionDefinitions[1].blue = 0x00;

    updateToolBar();
}

MainCanvas::~MainCanvas()
{
    if (_image != NULL) {
	    cvReleaseImage(&_image);
	    cvReleaseMat(&_regions);
	    delete[] _imageBuffer;
    }

    if (_undoRegions != NULL) {
	    cvReleaseMat(&_undoRegions);
    }

    // TO DO: warn if not saved
}

void MainCanvas::on_erase_background(wxEraseEvent &event)
{
    // do nothing (and avoid flicker)
}

void MainCanvas::on_paint(wxPaintEvent &WXUNUSED(event))
{
    int width, height;
    GetClientSize(&width, &height);

    wxPaintDC dc(this);
    if (_image == NULL) {
        dc.Clear();
        dc.SetTextForeground(wxColor(0, 0, 255));
        wxSize s = dc.GetTextExtent("no video");
        dc.DrawText("no image", (int)(width - s.x)/2, (int)(height - s.y)/2);
	    return;
    }

    // copy frame to screen
    wxImage screenImage(_image->width, _image->height);
    screenImage.SetData(_imageBuffer, true);
    dc.DrawBitmap(screenImage.Scale(width, height), 0, 0);

    // draw grid
    if (_bDrawGrid) {
        dc.SetPen(wxPen(wxColor(255, 0, 255)));
        dc.DrawLine(width/2, 0, width/2, height - 1);
        dc.DrawLine(0, height/2, width - 1, height/2);
        dc.SetPen(wxPen(wxColor(255, 127, 255), 1, wxSHORT_DASH));
        for (unsigned i = 1; i < 5; i++) {
            dc.DrawLine(i * width/10, 0, i * width/10, height - 1);
            dc.DrawLine(i * width/10 + width/2, 0, i * width/10 + width/2, height - 1);
            dc.DrawLine(0, i * height/10, width - 1, i * height/10);
            dc.DrawLine(0, i * height/10 + height/2, width - 1, i * height/10 + height/2);
        }
    }

    // draw contour in POLYGON mode
    if ((_drawMode == DM_POLYGON) && (!_points.empty())) {
	double scaleX = (double)width / (double)_image->width;
	double scaleY = (double)height / (double)_image->height;

	// TO DO: fix colour
        dc.SetPen(wxPen(wxColor(255, 255, 255)));
	for (unsigned i = 1; i < _points.size(); i++) {
	    dc.DrawLine((int)(scaleX * _points[i - 1].x), (int)(scaleY * _points[i - 1].y),
		(int)(scaleX * _points[i].x), (int)(scaleY * _points[i].y));
	}
	dc.DrawLine((int)(scaleX * _points.back().x), (int)(scaleY * _points.back().y),
	    _lastMousePoint.x, _lastMousePoint.y);
    }   
}

void MainCanvas::on_size(wxSizeEvent &event)
{
    int width, height;

    GetClientSize(&width, &height);

    this->Refresh(false);
    this->Update();
}

void MainCanvas::on_key(wxKeyEvent &event)
{
    switch (event.m_keyCode) {
    case WXK_ESCAPE:
	_mouseMode = MM_NONE;
	_points.clear();
    	break;
    case WXK_DELETE:
	// TO DO: clear current region
	break;
    case WXK_TAB:
    {
	// TO DO: shift goes backwards
	map<int, TRegionDef>::const_iterator it = _regionDefinitions.find(_activeLabel);
	if (it == _regionDefinitions.end()) {
	    if (_regionDefinitions.empty()) {
		_activeLabel = -1;
	    } else {
		_activeLabel = _regionDefinitions.begin()->first;
	    }
	} else {
	    if (++it == _regionDefinitions.end()) {
		_activeLabel = -1;
	    } else {
		_activeLabel = it->first;
	    }
	}
	break;
    }
    case 'd':
    case 'D':
	if (_drawMode != DM_FILL) {
	    setDrawMode((TDrawMode)(_drawMode + 1));
	} else {
	    setDrawMode(DM_POLYGON);
	}
	break;
    case 'v':
    case 'V':
	if (_viewMode != VM_REGION_OVERLAY) {
	    setViewMode((TViewMode)(_viewMode + 1));
	} else {
	    setViewMode(VM_IMAGE);
	}
	break;
    default:
    	event.Skip();
    }

    // refresh view
    updateStatusBar();
    this->Refresh(false);
    this->Update();
}

void MainCanvas::on_mouse(wxMouseEvent &event)
{
    if (_image == NULL)
	return;

    int width, height;
    GetClientSize(&width, &height);
    double scaleX = (double)_image->width / (double)width;
    double scaleY = (double)_image->height / (double)height;

    if (event.LeftUp()) {
	if (_mouseMode == MM_REGION) {
	    _points.push_back(cvPoint((int)(scaleX * event.m_x),
		    (int)(scaleY * event.m_y)));
	}
    } if (event.LeftDown()) {
	if ((_drawMode == DM_POLYGON) && (_mouseMode != MM_REGION)) {	    
	    _mouseMode = MM_REGION;
	    _points.clear();
	} else if ((_drawMode == DM_BRUSH) && (!event.m_controlDown)) {
	    undoableAction();
	    for (int y = (int)(scaleY * (event.m_y - 5));
		 y < (int)(scaleY * (event.m_y + 5)); y++) {
		if ((y < 0) || (y >= _regions->rows)) continue;
		for (int x = (int)(scaleX * (event.m_x - 5));
		     x < (int)(scaleX * (event.m_x + 5)); x++) {
		    if ((x < 0) || (x >= _regions->cols)) continue;
		    CV_MAT_ELEM(*_regions, int, y, x) = _activeLabel;
		}
	    }
	    updateImageBuffer();
	} else if ((_drawMode == DM_FILL) ||
	    ((_drawMode == DM_BRUSH) && (event.m_controlDown))) {
	    undoableAction();
	    deque<CvPoint> frontier;
	    frontier.push_back(cvPoint((int)(scaleX * event.m_x),
				   (int)(scaleY * event.m_y)));
	    int fillLabel = CV_MAT_ELEM(*_regions, int, 
		frontier.front().y, frontier.front().x);
	    if (fillLabel == _activeLabel)
		frontier.clear();
	    while (!frontier.empty()) {
		CvPoint p = frontier.front();
		frontier.pop_front();
		if (CV_MAT_ELEM(*_regions, int, p.y, p.x) == _activeLabel)
		    continue;
		CV_MAT_ELEM(*_regions, int, p.y, p.x) = _activeLabel;
		if ((p.x > 0) && (CV_MAT_ELEM(*_regions, int, p.y, p.x - 1) == fillLabel))
		    frontier.push_back(cvPoint(p.x - 1, p.y));
		if ((p.x < _regions->cols - 1) && 
		    (CV_MAT_ELEM(*_regions, int, p.y, p.x + 1) == fillLabel))
		    frontier.push_back(cvPoint(p.x + 1, p.y));
		if ((p.y > 0) && (CV_MAT_ELEM(*_regions, int, p.y - 1, p.x) == fillLabel))
		    frontier.push_back(cvPoint(p.x, p.y - 1));
		if ((p.y < _regions->rows - 1) && 
		    (CV_MAT_ELEM(*_regions, int, p.y + 1, p.x) == fillLabel))
		    frontier.push_back(cvPoint(p.x, p.y + 1));
	    }
	    updateImageBuffer();
	}
    } else if (event.LeftDClick()) {
	if ((_drawMode == DM_POLYGON) && (_points.size() > 1)) {
	    _points.push_back(cvPoint((int)(scaleX * event.m_x),
		    (int)(scaleY * event.m_y)));
	    // paint the region
	    undoableAction();
	    IplImage *mask = cvCreateImage(cvSize(_regions->cols, _regions->rows),
		IPL_DEPTH_8U, 1);
	    cvZero(mask);
	    CvPoint mu = _points.front();
	    for (unsigned i = 1; i < _points.size(); i++) {
		mu.x += _points[i].x;
		mu.y += _points[i].y;
		cvLine(mask, _points[i - 1], _points[i], cvScalar(1));
	    }
	    cvLine(mask, _points.back(), _points.front(), cvScalar(1));
	    mu.x /= _points.size();
	    mu.y /= _points.size();
	    cvFloodFill(mask, mu, cvScalar(1));
	    for (int y = 0; y < _regions->rows; y++) {
		for (int x = 0; x < _regions->cols; x++) {
		    if (CV_IMAGE_ELEM(mask, unsigned char, y, x) != 0) {
			CV_MAT_ELEM(*_regions, int, y, x) = _activeLabel;
		    }
		}
	    }
	    cvReleaseImage(&mask);
	    // update buffer
	    updateImageBuffer();
	    _points.clear();
	    _mouseMode = MM_NONE;
	}
    } else if (event.Dragging() && (_drawMode == DM_BRUSH)) {
	// TO DO: loop from lastMousePoint to event point
	double dt = 1.0 / (fabs(scaleX * (event.m_x - _lastMousePoint.x)) +
	    fabs(scaleY * (event.m_y - _lastMousePoint.y)) + 1.0);
	for (double t = 0.0; t < 1.0; t += dt) {
	    double x = event.m_x + t * (_lastMousePoint.x - event.m_x);
	    double y = event.m_y + t * (_lastMousePoint.y - event.m_y);

	    for (int v = (int)(scaleY * (y - 5)); v < (int)(scaleY * (y + 5)); v++) {
		if ((v < 0) || (v >= _regions->rows)) continue;
		for (int u = (int)(scaleX * (x - 5)); u < (int)(scaleX * (x + 5)); u++) {
		    if ((u < 0) || (u >= _regions->cols)) continue;
		    CV_MAT_ELEM(*_regions, int, v, u) = _activeLabel;
		}
	    }
	}
	updateImageBuffer();
    } else if (event.Moving()) {
	// TO DO: show region label
    }

    _lastMousePoint = wxPoint(event.m_x, event.m_y);

    event.Skip();
    updateStatusBar();
    this->Refresh(false);
    this->Update();
}

void MainCanvas::setDrawMode(TDrawMode m)
{
    _drawMode = m;
    switch (_drawMode) {
    case DM_POLYGON:
	SetCursor(wxCursor(wxCURSOR_PENCIL));
	break;
    case DM_BRUSH:
	SetCursor(wxCursor(wxCURSOR_PAINT_BRUSH));
	break;
    case DM_FILL:
	SetCursor(wxCursor(wxCURSOR_CROSS));
	break;
    default:
	SetCursor(*wxSTANDARD_CURSOR);
    }
    updateStatusBar();
}

void MainCanvas::setViewMode(TViewMode m)
{
    _viewMode = m;
    updateImageBuffer();
    updateStatusBar();
}

void MainCanvas::setActiveLabel(int lbl)
{
    if (_regionDefinitions.find(lbl) == _regionDefinitions.end())
	    _activeLabel = -1;
    else _activeLabel = lbl;
    _points.clear();
    updateStatusBar();
}

bool MainCanvas::openImage(const char *filename)
{
    assert(filename != NULL);

    if (_image != NULL) {
	    cvReleaseImage(&_image);
	    delete[] _imageBuffer;
    }
    _image = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
    _imageBuffer = new unsigned char[3 * _image->width * _image->height];
    newRegions();

    updateImageBuffer();
    return true;
}

void MainCanvas::newRegions()
{
    if (_regions != NULL) {
	    cvReleaseMat(&_regions);
	    _regions = NULL;
    }

    if (_undoRegions != NULL) {
	    cvReleaseMat(&_undoRegions);
	    _undoRegions = NULL;
    }
    
    if (_image != NULL) {
	    _regions = cvCreateMat(_image->height, _image->width, CV_32SC1);
	    cvSet(_regions, cvScalar(-1));
    }

    _points.clear();
    updateImageBuffer();
}

bool MainCanvas::openRegions(const char *filename)
{
    assert(filename != NULL);
    if (_image == NULL)
        return false;

    newRegions();

    ifstream ifs(filename);
    assert(!ifs.fail());
    int id;
    for (int y = 0; y < _regions->rows; y++) {
        for (int x = 0; x < _regions->cols; x++) {
            ifs >> id;
    	    CV_MAT_ELEM(*_regions, int, y, x) = id;
        }
    }
    ifs.close();

    _points.clear();
    updateImageBuffer();
    return true;
}

bool MainCanvas::saveRegions(const char *filename)
{
    assert(filename != NULL);
    if (_regions == NULL)
	    return false;
    
    ofstream ofs(filename);
    assert(!ofs.fail());
    for (int y = 0; y < _regions->rows; y++) {
        for (int x = 0; x < _regions->cols; x++) {
            if (x > 0) ofs << " ";
            ofs << CV_MAT_ELEM(*_regions, int, y, x);
        }
        ofs << "\n";
    }
    ofs.close();

    _points.clear();
    return true;
}

bool MainCanvas::openRegionDefs(const char *filename)
{
    assert(filename != NULL);
    _regionDefinitions.clear();

    XMLNode root = XMLNode::parseFile(filename, "regionDefinitions");
    if (root.isEmpty()) {
	    return false;
    }

    TRegionDef definition;
    for (int i = 0; i < root.nChildNode("region"); i++) {
	    XMLNode node = root.getChildNode("region", i);
        int id = atoi(node.getAttribute("id"));
	    definition.name = string(node.getAttribute("name"));
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        int colour1, colour2, colour3;
        if (sscanf(node.getAttribute("color"), "%d %d %d", 
                &colour1, &colour2, &colour3) != 3) {
            cerr << "ERROR: could not parse color for \"" << definition.name << "\" \"" << endl;
            return false;
        }
        definition.red = (unsigned char)colour1;
        definition.green = (unsigned char)colour2;
        definition.blue = (unsigned char)colour3;
#else
        if (sscanf(node.getAttribute("color"), "%hhd %hhd %hhd", 
                &definition.red, &definition.green, &definition.blue) != 3) {
            cerr << "ERROR: could not parse color for \"" << definition.name << "\" \"" << endl;
            return false;
        }
#endif
        if (_regionDefinitions.find(id) != _regionDefinitions.end()) {
            cerr << "WARNING: id for \"" << definition.name << "\" already defined" << endl;
        }
        _regionDefinitions[id] = definition;
    }

    _points.clear();
    updateStatusBar();
    updateToolBar();
    updateImageBuffer();
    return true;
}

void MainCanvas::undo()
{
    if (_undoRegions == NULL)
	    return;

    CvMat *oldRegions = _regions;
    _regions = _undoRegions;
    _undoRegions = oldRegions;
    
    updateImageBuffer();
}

void MainCanvas::undoableAction()
{
    if (_regions == NULL) return;
    if (_undoRegions != NULL) {
	    cvReleaseMat(&_undoRegions);
    }

    _undoRegions = cvCloneMat(_regions);
}

void MainCanvas::updateImageBuffer()
{
    if (_imageBuffer == NULL)
	return;

    if (_viewMode == VM_REGIONS) {
	unsigned char r, g, b;
	for (int y = 0; y < _image->height; y++) {
	    for (int x = 0; x < _image->width; x++) {
		int id = CV_MAT_ELEM(*_regions, int, y, x);
		if (_regionDefinitions.find(id) == _regionDefinitions.end()) {
		    r = g = b = 0x00;
		} else {
		    r = _regionDefinitions[id].red;
		    g = _regionDefinitions[id].green;
		    b = _regionDefinitions[id].blue;
		}
		_imageBuffer[3 * (y * _image->width + x) + 0] = r;
		_imageBuffer[3 * (y * _image->width + x) + 1] = g;
		_imageBuffer[3 * (y * _image->width + x) + 2] = b;
	    }
	}
    } else {
	for (int y = 0; y < _image->height; y++) {
	    for (int x = 0; x < _image->width; x++) {
		_imageBuffer[3 * (y * _image->width + x) + 0] =
		    _image->imageData[y * _image->widthStep + 3 * x + 2];
		_imageBuffer[3 * (y * _image->width + x) + 1] =
		    _image->imageData[y * _image->widthStep + 3 * x + 1];
		_imageBuffer[3 * (y * _image->width + x) + 2] =
		    _image->imageData[y * _image->widthStep + 3 * x + 0];
	    }
	}
    }

    unsigned char r, g, b;
    switch (_viewMode) {
    case VM_EDGE_OVERLAY:
	for (int y = 0; y < _image->height; y++) {
	    for (int x = 0; x < _image->width; x++) {
		int id = CV_MAT_ELEM(*_regions, int, y, x);
		if ((x == 0) || (x == _image->width - 1) ||
		    (y == 0) || (y == _image->height - 1) ||
		    (CV_MAT_ELEM(*_regions, int, y, x - 1) != id) ||
		    (CV_MAT_ELEM(*_regions, int, y - 1, x) != id) ||
		    (CV_MAT_ELEM(*_regions, int, y, x + 1) != id) ||
		    (CV_MAT_ELEM(*_regions, int, y + 1, x) != id)) {
		    if (_regionDefinitions.find(id) == 
			_regionDefinitions.end()) {
			r = g = b = 0x00;
		    } else {
			r = _regionDefinitions[id].red;
			g = _regionDefinitions[id].green;
			b = _regionDefinitions[id].blue;
		    }
		    _imageBuffer[3 * (y * _image->width + x) + 0] = r;
		    _imageBuffer[3 * (y * _image->width + x) + 1] = g;
		    _imageBuffer[3 * (y * _image->width + x) + 2] = b;
		} else {
		    _imageBuffer[3 * (y * _image->width + x) + 0] /= 2;
		    _imageBuffer[3 * (y * _image->width + x) + 1] /= 2;
		    _imageBuffer[3 * (y * _image->width + x) + 2] /= 2;
		}
	    }
	}
	break;
    case VM_REGION_OVERLAY:
	for (int y = 0; y < _image->height; y++) {
	    for (int x = 0; x < _image->width; x++) {
		int id = CV_MAT_ELEM(*_regions, int, y, x);
		if (_regionDefinitions.find(id) == 
		    _regionDefinitions.end()) {
		    r = g = b = 0x00;
		} else {
		    r = _regionDefinitions[id].red;
		    g = _regionDefinitions[id].green;
		    b = _regionDefinitions[id].blue;
		}
		_imageBuffer[3 * (y * _image->width + x) + 0] /= 2;
		_imageBuffer[3 * (y * _image->width + x) + 1] /= 2;
		_imageBuffer[3 * (y * _image->width + x) + 2] /= 2;
		_imageBuffer[3 * (y * _image->width + x) + 0] += (unsigned char)(0.5 * r);
		_imageBuffer[3 * (y * _image->width + x) + 1] += (unsigned char)(0.5 * g);
		_imageBuffer[3 * (y * _image->width + x) + 2] += (unsigned char)(0.5 * b);
	    }
	}
	break;
    default:
	    // do nothing
	    break;
    }
}

void MainCanvas::updateStatusBar()
{
    string activeLabelName = "<void>";
    if (_regionDefinitions.find(_activeLabel) != _regionDefinitions.end()) {
	    activeLabelName = _regionDefinitions[_activeLabel].name;
    }
    ((MainWindow *)GetParent())->SetStatusText(wxString::Format(
        "Current label: %d (%s)", _activeLabel, activeLabelName.c_str()));
}

void MainCanvas::updateToolBar()
{
    wxToolBar *toolbar = ((wxFrame *)this->GetParent())->GetToolBar();
    toolbar->ClearTools();
    toolbar->SetToolBitmapSize(wxSize(24, 24));

    wxMemoryDC memDC;
    wxBitmap *bmp; // reference counted so will be deleted automatically

    int maxId = 0;
    for (map<int, TRegionDef>::const_iterator it = _regionDefinitions.begin();
	    it != _regionDefinitions.end(); ++it) {
	    if (it->first < 0) continue;
        if (it->first > maxId) maxId = it->first;
	    bmp = new wxBitmap(24, 24);
	    memDC.SelectObject(*bmp);
	    memDC.SetBrush(wxBrush(wxColour(it->second.red, 
	        it->second.green, it->second.blue)));
	    memDC.DrawRectangle(0, 0, 24, 24);
	    toolbar->AddTool(TOOLBAR_BASE + it->first,
	        _T(it->second.name.c_str()), *bmp,
	        _T(it->second.name.c_str()));
    }

    bmp = new wxBitmap(24, 24);
	memDC.SelectObject(*bmp);
    memDC.SetBrush(*wxWHITE_BRUSH);
    memDC.SetPen(*wxBLACK_PEN);
    memDC.DrawRectangle(0, 0, 24, 24);
    memDC.DrawRectangle(1, 1, 22, 22);
    memDC.DrawLine(1, 1, 22, 22);
    memDC.DrawLine(1, 22, 22, 1);
	toolbar->AddTool(TOOLBAR_BASE + maxId + 1, _T("void"), *bmp, _T("void"));

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
    // win32 workaround
    bmp = new wxBitmap(24, 24);
	memDC.SelectObject(*bmp);
#endif

    toolbar->Realize();
}

// MainWindow Implementation ---------------------------------------------------

MainWindow::MainWindow(wxWindow* parent, wxWindowID id, const wxString& title,
    const wxPoint& pos, const wxSize& size, long style) : 
    wxFrame(parent, id, title, pos, size, style)
{
    wxMenu *file_menu = new wxMenu;
    wxMenu *edit_menu = new wxMenu;
    wxMenu *options_menu = new wxMenu;
    wxMenu *help_menu = new wxMenu;

    file_menu->Append(FILE_NEW, _T("&New\tCtrl-N"), _T("Clear region labels"));
    file_menu->Append(FILE_OPEN, _T("&Open...\tCtrl-O"), _T("Open region labels"));
    file_menu->Append(FILE_OPEN_IMAGE, _T("Open &Image...\tCtrl-I"), _T("Open image file"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_SAVE, _T("&Save\tCtrl-S"), _T("Save region labels"));
    file_menu->Append(FILE_SAVEAS, _T("Save &As..."), _T("Save region labels"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_IMPORT_REGIONDEFS, _T("&Region Definitions...\tCtrl-R"), _T("Import region definitions"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_EXIT, _T("E&xit\tAlt-X"), _T("Exit this program"));
    edit_menu->AppendRadioItem(EDIT_POLYGON_MODE, _T("&Polygon mode"), _T("Label in polygon mode"));
    edit_menu->AppendRadioItem(EDIT_BRUSH_MODE, _T("&Brush mode"), _T("Label in brush mode"));
    edit_menu->AppendRadioItem(EDIT_FILL_MODE, _T("&Fill mode"), _T("Label in flood fill mode"));
    edit_menu->AppendSeparator();
    edit_menu->Append(EDIT_CHANGE_LABEL, _T("&Change label..."), _T("Change current label"));
    edit_menu->AppendSeparator();
    edit_menu->Append(EDIT_UNDO, _T("&Undo\tCtrl-Z"), _T("Undo last change"));
    options_menu->AppendCheckItem(OPTIONS_GRID, _T("Show &Grid"), _T("Show or hide grid"));
    options_menu->AppendSeparator();
    options_menu->AppendRadioItem(OPTIONS_VIEW_IMAGE, _T("View &image"), _T("Set viewing mode"));
    options_menu->AppendRadioItem(OPTIONS_VIEW_REGIONS, _T("View &regions"), _T("Set viewing mode"));
    options_menu->AppendRadioItem(OPTIONS_VIEW_EDGE_OVERLAY, _T("View &boundaries"), _T("Set viewing mode"));
    options_menu->AppendRadioItem(OPTIONS_VIEW_REGIONS_OVERLAY, _T("View &overlay"), _T("Set viewing mode"));
    help_menu->Append(HELP_ABOUT, _T("&About...\tF1"), _T("Show about dialog"));

    wxMenuBar *menu_bar = new wxMenuBar();
    menu_bar->Append(file_menu, _T("&File"));
    menu_bar->Append(edit_menu, _T("&Edit"));
    menu_bar->Append(options_menu, _T("&Options"));
    menu_bar->Append(help_menu, _T("&Help"));
    SetMenuBar(menu_bar);

    edit_menu->Check(EDIT_BRUSH_MODE, true);
    options_menu->Check(OPTIONS_GRID, false);
    options_menu->Check(OPTIONS_VIEW_IMAGE, true);

    this->CreateStatusBar();
    this->CreateToolBar(wxNO_BORDER | wxHORIZONTAL | wxTB_FLAT);

    // this is required for keyboard focus under Linux
    _canvas = new MainCanvas(this);
    _canvas->setDrawMode(DM_BRUSH);
    _canvas->setViewMode(VM_IMAGE);
}

MainWindow::~MainWindow()
{
    // do nothing
}

void MainWindow::on_file_menu(wxCommandEvent& event)
{
    if (event.GetId() == FILE_NEW) {
        wxMessageDialog dlg(this, _T("Clear all regions?"),
            _T("New"), wxYES_NO | wxICON_QUESTION);
        if (dlg.ShowModal() == wxID_YES) {
            _canvas->newRegions();
            _regionsFilename = string("");
	    SetTitle("Image Region Labeler");
        }
    } else if (event.GetId() == FILE_OPEN) {
        wxFileDialog dlg(this, _T("Choose region file to open"), _T(""), 
	    _T(""), _T("Text files (*.txt)|*.txt|All files (*.*)|*.*"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
            _regionsFilename = dlg.GetPath();
	    _canvas->openRegions(_regionsFilename.c_str());
	    SetTitle((string("Image Region Labeler [") +
		    dlg.GetFilename() + string("]")).c_str());
        }
    } else if (event.GetId() == FILE_OPEN_IMAGE) {
        wxFileDialog dlg(this, _T("Choose image to open"), _T(""), 
	    _T(""), _T("Image files (*.jpg)|*.jpg|All files (*.*)|*.*"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
            _canvas->openImage(dlg.GetPath().c_str());
	    _regionsFilename.clear();
	    SetTitle((string("Image Region Labeler [") +
		    dlg.GetFilename() + string("]")).c_str());
        }
    } else if ((event.GetId() == FILE_SAVE) && (_regionsFilename.size())) {
	_canvas->saveRegions(_regionsFilename.c_str());
    } else if ((event.GetId() == FILE_SAVE) || (event.GetId() == FILE_SAVEAS)) {
        wxFileDialog dlg(this, _T("Choose region file to save"), _T(""), 
	    _T(""), _T("Text files (*.txt)|*.txt"), wxSAVE | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
	    _regionsFilename = dlg.GetPath().c_str();
	    _canvas->saveRegions(_regionsFilename.c_str());
	    SetTitle((string("Image Region Labeler [") +
		    dlg.GetFilename() + string("]")).c_str());
        }
    } else if (event.GetId() == FILE_IMPORT_REGIONDEFS) {
        wxFileDialog dlg(this, _T("Choose region definitions to open"), _T(""), 
	    _T(""), _T("XML files (*.xml)|*.xml|All files (*.*)|*.*"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
	    _canvas->openRegionDefs(dlg.GetPath().c_str());
        }
    } else if (event.GetId() == FILE_EXIT) {
        Close(true);
    }

    Refresh(false);
    Update();
}

void MainWindow::on_edit_menu(wxCommandEvent& event)
{
    if (event.GetId() == EDIT_POLYGON_MODE) {
	_canvas->setDrawMode(DM_POLYGON);
    } else if (event.GetId() == EDIT_BRUSH_MODE) {
	_canvas->setDrawMode(DM_BRUSH);
    } else if (event.GetId() == EDIT_FILL_MODE) {
	_canvas->setDrawMode(DM_FILL);
    } else if (event.GetId() == EDIT_CHANGE_LABEL) {
        wxTextEntryDialog dlg(this, "Enter label id:");
        if (dlg.ShowModal() == wxID_OK) {
	    _canvas->setActiveLabel(atoi(dlg.GetValue().c_str()));
        }
    } else if (event.GetId() == EDIT_UNDO) {
	_canvas->undo();
    }

    Refresh(false);
    Update();
}

void MainWindow::on_options_menu(wxCommandEvent& event)
{
    if (event.GetId() == OPTIONS_GRID) {
        _canvas->drawGrid(event.IsChecked());
    } else if (event.GetId() == OPTIONS_VIEW_IMAGE) {
	_canvas->setViewMode(VM_IMAGE);
    } else if (event.GetId() == OPTIONS_VIEW_REGIONS) {
	_canvas->setViewMode(VM_REGIONS);
    } else if (event.GetId() == OPTIONS_VIEW_EDGE_OVERLAY) {
	_canvas->setViewMode(VM_EDGE_OVERLAY);
    } else if (event.GetId() == OPTIONS_VIEW_REGIONS_OVERLAY) {
	_canvas->setViewMode(VM_REGION_OVERLAY);
    }

    Refresh(false);
    Update();
}

void MainWindow::on_help_menu(wxCommandEvent& event)
{
    if (event.GetId() == HELP_ABOUT) {
        wxAboutDialogInfo info;
        info.SetName(_("Region Labeler"));
        info.SetVersion(_("0.3"));
        info.SetDescription(_("This program allows you to label regions in an image."));
        info.SetCopyright(_T("(C) 2008 Staphen Gould <sgould@stanford.edu>"));

        wxAboutBox(info);
    }
}

void MainWindow::on_toolbar(wxCommandEvent& event)
{
    int toolId = event.GetId() - (int)TOOLBAR_BASE;
    _canvas->setActiveLabel(toolId);
}

void MainWindow::on_close(wxCloseEvent& event)
{
    // not implemented yet
    event.Skip();
}

// RegionLabelerApp Implementation --------------------------------------------

bool RegionLabelerApp::OnInit()
{
    // setup main window
    gMainWindow = new MainWindow(NULL, wxID_ANY, wxT("Image Region Labeler"),
        wxDefaultPosition, wxSize(640, 480));
    SetTopWindow(gMainWindow);
    gMainWindow->Show();
    gMainWindow->SetFocus();    

    // call base class for command-line options
    wxApp::OnInit();

    return true;
}

void RegionLabelerApp::OnInitCmdLine(wxCmdLineParser& parser)
{
    parser.SetDesc(COMMAND_LINE_DESCRIPTION);
    if (parser.Parse(true)) {
	exit(1);
    }

    wxString str;
    if (parser.Found("d", &str)) {
	gMainWindow->_canvas->openRegionDefs(str.c_str());
    }

    if (parser.Found("i", &str)) {
	gMainWindow->_canvas->openImage(str.c_str());
    }

    if (parser.Found("r", &str)) {
	gMainWindow->_regionsFilename = string(str.c_str());
	gMainWindow->_canvas->openRegions(str.c_str());
    }
}

int RegionLabelerApp::OnExit()
{
    return 0;
}

IMPLEMENT_APP(RegionLabelerApp)

