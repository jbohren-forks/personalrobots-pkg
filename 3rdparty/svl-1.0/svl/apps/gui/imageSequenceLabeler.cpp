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
** FILENAME:    imageSequenceLabeler.cpp
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <limits>
#include <iostream>
#include <map>

#include "wx/wx.h"
#include "wx/utils.h"
#include "wx/wxprec.h"
#include "wx/cmdline.h"
#include "wx/aboutdlg.h"
#include "wx/msgdlg.h"

#include "svlBase.h"
#include "svlVision.h"

#include "imageSequenceLabeler.h"

using namespace std;

#define NOT_IMPLEMENTED_YET wxMessageBox(_T("Functionality not implementet yet."),\
					 _T("Error"), wxOK | wxICON_EXCLAMATION, this);

// Global Variables and Tables -------------------------------------------------

MainWindow *gMainWindow = NULL;

static const wxCmdLineEntryDesc COMMAND_LINE_DESCRIPTION[] =
{
    { wxCMD_LINE_OPTION, "c", "calibration", "camera calibration coefficients filename" },
    { wxCMD_LINE_OPTION, "o", "objects", "object list XML filename" },
    { wxCMD_LINE_OPTION, "i", "images", "image sequence or video filename" },

    { wxCMD_LINE_NONE }
};

// Event Tables ----------------------------------------------------------------

BEGIN_EVENT_TABLE(MainCanvas, wxWindow)
    EVT_ERASE_BACKGROUND(MainCanvas::on_erase_background)
    EVT_SIZE(MainCanvas::on_size)
    EVT_PAINT(MainCanvas::on_paint)
    EVT_CHAR(MainCanvas::on_key)
    EVT_MOUSE_EVENTS(MainCanvas::on_mouse)

    EVT_TIMER(PLAY_TIMER_ID, MainCanvas::on_timer)
END_EVENT_TABLE()

BEGIN_EVENT_TABLE(MainWindow, wxFrame)
    EVT_CLOSE(MainWindow::on_close)

    EVT_MENU(FILE_NEW, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN_SEQUENCE, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN_DIRECTORY, MainWindow::on_file_menu)
    EVT_MENU(FILE_OPEN_VIDEO, MainWindow::on_file_menu)
    EVT_MENU(FILE_SAVE, MainWindow::on_file_menu)
    EVT_MENU(FILE_SAVEAS, MainWindow::on_file_menu)
    EVT_MENU(FILE_IMPORT, MainWindow::on_file_menu)
    EVT_MENU(FILE_EXIT, MainWindow::on_file_menu)

    EVT_MENU(EDIT_FIND, MainWindow::on_edit_menu)
    EVT_MENU(EDIT_GOTO_FRAME, MainWindow::on_edit_menu)
    EVT_MENU(EDIT_GOTO_EMPTY, MainWindow::on_edit_menu)

    EVT_MENU(OPTIONS_UNDISTORT, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_LOADCALIBRATION, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_GRID, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_CENTROID_MODE, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_DEFAULT_NAME, MainWindow::on_options_menu)
    EVT_MENU(OPTIONS_DEFAULT_SIZE, MainWindow::on_options_menu)

    EVT_MENU(HELP_KEYBOARD, MainWindow::on_help_menu)
    EVT_MENU(HELP_OBJECT_STATS, MainWindow::on_help_menu)
    EVT_MENU(HELP_ABOUT, MainWindow::on_help_menu)
END_EVENT_TABLE()

// MainCanvas Implementation ---------------------------------------------------

MainCanvas::MainCanvas(wxWindow *parent, wxWindowID id,const wxPoint& pos,
    const wxSize& size, long style, const wxString& name) :
    wxWindow(parent, id, pos, size, style, name),
    _index(-1), _defaultObjectName("[unknown]"), _defaultObjectSize(make_pair(32, 32)),
    _imageData(NULL), _scaleX(0), _scaleY(0), _bDrawGrid(false),
    _activeObject(-1), _mouseMode(MM_NONE), _bCentroidMode(false),
    _playTimer(this, PLAY_TIMER_ID), _bUndistort(true)
{
    SetMinSize(wxSize(320, 240));
    _image = wxImage(320, 240);

    // set default camera calibration
    calibrate(NULL);
}

MainCanvas::~MainCanvas()
{
    closeSequence();
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
    if (_index < 0) {
        dc.Clear();
        dc.SetTextForeground(wxColor(0, 0, 255));
        wxSize s = dc.GetTextExtent("no images");
        dc.DrawText("no images", (int)(width - s.x)/2, (int)(height - s.y)/2);
    } else {
        dc.DrawBitmap(_image.Scale(width, height), 0, 0);
    }

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

    // draw objects
    svlObject2dFrame *frame = this->getCurrentFrame();
    if (frame != NULL) {
        dc.SetBrush(*wxTRANSPARENT_BRUSH);
        for (unsigned i = 0; i < frame->size(); i++) {
            if (_activeObject == (int)i) {
                dc.SetPen(wxPen(wxColor(255, 0, 0), 2));
                dc.SetTextForeground(wxColor(255, 0, 0));
            } else {
                dc.SetPen(wxPen(wxColor(0, (int)(255 * (*frame)[i].pr), 0), 2));
                dc.SetTextForeground(wxColor(0, (int)(255 * (*frame)[i].pr), 0));
            }
            dc.DrawRectangle((int)(_scaleX * (*frame)[i].x),
                (int)(_scaleY * (*frame)[i].y),
                (int)(_scaleX * (*frame)[i].w),
                (int)(_scaleY * (*frame)[i].h));
            wxSize s = dc.GetTextExtent(wxString((*frame)[i].name.c_str()));
            dc.DrawText(wxString((*frame)[i].name.c_str()), 
                (int)(_scaleX * (*frame)[i].x) + 1,
                (int)(_scaleY * (*frame)[i].y) - s.GetHeight() - 1);
        }
    }

}

void MainCanvas::on_size(wxSizeEvent &event)
{
    int width, height;

    GetClientSize(&width, &height);
    _scaleX = (double)width / (double)_image.GetWidth();
    _scaleY = (double)height / (double)_image.GetHeight();

    this->Refresh(false);
    this->Update();
}

void MainCanvas::on_key(wxKeyEvent &event)
{
    if (_playTimer.IsRunning()) {
        _playTimer.Stop();
        return;
    }

    svlObject2dFrame *frame = this->getCurrentFrame();

    switch (event.m_keyCode) {
    case WXK_ESCAPE:
	    Close();
    	break;
    case WXK_DELETE:
        if (_index != -1) {
            if ((frame != NULL) && (_activeObject != -1)) {
                frame->erase(frame->begin() + _activeObject);
            } else if (frame != NULL) {
                wxMessageDialog dlg(this, _T("Delete all objects in this frame?"),
                    _T("Confirm"), wxYES_NO | wxICON_QUESTION);
                if (dlg.ShowModal() == wxID_YES) {
                    frame->clear();
                }
            }
        }
        break;
    case WXK_INSERT:
        // TO DO: don't copy repeated objects
        {
            svlObject2dFrame *prevFrame = this->getCurrentFrame(-1);
            if (prevFrame != NULL) {
                if (frame == NULL) {
                    _objects[strBaseName(_imageSequence[_index])] = *prevFrame;
                } else {
                    frame->insert(frame->end(), prevFrame->begin(), prevFrame->end());
                }
            }
        }
        break;
    case ' ':
        nextFrame();
        break;
    case 'p':
        _playTimer.Start(33);
        break;
    case '-':
    case '_':
        if ((_activeObject != -1) && (frame != NULL) &&
            ((*frame)[_activeObject].w > 2.0) &&
            ((*frame)[_activeObject].h > 2.0)) {
            (*frame)[_activeObject].x += 1.0;
            (*frame)[_activeObject].y += 1.0;
            (*frame)[_activeObject].w -= 2.0;
            (*frame)[_activeObject].h -= 2.0;
        }
        break;
    case '+':
    case '=':
        if (_activeObject != -1) {
            (*frame)[_activeObject].x -= 1.0;
            (*frame)[_activeObject].y -= 1.0;
            (*frame)[_activeObject].w += 2.0;
            (*frame)[_activeObject].h += 2.0;
        }
        break;
    default:
    	event.Skip();
    }

    // refresh view
    findActiveObject();
    this->Refresh(false);
    this->Update();
}

void MainCanvas::on_mouse(wxMouseEvent &event)
{
    const double TOLERANCE = 5.0;
    if ((_index < 0) || (_playTimer.IsRunning())) return;

    svlObject2dFrame *frame = this->getCurrentFrame();

    if (event.LeftUp()) {
        if ((_mouseMode == MM_SIZE_CORNER) || (_mouseMode == MM_SIZE_WIDTH) || (_mouseMode == MM_SIZE_HEIGHT)) {
            if ((fabs((*frame)[_activeObject].w) <= TOLERANCE) &&
                (fabs((*frame)[_activeObject].h) <= TOLERANCE)) {
                frame->erase(frame->begin() + _activeObject);
                _activeObject = -1;
            } else {
                if ((*frame)[_activeObject].w < 0.0) {
                    (*frame)[_activeObject].x += (*frame)[_activeObject].w;
                    (*frame)[_activeObject].w = - (*frame)[_activeObject].w;
                }
                if ((*frame)[_activeObject].h < 0.0) {
                    (*frame)[_activeObject].y += (*frame)[_activeObject].h;
                    (*frame)[_activeObject].h = - (*frame)[_activeObject].h;
                }
            }
        }
        _mouseMode = MM_NONE;
    } if (event.LeftDown() && (_activeObject == -1)) {
        _mouseDownPoint = wxPoint(event.m_x, event.m_y);
        _mouseMode = MM_SIZE_CORNER;
        if (frame == NULL) {
            _objects[strBaseName(_imageSequence[_index])] = svlObject2dFrame();
            frame = &_objects[strBaseName(_imageSequence[_index])];
        }
        frame->push_back(svlObject2d(event.m_x / _scaleX, event.m_y / _scaleY, 0.0, 0.0));
	    frame->back().name = _defaultObjectName;
        _activeObject = frame->size() - 1;
        if (_bCentroidMode) {
            frame->back().x -= _defaultObjectSize.first / 2;
            frame->back().y -= _defaultObjectSize.second / 2;
            frame->back().w = _defaultObjectSize.first;
            frame->back().h = _defaultObjectSize.second;
            _mouseMode = MM_MOVE;
        }
    } else if (event.LeftDown() && (_activeObject != -1)) {
        if (fabs(event.m_x - _scaleX * (*frame)[_activeObject].x) < TOLERANCE) {
            (*frame)[_activeObject].x += (*frame)[_activeObject].w;
            (*frame)[_activeObject].w = - (*frame)[_activeObject].w;
        }
        if (fabs(event.m_y - _scaleY * (*frame)[_activeObject].y) < TOLERANCE) {
            (*frame)[_activeObject].y += (*frame)[_activeObject].h;
            (*frame)[_activeObject].h = - (*frame)[_activeObject].h;
        }
        if ((fabs(event.m_x - _scaleX * ((*frame)[_activeObject].x + (*frame)[_activeObject].w)) < TOLERANCE) &&
            (fabs(event.m_y - _scaleY * ((*frame)[_activeObject].y + (*frame)[_activeObject].h)) < TOLERANCE)) {
            _mouseMode = MM_SIZE_CORNER;   
        } else if (fabs(event.m_x - _scaleX * ((*frame)[_activeObject].x + (*frame)[_activeObject].w)) < TOLERANCE) {
            _mouseMode = MM_SIZE_WIDTH;
        } else if (fabs(event.m_y - _scaleY * ((*frame)[_activeObject].y + (*frame)[_activeObject].h)) < TOLERANCE) {
            _mouseMode = MM_SIZE_HEIGHT;
        } else {
            _mouseMode = MM_MOVE;
        }
    	_mouseDownPoint = wxPoint(event.m_x, event.m_y);
    } else if (event.LeftDClick() && (_activeObject != -1)) {
        int objectIndex = _activeObject; // to prevent error on mouse move
        wxTextEntryDialog dlg(this, wxString::Format("Enter name for object at <%d, %d, %d, %d>", 
	        (int)(*frame)[objectIndex].x, (int)(*frame)[objectIndex].y,
            (int)(*frame)[objectIndex].w, (int)(*frame)[objectIndex].h),
            wxString("Object Name"), wxString((*frame)[objectIndex].name.c_str()));
        if (dlg.ShowModal() == wxID_OK) {
	        (*frame)[objectIndex].name = string(dlg.GetValue().c_str());
        }	
        findActiveObject();
    } else if (event.Dragging() && (_activeObject != -1)) {
        // resize or move object
        if (_mouseMode == MM_MOVE) {
            (*frame)[_activeObject].x += (event.m_x - _mouseDownPoint.x) / _scaleX;
            (*frame)[_activeObject].y += (event.m_y - _mouseDownPoint.y) / _scaleY;
        } else if (_mouseMode == MM_SIZE_CORNER) {
            (*frame)[_activeObject].w += (event.m_x - _mouseDownPoint.x) / _scaleX;
            (*frame)[_activeObject].h += (event.m_y - _mouseDownPoint.y) / _scaleY;
            if ((*frame)[_activeObject].w < 0.0) {
                (*frame)[_activeObject].x += (*frame)[_activeObject].w;
                (*frame)[_activeObject].w = - (*frame)[_activeObject].w;
            }
            if ((*frame)[_activeObject].h < 0.0) {
                (*frame)[_activeObject].y += (*frame)[_activeObject].h;
                (*frame)[_activeObject].h = - (*frame)[_activeObject].h;
            }
            // handle shift
            if (event.ShiftDown()) {
                if (((*frame)[_activeObject].w < (*frame)[_activeObject].h)) {
                    (*frame)[_activeObject].h = (*frame)[_activeObject].w;
                } else {
                    (*frame)[_activeObject].w = (*frame)[_activeObject].h;
                }
                event.m_x = (int)(((*frame)[_activeObject].x + (*frame)[_activeObject].w) * _scaleX);
                event.m_y = (int)(((*frame)[_activeObject].y + (*frame)[_activeObject].h) * _scaleY);
                WarpPointer(event.m_x, event.m_y);
            }
        } else if (_mouseMode == MM_SIZE_WIDTH) {
            (*frame)[_activeObject].w += (event.m_x - _mouseDownPoint.x) / _scaleX;
        } else if (_mouseMode == MM_SIZE_HEIGHT) {
            (*frame)[_activeObject].h += (event.m_y - _mouseDownPoint.y) / _scaleY;
        }
        _mouseDownPoint = wxPoint(event.m_x, event.m_y);
        // update status bar
        ((wxFrame *)GetParent())->SetStatusText(
            wxString::Format("Object: %s at (%d, %d) [%d x %d]",
            (*frame)[_activeObject].name.c_str(),
            (int)(*frame)[_activeObject].x, (int)(*frame)[_activeObject].y,
            (int)(*frame)[_activeObject].w, (int)(*frame)[_activeObject].h));
    } else if (event.Moving()) {
        int lastActive = _activeObject;
        findActiveObject();
        // check if we need to repaint
        if (_activeObject == lastActive) {
            return;
        }
    }

    event.Skip();
    this->Refresh(false);
    this->Update();
}

void MainCanvas::on_timer(wxTimerEvent& event)
{
    if (!nextFrame()) {
        _playTimer.Stop();
    }

    this->Refresh(false);
    this->Update();
}

void MainCanvas::undistort(bool b)
{
    _bUndistort = b;

    if (_imageData == NULL) {
        return;
    }

    // redisplay the image
    const IplImage *imgPtr;
    imgPtr = _imageSequence.image(_index);
    assert(imgPtr != NULL);

    if (_bUndistort) {
        imgPtr = _intrinsics.undistort(imgPtr);
    }

    assert((imgPtr->width == _image.GetWidth()) && (imgPtr->height == _image.GetHeight()));
    for (int y = 0; y < imgPtr->height; y++) {
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
        int yy = imgPtr->height - y - 1;
#else
        int yy = y;
#endif
        for (int x = 0; x < imgPtr->width; x++) {
            _imageData[3 * (yy * imgPtr->width + x) + 0] = imgPtr->imageData[y * imgPtr->widthStep + 3 * x + 2];
            _imageData[3 * (yy * imgPtr->width + x) + 1] = imgPtr->imageData[y * imgPtr->widthStep + 3 * x + 1];
            _imageData[3 * (yy * imgPtr->width + x) + 2] = imgPtr->imageData[y * imgPtr->widthStep + 3 * x + 0];
        }
    }

    _image.SetData(_imageData, true);

    // refresh view
    findActiveObject();
    this->Update();
    this->Refresh();
}

void MainCanvas::setDefaultObjectSize(std::string s)
{
    sscanf(s.c_str(), "%d %d", &_defaultObjectSize.first,
	&_defaultObjectSize.second);
}

string MainCanvas::getDefaultObjectSize() const
{
    return string(wxString::Format("%d %d", _defaultObjectSize.first,
	    _defaultObjectSize.second));    
}

bool MainCanvas::calibrate(const char *filename)
{
    if (filename == NULL) {
	// just some defaults for now
	_intrinsics.initialize(557.267, 568.585, 328.355, 238.273, 0.0,
	    0.0, 0.0, 0.0, 0.0);
    } else {
	_intrinsics.initialize(filename);
    }

    return true;
}

void MainCanvas::clearObjects()
{
    _objects.clear();
    _activeObject = -1;
}

bool MainCanvas::loadObjects(const char *filename)
{
    bool retVal = readObject2dSequence(filename, _objects);
    _activeObject = -1;
    ((MainWindow *)GetParent())->SetStatusText(wxString::Format("%d labeled frames loaded", _objects.size()));

    return retVal;
}

bool MainCanvas::saveObjects(const char *filename) const
{
    bool retVal = writeObject2dSequence(filename, _objects);
    ((MainWindow *)GetParent())->SetStatusText(wxString::Format("%d labeled frames saved", _objects.size()));

    return retVal;
}

void MainCanvas::importObjects(const char *filename)
{
    svlObject2dSequence newObjects;
    readObject2dSequence(filename, newObjects);

    for (svlObject2dSequence::const_iterator it = newObjects.begin(); it != newObjects.end(); ++it) {
        if (_objects.find(it->first) != _objects.end()) {
            _objects[it->first].insert(_objects[it->first].end(),
                it->second.begin(), it->second.end());
        } else {
            _objects[it->first] = it->second;
        }
    }

    ((MainWindow *)GetParent())->SetStatusText(wxString::Format("%d labeled frames imported", newObjects.size()));
}

bool MainCanvas::openSequence(const char *filename)
{
    assert(filename != NULL);
    closeSequence();

    _imageSequence.load(filename);
    nextFrame();

    return true;
}

bool MainCanvas::openDirectory(const char *directory)
{
    assert(directory != NULL);
    closeSequence();

    _imageSequence.dir(directory);

    nextFrame();

    ((MainWindow *)GetParent())->SetStatusText(wxString::Format("%d images read", _imageSequence.size()));
    return true;
}

bool MainCanvas::openVideo(const char *filename)
{
    NOT_IMPLEMENTED_YET;
    return false;
}

void MainCanvas::closeSequence()
{
    _imageSequence.clear();
    if (_imageData != NULL) {
        delete[] _imageData;
        _imageData = NULL;
    }

    _index = -1;
}

bool MainCanvas::gotoFrame(unsigned index)
{
    if (_imageSequence.empty()) {
        return false;
    }

    if ((int)index == _index) {
        return true;
    }

    _activeObject = -1;
    _index = index - 1;
    if (_index >= (int)_imageSequence.size()) {
        _index = (int)_imageSequence.size() - 1;
    }
    return nextFrame();
}

bool MainCanvas::gotoEmpty()
{
    if (_index < 0) {
	return false;
    }

    int k = _index;
    while (k < (int)_imageSequence.size()) {
        if (getCurrentFrame(k - _index) == NULL) {
	    break;
	}
	k++;
    }

    return gotoFrame(k);
}

bool MainCanvas::nextFrame()
{
    if (_imageSequence.empty()) {
        return false;
    }

    const IplImage *imgPtr;
    while (1) {
        if ((_index + 1) >= (int)_imageSequence.size())
            return false;
        cerr << "Reading image (" << (_index + 2) << " of "
             << _imageSequence.size() << ") " << _imageSequence[_index + 1] << endl;
        imgPtr = _imageSequence.image(_index + 1);
        
        if (imgPtr != NULL) break;
        _index += 1;
    }
    if (_imageData != NULL) {
        delete[] _imageData;
        _imageData = NULL;
    }
    cerr << "...image is " << toString(*imgPtr) << endl;
	
    if (_bUndistort) {
    	imgPtr = _intrinsics.undistort(imgPtr);
    }

    // copy frame into image (BGR format to RGB format)
    if (_imageData == NULL) {
	    _imageData = new unsigned char[3 * imgPtr->width * imgPtr->height];
        if ((_image.GetWidth() != imgPtr->width) || (_image.GetHeight() != imgPtr->height)) {
            //_image.Resize(wxSize(imgPtr->width, imgPtr->height), wxPoint(0, 0));
	        _image = wxImage(imgPtr->width, imgPtr->height);
        }

        int width, height;
        GetClientSize(&width, &height);
        _scaleX = (double)width / (double)_image.GetWidth();
        _scaleY = (double)height / (double)_image.GetHeight();
    }

    assert((imgPtr->width == _image.GetWidth()) && (imgPtr->height == _image.GetHeight()));
    if (imgPtr->nChannels == 3) {
	    for (int y = 0; y < imgPtr->height; y++) {
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
            int yy = imgPtr->height - y - 1;
#else
            int yy = y;
#endif
	        for (int x = 0; x < imgPtr->width; x++) {
	            _imageData[3 * (yy * imgPtr->width + x) + 0] = imgPtr->imageData[y * imgPtr->widthStep + 3 * x + 2];
    	        _imageData[3 * (yy * imgPtr->width + x) + 1] = imgPtr->imageData[y * imgPtr->widthStep + 3 * x + 1];
	            _imageData[3 * (yy * imgPtr->width + x) + 2] = imgPtr->imageData[y * imgPtr->widthStep + 3 * x + 0];
	        }
    	}
    } else {
	    for (int y = 0; y < imgPtr->height; y++) {
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
            int yy = imgPtr->height - y - 1;
#else
            int yy = y;
#endif
    	    for (int x = 0; x < imgPtr->width; x++) {
	            _imageData[3 * (yy * imgPtr->width + x) + 0] = imgPtr->imageData[y * imgPtr->widthStep + x];
	            _imageData[3 * (yy * imgPtr->width + x) + 1] = imgPtr->imageData[y * imgPtr->widthStep + x];
	            _imageData[3 * (yy * imgPtr->width + x) + 2] = imgPtr->imageData[y * imgPtr->widthStep + x];
	        }
	    }
    }

    _image.SetData(_imageData, true);
    _index += 1;
    _activeObject = -1;
    ((wxFrame *)GetParent())->SetStatusText(wxString::Format("Frame %d [%s]", 
            _index, _imageSequence[_index].c_str()));

    return true;
}

void MainCanvas::findActiveObject()
{
    const double TOLERANCE = 5.0;

    if (_activeObject != -1) {
        if (_index == -1) {
            ((wxFrame *)GetParent())->SetStatusText(_T(""));
        } else {
            ((wxFrame *)GetParent())->SetStatusText(wxString::Format("Frame %d [%s]", 
                    _index, _imageSequence[_index].c_str()));
        }
    }

    int lastActiveObject = _activeObject;
    _activeObject = -1;
    SetCursor(wxCURSOR_ARROW);
    svlObject2dFrame *frame = getCurrentFrame();
    if (frame == NULL) return;

    wxPoint mouse = this->ScreenToClient(::wxGetMousePosition());
    if ((lastActiveObject != -1) && (lastActiveObject < frame->size())) {
        if ((*frame)[lastActiveObject].hit(mouse.x / _scaleX, mouse.y / _scaleY)) {
            _activeObject = lastActiveObject;
            ((wxFrame *)GetParent())->SetStatusText(
                wxString::Format("Object: %s at (%d, %d) [%d x %d]",
                (*frame)[_activeObject].name.c_str(),
                (int)(*frame)[_activeObject].x, (int)(*frame)[_activeObject].y,
                (int)(*frame)[_activeObject].w, (int)(*frame)[_activeObject].h));
            bool left = (fabs(mouse.x - _scaleX * (*frame)[_activeObject].x) < TOLERANCE);
            bool right = (fabs(mouse.x - _scaleX * ((*frame)[_activeObject].x + (*frame)[_activeObject].w)) < TOLERANCE);
            bool top = (fabs(mouse.y - _scaleY * (*frame)[_activeObject].y) < TOLERANCE);
            bool bottom = (fabs(mouse.y - _scaleY * ((*frame)[_activeObject].y + (*frame)[_activeObject].h)) < TOLERANCE);
            if ((left && top) || (right && bottom)) {
                SetCursor(wxCURSOR_SIZENWSE);
            } else if ((left && bottom) || (right && top)) {
                SetCursor(wxCURSOR_SIZENESW);
            } else if (left || right) {
                SetCursor(wxCURSOR_SIZEWE);
            } else if (top || bottom) {
                SetCursor(wxCURSOR_SIZENS);
            }
            return;
        }
    }

    for (int i = frame->size() - 1; i >= 0; i--) {
        if ((*frame)[i].hit(mouse.x / _scaleX, mouse.y / _scaleY)) {
            _activeObject = i;
            ((wxFrame *)GetParent())->SetStatusText(
                wxString::Format("Object: %s at (%d, %d) [%d x %d]",
                (*frame)[_activeObject].name.c_str(),
                (int)(*frame)[_activeObject].x, (int)(*frame)[_activeObject].y,
                (int)(*frame)[_activeObject].w, (int)(*frame)[_activeObject].h));
            bool left = (fabs(mouse.x - _scaleX * (*frame)[_activeObject].x) < TOLERANCE);
            bool right = (fabs(mouse.x - _scaleX * ((*frame)[_activeObject].x + (*frame)[_activeObject].w)) < TOLERANCE);
            bool top = (fabs(mouse.y - _scaleY * (*frame)[_activeObject].y) < TOLERANCE);
            bool bottom = (fabs(mouse.y - _scaleY * ((*frame)[_activeObject].y + (*frame)[_activeObject].h)) < TOLERANCE);
            if ((left && top) || (right && bottom)) {
                SetCursor(wxCURSOR_SIZENWSE);
            } else if ((left && bottom) || (right && top)) {
                SetCursor(wxCURSOR_SIZENESW);
            } else if (left || right) {
                SetCursor(wxCURSOR_SIZEWE);
            } else if (top || bottom) {
                SetCursor(wxCURSOR_SIZENS);
            }
            break;
        }
    }
}

svlObject2dFrame * MainCanvas::getCurrentFrame(int offset)
{
    int indx = _index + offset;

    if ((indx < 0) || (indx >= (int)_imageSequence.size()))
        return NULL;

    svlObject2dSequence::iterator it = _objects.find(strBaseName(_imageSequence[indx]));
    if (it == _objects.end()) {
        return NULL;
    }
    
    return &it->second;
}

map<string, int> MainCanvas::objectCounts()
{
    map<string, int> counts;
    
    for (svlObject2dSequence::const_iterator it = _objects.begin(); it != _objects.end(); ++it) {
        for (svlObject2dFrame::const_iterator jt = it->second.begin(); jt != it->second.end(); ++jt) {
            if (counts.find(jt->name) == counts.end()) {
                counts[jt->name] = 1;
            } else {
                counts[jt->name] += 1;
            }
        }
    }

    return counts;
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
    file_menu->Append(FILE_NEW, _T("&New\tCtrl-N"), _T("Clear object labels"));
    file_menu->Append(FILE_OPEN, _T("&Open...\tCtrl-O"), _T("Open object label file"));
    file_menu->Append(FILE_OPEN_SEQUENCE, _T("Open &Image Sequence...\tCtrl-I"), _T("Open image sequence file"));
    file_menu->Append(FILE_OPEN_DIRECTORY, _T("Open &Directory...\tCtrl-D"), _T("Open directory of images"));
    file_menu->Append(FILE_OPEN_VIDEO, _T("Open &Video...\tCtrl-V"), _T("Open video file"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_SAVE, _T("&Save\tCtrl-S"), _T("Save object label file"));
    file_menu->Append(FILE_SAVEAS, _T("Save &As..."), _T("Save object label file"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_IMPORT, _T("Im&port..."), _T("Import another object label file"));
    file_menu->AppendSeparator();
    file_menu->Append(FILE_EXIT, _T("E&xit\tAlt-X"), _T("Exit this program"));
    edit_menu->Append(EDIT_FIND, _T("&Find...\tCtrl-F"), _T("Find objects"));
    edit_menu->AppendSeparator();
    edit_menu->Append(EDIT_GOTO_FRAME, _T("&Goto...\tCtrl-G"), _T("Goto frame number"));
    edit_menu->Append(EDIT_GOTO_EMPTY, _T("Goto E&mpty"), _T("Goto next unlabeled frame"));
    options_menu->AppendCheckItem(OPTIONS_UNDISTORT, _T("&Undistort"), _T("Undistort video using camera intrinsics"));
    options_menu->Append(OPTIONS_LOADCALIBRATION, _T("&Camera Calibration..."), _T("Load camera calibration coefficients"));
    options_menu->AppendSeparator();
    options_menu->AppendCheckItem(OPTIONS_GRID, _T("Show &Grid"), _T("Show or hide grid"));
    options_menu->AppendSeparator();
    options_menu->AppendCheckItem(OPTIONS_CENTROID_MODE, _T("Centr&oid mode"), _T("Mark centroid rather than bounding box"));
    options_menu->AppendSeparator();
    options_menu->Append(OPTIONS_DEFAULT_NAME, _T("Default &Object Name..."), _T("Set default name for inserted objects"));
    options_menu->Append(OPTIONS_DEFAULT_SIZE, _T("Default Object &Size..."), _T("Set default size for inserted objects (centroid mode)"));
    help_menu->Append(HELP_OBJECT_STATS, _T("&Statistics..."), _T("Show statistics on objects"));
    help_menu->AppendSeparator();
    help_menu->Append(HELP_KEYBOARD, _T("&Keyboard"), _T("Show keyboard shortcuts"));
    help_menu->Append(HELP_ABOUT, _T("&About...\tF1"), _T("Show about dialog"));

    wxMenuBar *menu_bar = new wxMenuBar();
    menu_bar->Append(file_menu, _T("&File"));
    menu_bar->Append(edit_menu, _T("&Edit"));
    menu_bar->Append(options_menu, _T("&Options"));
    menu_bar->Append(help_menu, _T("&Help"));
    SetMenuBar(menu_bar);

    options_menu->Check(OPTIONS_UNDISTORT, true);
    options_menu->Check(OPTIONS_GRID, false);
    options_menu->Check(OPTIONS_CENTROID_MODE, false);

    CreateStatusBar();

    // this is required for keyboard focus under Linux
    _canvas = new MainCanvas(this);
}

MainWindow::~MainWindow()
{
    // do nothing
}

void MainWindow::on_file_menu(wxCommandEvent& event)
{
    if (event.GetId() == FILE_NEW) {
        wxMessageDialog dlg(this, _T("Delete all objects?"),
            _T("New"), wxYES_NO | wxICON_QUESTION);
        if (dlg.ShowModal() == wxID_YES) {
            _canvas->clearObjects();
            _objectFilename = string("");
        }
    } else if (event.GetId() == FILE_OPEN) {
        wxFileDialog dlg(this, _T("Choose object list file to open"), _T(""), 
	    _T(""), _T("XML files (*.xml)|*.xml|Text files (*.txt)|*.txt|All files (*.*)|*.*"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
            _objectFilename = dlg.GetPath();
	    _canvas->loadObjects(_objectFilename.c_str());
        }
    } else if (event.GetId() == FILE_OPEN_SEQUENCE) {
        wxFileDialog dlg(this, _T("Choose image sequence to open"), _T(""), 
	    _T(""), _T("Sequence files (*.xml)|*.xml"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
            _canvas->openSequence(dlg.GetPath().c_str());
        }
    } else if (event.GetId() == FILE_OPEN_DIRECTORY) {
	wxDirDialog dlg(this, _T("Choose image directory"));
	if (dlg.ShowModal() == wxID_OK) {
	    _canvas->openDirectory(dlg.GetPath().c_str());
	}
    } else if (event.GetId() == FILE_OPEN_VIDEO) {
        wxFileDialog dlg(this, _T("Choose video file to open"), _T(""), 
	    _T(""), _T("Video files (*.avi)|*.avi"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
            _canvas->openVideo(dlg.GetPath().c_str());
        }
    } else if ((event.GetId() == FILE_SAVE) && (_objectFilename.size())) {
	_canvas->saveObjects(_objectFilename.c_str());
    } else if ((event.GetId() == FILE_SAVE) || (event.GetId() == FILE_SAVEAS)) {
        wxFileDialog dlg(this, _T("Choose object list file to save"), _T(""), 
	    _T(""), _T("XML files (*.xml)|*.xml"), wxSAVE | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
	    _objectFilename = dlg.GetPath().c_str();
	    _canvas->saveObjects(_objectFilename.c_str());
        }
    } else if (event.GetId() == FILE_IMPORT) {
        wxFileDialog dlg(this, _T("Choose object list file to import"), _T(""), 
			 _T(""), _T("XML files (*.xml)|*.xml"), wxOPEN | wxFD_CHANGE_DIR);
        if (dlg.ShowModal() == wxID_OK) {
	    _canvas->importObjects(dlg.GetPath().c_str());	    
        }       
    } else if (event.GetId() == FILE_EXIT) {
        Close(true);
    }

    Refresh(false);
    Update();
}

void MainWindow::on_edit_menu(wxCommandEvent& event)
{
    if (event.GetId() == EDIT_FIND) {
    	NOT_IMPLEMENTED_YET;
    } else if (event.GetId() == EDIT_GOTO_FRAME) {
        wxTextEntryDialog dlg(this, "Enter frame index:");
        if (dlg.ShowModal() == wxID_OK) {
            _canvas->gotoFrame(atoi(dlg.GetValue().c_str()));
        }	
    } else if (event.GetId() == EDIT_GOTO_EMPTY) {
	_canvas->gotoEmpty();
    }

    _canvas->findActiveObject();
    Refresh(false);
    Update();
}

void MainWindow::on_options_menu(wxCommandEvent& event)
{
    if (event.GetId() == OPTIONS_UNDISTORT) {
        _canvas->undistort(event.IsChecked());
    } else if (event.GetId() == OPTIONS_LOADCALIBRATION) {
        wxFileDialog dlg(this, _T("Choose camera calibration file"), _T(""), 
	        _T(""), _T("XML files (*.xml)|*.xml"), wxOPEN);
        if (dlg.ShowModal() == wxID_OK) {
	        _canvas->calibrate(dlg.GetPath().c_str());	    
        }       	
    } else if (event.GetId() == OPTIONS_GRID) {
        _canvas->drawGrid(event.IsChecked());
    } else if (event.GetId() == OPTIONS_CENTROID_MODE) {
        _canvas->centroidMode(event.IsChecked());
    } else if (event.GetId() == OPTIONS_DEFAULT_NAME) {
        wxTextEntryDialog dlg(this, "Enter default object name:");
        if (dlg.ShowModal() == wxID_OK) {
	        _canvas->setDefaultObjectName(dlg.GetValue().c_str());
        }	
    } else if (event.GetId() == OPTIONS_DEFAULT_SIZE) {
        wxTextEntryDialog dlg(this, (wxString &)_T("Enter default object size:"),
	    wxString("Set Size"), wxString(_canvas->getDefaultObjectSize().c_str()));
	    if (dlg.ShowModal() == wxID_OK) {
	        _canvas->setDefaultObjectSize(dlg.GetValue().c_str());
        }
    }

    _canvas->findActiveObject();
    Refresh(false);
    Update();
}

void MainWindow::on_help_menu(wxCommandEvent& event)
{
    if (event.GetId() == HELP_OBJECT_STATS) {
        map<string, int> counts = _canvas->objectCounts();
        string message;
        for (map<string, int>::const_iterator it = counts.begin(); it != counts.end(); ++it) {
            message = message + it->first + string(":\t") +
                toString(it->second) + string("\n");
        }
        if (counts.empty()) {
            message = message + string("No objects.");
        }
        wxMessageBox(wxString(message.c_str()), _T("Statistics"));
    } else if (event.GetId() == HELP_KEYBOARD) {
        NOT_IMPLEMENTED_YET;
    } else if (event.GetId() == HELP_ABOUT) {
        wxAboutDialogInfo info;
        info.SetName(_T("Image Object Labeler"));
        info.SetVersion(_T("0.4"));
        info.SetDescription(_T("This program allows you to view and label image sequences and video streams."));
        info.SetCopyright(_T("(C) 2007-2008 Staphen Gould <sgould@stanford.edu>"));

        wxAboutBox(info);
    }
}

void MainWindow::on_close(wxCloseEvent& event)
{
    // not implemented yet
    event.Skip();
}

// ImageLabelerApp Implementation --------------------------------------------

bool ImageLabelerApp::OnInit()
{
    // setup main window
    gMainWindow = new MainWindow(NULL, wxID_ANY, wxT("Image Object Labeler GUI"),
        wxDefaultPosition, wxSize(640, 480));
    SetTopWindow(gMainWindow);
    gMainWindow->Show();
    gMainWindow->SetFocus();    

    // call base class for command-line options
    wxApp::OnInit();

    return true;
}

void ImageLabelerApp::OnInitCmdLine(wxCmdLineParser& parser)
{
    parser.SetDesc(COMMAND_LINE_DESCRIPTION);
    if (parser.Parse(true)) {
	exit(1);
    }

    wxString str;
    if (parser.Found("c", &str)) {
        gMainWindow->_canvas->calibrate(str.c_str());
    }

    if (parser.Found("o", &str)) {
	gMainWindow->_objectFilename = string(str.c_str());
	gMainWindow->_canvas->loadObjects(str.c_str());
    }

    if (parser.Found("i", &str)) {
        gMainWindow->_canvas->openSequence(str.c_str());
    }
}

int ImageLabelerApp::OnExit()
{
    return 0;
}

IMPLEMENT_APP(ImageLabelerApp)
