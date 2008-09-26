#pragma once

//-------------------------------------------------------------------------------
//
//    cvx_multiwin.h
//    OpenCV based code generating a multi-panel image window.
//    Copyright (C) 2007 Adrian Kaehler, Gary Bradski
//
//----------------------------------------------------------------------------
//
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Stanford University nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS AND CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//---------------------------------------------------------------------------- 
//   Author: Adrian Kaehler
//
//   Description:
//
// This is the CvxPlot class.  It is an extension on the IplImage class, and is
// generically used in similar ways.  In addition to the image it contains, it 
// also implements a number of methods which allow the image to be interpreted
// as a plot.
//
//   This code should be understood to be developmental, and may contain bugs.
// Users are invited to make any changes they see fit, and requested to
// make modified or debugged versions generally available to all interested
// parties.  Please maintain a change log in the headder.
//
// CHANGE LOG:
// 5/5/06	Gary Bradski added in setable image origin.
//
//----------------------------------------------------------------------------
#include <cv.h>
#include <highgui.h>
#include <vector>
#include "cvx_defs.h"

using namespace std;

class CvxPlotPoint;
class CvxPlotTile;

class CvxPlot {

public:

	typedef enum {
		POINT,
		SQUARE,
		CIRCLE
	} EPointType;

private:

	vector<CvxPlotPoint>	m_points;
	vector<CvxPlotTile>		m_tiles;

	char		m_szWindowName[256];

	IplImage*	m_image;
	float		m_xmin;
	float		m_xmax;
	float		m_ymin;
	float		m_ymax;
	bool		m_initialized;
	CvScalar	m_background;

	char*		m_label_text;
	CvScalar	m_label_color;

	CvScalar	m_axis_color;

	static bool		M_CLASS_INITIALIZED;
	static CvFont*	M_FONT;

public:

	CvxPlot() {
		m_image = NULL;
		m_initialized = false;
	}
	
	CvxPlot(
		char* name,
		int width,
		int height,
		float xmin,
		float xmax,
		float ymin,
		float ymax,
		CvScalar background = CVX_WHITE
	) {
		m_image = NULL;
		m_initialized = NULL;
		initialize( name, width, height, xmin, xmax, ymin, ymax, background );
	}

	~CvxPlot() {
		if( m_image!=NULL ) {
			cvReleaseImage( &m_image );
		}
		if( m_initialized ) {
			cvDestroyWindow( m_szWindowName );
		}
	}

	void initialize(
		char* name,
		int width,
		int height,
		float xmin,
		float xmax,
		float ymin,
		float ymax,
		CvScalar background = CVX_WHITE
	) {
		if( m_initialized ) return;

#ifndef WIN32
		strncpy( m_szWindowName, name, 256 );
#else
		strncpy_s( m_szWindowName, 256, name, 256 );
#endif

		cvNamedWindow( m_szWindowName, CV_WINDOW_AUTOSIZE );

		// A Font object exists which the CvxMultiWin class uses to 
		// write labels (when they are present) on the subwindows.
		//  This Font is a static member of the class, so is only
		// initialized when the class has never been initialized.
		// (This will happen on the first call to CvxMultiWin::initialize()
		// obviously.)
		//
		if( ! M_CLASS_INITIALIZED ) {
			M_FONT = new CvFont;
			assert( M_FONT );
			cvInitFont( M_FONT, CV_FONT_VECTOR0, 1.0f, 1.0f, 0.0, 2 );
			M_CLASS_INITIALIZED = true;
		}

		m_image			= cvCreateImage( cvSize(width,height), IPL_DEPTH_8U, 3 );
		m_xmin			= xmin;
		m_xmax			= xmax;
		m_ymin			= ymin;
		m_ymax			= ymax;
		m_background	= background;

		m_label_text	= NULL;
		m_label_color	= CVX_CYAN;

		m_axis_color	= CVX_BLACK;
		if( background.val[0]==0.0 && background.val[1]==0.0 && background.val[2]==0.0 ){
			m_axis_color = CVX_WHITE;
		}

		clear();
	}

	void clear( CvScalar background = cvScalar(-1.0) ) {
		if( background.val[0] != -1.0 ) {
			m_background=background;
		}
		cvSet( m_image, m_background );
		m_points.clear();
		m_tiles.clear();
	}

	void add_point(
		float fx,
		float fy,
		CvScalar color=CVX_BLACK,
		EPointType type=POINT,
		int size=6
	);

	void add_tile( 
		float fx,
		float fy,
		float dx,
		float dy,
		CvScalar color = CVX_BLACK
	);

	//bool add_point(
	//	CvPoint2D32f p,
	//	CvScalar color=CVX_BLACK,
	//	EPointType type=POINT,
	//	int size=6
	//) {
	//	add_point( p.x, p.y, color, type, size );
	//}

	float x2f( int ix ) { return( (float)((m_xmax-m_xmin)*ix/(float)m_image->width + m_xmin )); }
	float y2f( int iy ) { return( (float)((m_ymax-m_ymin)*(m_image->height-iy)/(float)m_image->height + m_ymin )); }
	CvPoint2D32f p2f( CvPoint p ) { return cvPoint2D32f( x2f(p.x), y2f(p.y) ); }

	int x2i( float fx ) { return( (int)((fx-m_xmin) / (m_xmax-m_xmin) * (float)m_image->width) ); }
	int y2i( float fy ) { return( m_image->height - (int)((fy-m_ymin) / (m_ymax-m_ymin) * (float)m_image->height) ); }
	CvPoint p2i( CvPoint2D32f p ) { return cvPoint( x2i(p.x), y2i(p.y) ); }

	void label( char* text ) {

		size_t len = strlen( text );
		if( m_label_text==NULL ) {
			m_label_text = (char*) new char[len+1];
		} else {
			if( len>strlen(m_label_text) ) {
				delete[] m_label_text;
				m_label_text = (char*) new char[len+1];
			}
		}
#ifndef WIN32
		strcpy( m_label_text, text );
#else
		strcpy_s( m_label_text, 256, text );
#endif
	}

	// This just draws the plot to the screen in it's current state
	//
	void draw( void ) {
		cvShowImage( m_szWindowName, m_image );
	}

	// This actually updates the plot image to include the "latest"
	// changes to the data or any other parameters.
	//
	// Don't use this inside of cvWaitKey() type idle loops!
	//
	void redraw( void );

};


class CvxPlotPoint {

public:

	float				fx;
	float				fy;
	CvScalar			color;
	CvxPlot::EPointType	type;
	int					size;

	CvxPlotPoint( float x, float y, CvScalar c, CvxPlot::EPointType t, int s ) {
		fx		= x;
		fy		= y;
		color	= c;
		type	= t;
		size	= s;
	}

};

class CvxPlotTile {

public:

	float		fx;
	float		fy;
	float		dx;
	float		dy;
	CvScalar	color;

	CvxPlotTile( float x, float y, float w, float h, CvScalar c ) {
		fx		= x;
		fy		= y;
		dx		= w;
		dy		= h;
		color	= c;
	}

};