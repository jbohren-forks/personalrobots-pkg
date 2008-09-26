//-------------------------------------------------------------------------------
//
//    cvx_plot.cpp
//    OpenCV based code generating a multi-panel image window.
//    Copyright (C) 2007 Adrian Kaehler, Gary Bradski
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//
//-------------------------------------------------------------------------------
#include "cvx_plot.h"

CvFont*	CvxPlot::M_FONT					= NULL;
bool	CvxPlot::M_CLASS_INITIALIZED	= false;

void CvxPlot::redraw( void ) {

	// Clear the screen
	//
	cvSet( m_image, m_background );

	// Draw the axes
	//
	{
		int x = x2i(0.0f);
		cvLine( m_image, cvPoint(x,0), cvPoint(x,m_image->height), m_axis_color );
		int y = y2i(0.0f);
		cvLine( m_image, cvPoint(0,y), cvPoint(m_image->width,y), m_axis_color );
	}

	// Draw all of the tiles
	//
	for( vector<CvxPlotTile>::iterator it=m_tiles.begin(); it!=m_tiles.end(); ++it ) {

		int ix1 = x2i( it->fx );
		int iy1 = y2i( it->fy );

		int ix2 = x2i( it->fx + it->dx );
		int iy2 = y2i( it->fy + it->dy );

		cvRectangle(
			m_image,
			cvPoint( ix1, iy1 ),
			cvPoint( ix2, iy2 ),
			it->color,
			CV_FILLED
		);

cvRectangle(
	m_image,
	cvPoint( ix1, iy1 ),
	cvPoint( ix2, iy2 ),
	CVX_RED,
	1
);

	}

	// Draw all of the points
	//
	for( vector<CvxPlotPoint>::iterator it=m_points.begin(); it!=m_points.end(); ++it ) {

		int ix = x2i( it->fx );
		int iy = y2i( it->fy );
		
		if( ix<0 || ix>m_image->width || iy<0 || iy>m_image->height ) continue;

		switch( it->type ) {
			case POINT:
				cvCircle( 
					m_image, 
					cvPoint(ix,iy), 
					it->size, 
					it->color 
				);
				break;
			case SQUARE:
				cvRectangle(
					m_image, 
					cvPoint( 
						(int)(ix-1.14f*it->size/2), 
						(int)(iy-1.14f*it->size/2)
					), 
					cvPoint( 
						(int)(ix+1.14f*it->size/2), 
						(int)(iy+1.14f*it->size/2)
					), 
					it->color,
					CV_FILLED
				);
				break;
			case CIRCLE:
				cvCircle( 
					m_image, 
					cvPoint(ix,iy), 
					it->size, 
					it->color, 
					CV_FILLED
				);
				break;
		}

	}

	// Add any descriptive label
	//
	if( m_label_text != NULL ) {
		cvPutText( m_image, m_label_text, cvPoint(5,25), M_FONT, m_label_color );
	}

	cvShowImage( m_szWindowName, m_image );

}


void CvxPlot::add_point(
	float fx,
	float fy,
	CvScalar color,
	EPointType type,
	int size
) {
	CvxPlotPoint p( fx, fy, color, type, size );
	m_points.push_back( p );
}

void CvxPlot::add_tile( 
	float fx,
	float fy,
	float dx,
	float dy,
	CvScalar color
) { 

	CvxPlotTile p( fx, fy, dx, dy, color );
	m_tiles.push_back( p );

}