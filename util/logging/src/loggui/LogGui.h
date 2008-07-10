///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 16 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __LogGui__
#define __LogGui__

#include <wx/treectrl.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class LogGui
///////////////////////////////////////////////////////////////////////////////
class LogGui : public wxFrame 
{
	private:
	
	protected:
		wxTreeCtrl* topicTree;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onClose( wxCloseEvent& event ){ event.Skip(); }
		
	
	public:
		LogGui( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		~LogGui();
	
};

#endif //__LogGui__
