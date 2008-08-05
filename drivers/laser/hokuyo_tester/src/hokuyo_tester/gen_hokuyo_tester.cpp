///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 16 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "gen_hokuyo_tester.h"

///////////////////////////////////////////////////////////////////////////

GenHokuyoTester::GenHokuyoTester( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	this->SetMinSize( wxSize( 640,480 ) );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );
	
	visPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer1->Add( visPanel, 1, wxALL|wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer1;
	sbSizer1 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Config") ), wxVERTICAL );
	
	sbSizer1->SetMinSize( wxSize( 300,-1 ) ); 
	wxBoxSizer* bSizer31;
	bSizer31 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("Port:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	bSizer31->Add( m_staticText2, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	port = new wxTextCtrl( this, wxID_ANY, wxT("/dev/ttyACM0"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer31->Add( port, 1, wxALL|wxEXPAND|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer1->Add( bSizer31, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	connectButton = new wxButton( this, wxID_ANY, wxT("Connect"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer3->Add( connectButton, 1, wxALL|wxEXPAND, 5 );
	
	disconnectButton = new wxButton( this, wxID_ANY, wxT("Disconnect"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer3->Add( disconnectButton, 1, wxALL|wxEXPAND, 5 );
	
	scanButton = new wxToggleButton( this, wxID_ANY, wxT("Scan"), wxDefaultPosition, wxSize( -1,-1 ), 0 );
	bSizer3->Add( scanButton, 1, wxALL|wxEXPAND, 5 );
	
	sbSizer1->Add( bSizer3, 0, wxEXPAND, 5 );
	
	logText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE );
	sbSizer1->Add( logText, 1, wxALL|wxEXPAND, 5 );
	
	bSizer1->Add( sbSizer1, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	bSizer1->Fit( this );
	
	// Connect Events
	connectButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnConnect ), NULL, this );
	disconnectButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnDisconnect ), NULL, this );
	scanButton->Connect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnScan ), NULL, this );
}

GenHokuyoTester::~GenHokuyoTester()
{
	// Disconnect Events
	connectButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnConnect ), NULL, this );
	disconnectButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnDisconnect ), NULL, this );
	scanButton->Disconnect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( GenHokuyoTester::OnScan ), NULL, this );
}
