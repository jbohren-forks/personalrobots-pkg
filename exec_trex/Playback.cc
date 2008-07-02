
#include "Playback.hh"
#include "Debug.hh"


namespace TREX {
  PlaybackId Playback::s_id = PlaybackId::noId();

  PlaybackId Playback::request() {
    if(s_id == PlaybackId::noId()){
      new Playback();
    }
    s_id->addRef();
    return s_id;
  }
  
 
  void Playback::release() {
    if (s_id->decRef()) {
      delete (Playback*)s_id;
      s_id = PlaybackId::noId();
    }
  }

  void Playback::addRef() {
    m_refCount++;
  }

  bool Playback::decRef() {
    m_refCount--;
    return m_refCount == 0;
  }

 
  void Playback::getNodes(string name, unsigned int time, vector<EUROPA::TiXmlElement*> &nodes) {
    for (EUROPA::TiXmlElement* child = m_logRoot->FirstChildElement(); child; child = child->NextSiblingElement()) {
      if (child->Value() == name) {
	int timeCompare = 0;
	if (child->Attribute("time", &timeCompare)) {
	  if (timeCompare == (int)time) {
	    nodes.push_back(child);
	  }
	}
      }
    }
  }

  
  Playback::Playback() : m_id(this) {
    m_logRoot = EUROPA::initXml("logfile.xml");
    s_id = m_id;
  }
  Playback::~Playback() {
    delete m_logRoot;
    m_id.remove();
  }

}




