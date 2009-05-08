#include "executive_trex_pr2/logger.hh"
#include "Debug.hh"
#include "Observer.hh"


namespace TREX {
  LoggerId Logger::s_id = LoggerId::noId();

  LoggerId Logger::request() {
    if(s_id == LoggerId::noId()){
      new Logger();
    }
    s_id->addRef();
    return s_id;
  }
  
 
  void Logger::release() {
    if (s_id->decRef()) {
      delete (Logger*)s_id;
      s_id = LoggerId::noId();
    }
  }
  

  void Logger::addRef() {
    m_refCount++;
  }

  bool Logger::decRef() {
    m_refCount--;
    return m_refCount == 0;
  }

  FILE* Logger::getFile() {
    return m_file;
  }

  
  void Logger::writeObservation(Observation* obs, unsigned int time, const char* name) {
    if (m_file && m_enabled) {
      fprintf(m_file, "<%s time=\"%u\">\n", name, time);
      obs->printXML(m_file);
      fprintf(m_file, "</%s>\n", name); 
    }
  }


  void Logger::setEnabled(bool val) {
    m_enabled = val;
    if (val && !m_file) {
      m_file = fopen("logfile.xml", "w");
      fprintf(m_file, "<log>\n");
    } else if (!val && m_file) {
      fprintf(m_file, "</log>\n");
      fclose(m_file);
      m_file = NULL;
    }
  }

  Logger::Logger() : m_id(this) {
    m_file = NULL;
    m_enabled = false;
    s_id = m_id;
    m_refCount = 0;
  }
  Logger::~Logger() {
    if (m_file) {
      fclose(m_file);
      m_file = NULL;
    }
    m_id.remove();
  }

}
