#ifndef H_Logger
#define H_Logger

#include "Observer.hh"
#include "LogManager.hh"
#include <stdio.h>
#include "Id.hh"

namespace TREX {
  class Logger;
  class Observation;
  typedef EUROPA::Id<Logger> LoggerId;

  class Logger
  {
  public:
    /**
     * Creates and instance of the singleton and adds a reference.
     */
    static LoggerId request();
    /**
     * Releases a reference to the singleton.
     */
    static void release();
    /**
     * Get the log file.
     */
    FILE* getFile();
    /**
     * Sets if the logger is enabled.
     */
    void setEnabled(bool val);
    /**
     * Writes observations to the log file.
     */
    void writeObservation(Observation* obs, unsigned int time, const char* name);
  private:
    Logger();
    ~Logger();  
    /**
     * Adds a reference
     */
    void addRef();
    /**
     * Deletes a reference.
     */
    bool decRef();

    unsigned int m_refCount;
    static LoggerId s_id; /*! Singleton Id*/
    LoggerId m_id; /*! Object Id */
    FILE* m_file; /*! The file pointer. */
    bool m_enabled; /*! Enabled. */
  };

}

#endif
