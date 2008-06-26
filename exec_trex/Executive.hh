#ifndef H_Executive
#define H_Executive

#include "Agent.hh"
#include "LogManager.hh"
#include "Debug.hh"
#include "ExecDefs.hh"
#include "Observer.hh"

#include <pthread.h>



namespace TREX{

  class Executive
  {
  public:

    static ExecutiveId instance();

    Executive();

    ~Executive();

    /**
     * test if executive is initialized with inbound messages it requires
     */
    bool isInitialized() const;

  private:

    static ExecutiveId s_id;
    ExecutiveId m_id;
    bool m_initialized;
    
  };

}

#endif
