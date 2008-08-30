#ifndef __TS_LOGGER_H__
#define __TS_LOGGER_H__

#include <boost/thread.hpp>
#include <iostream>
#include <pthread.h>

#define FLUSH_INTERVAL 10

/**
   @brief A thread-safe logging utility
 */
class ThreadSafeLogger {
public:
  ThreadSafeLogger(std::ostream* aostream) :
    ostream(aostream),
    counter(0)
  {
  }

  void log(string prefix, string stuff) {
    if (ostream != NULL) {
      int tid = pthread_self();

      boost::mutex::scoped_lock lock(writeMutex);
      (*ostream) << tid << " " << prefix << ": " << stuff << std::endl;
      //      (*ostream) << prefix << ": " << stuff << std::endl;

      if ((counter++ % 10) == 0)
	flush();
    }
  }

  void flush() {
    if (ostream != NULL) {
      ostream->flush();
    }
  }
  
private:
  std::ostream* ostream;
  boost::mutex writeMutex;
  int counter;
};

#endif
