#ifndef H_PLAYBACK
#define H_PLAYBACK

#include <string>
#include <vector>
#include <stdio.h>
#include "XMLUtils.hh"
#include "Id.hh"

using namespace std;
namespace TREX {
  class Playback;
  typedef EUROPA::Id<Playback> PlaybackId;

  class Playback
  {
  public:
    /**
     * Creates and instance of the singleton and adds a reference.
     */
    static PlaybackId request();
    /**
     * Releases a reference to the singleton.
     */
    static void release();
    /**
     * Loads all config elements at a given time attribute.
     */
    void getNodes(string name, unsigned int time, vector<EUROPA::TiXmlElement*> &nodes);
  private:
    Playback();
    ~Playback();
    /**
     * Adds a reference
     */
    void addRef();
    /**
     * Deletes a reference.
     */
    bool decRef();


    unsigned int m_refCount;
    static PlaybackId s_id; /*! Singleton Id*/
    PlaybackId m_id; /*! Object Id */
    EUROPA::TiXmlElement* m_logRoot; /*! XML Root */
    
  };

}

#endif
