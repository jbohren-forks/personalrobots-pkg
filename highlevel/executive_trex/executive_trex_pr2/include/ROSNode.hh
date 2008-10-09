#ifndef H_ROSNode
#define H_ROSNode

// roscpp
#include <ros/node.h>

#include "Observer.hh"



namespace TREX{
  class ROSNode;
  typedef EUROPA::Id<ROSNode> ROSNodeId;


  class ROSNode : public ros::node
  {
  public:

    /**
     * Creates and instance of the singleton and adds a reference.
     */
    static ROSNodeId request();

    /**
     * Releases a reference to the singleton.
     */
    static void release();    

    ROSNode();

    ~ROSNode();

    template<class T> 
    void registerPublisher(const std::string &topic, size_t max_queue){
      ros::node::advertise<T>(topic, max_queue);
    }

    template<class M, class T>
    void registerSubscriber(const std::string &_topic, M &_msg, void (T::*fp)(), T* obj, int max_queue){
      ros::node::subscribe(_topic, _msg, fp, obj, max_queue);
    }
    
    template<class M>
    void publishMsg(const std::string &_topic, M &msg){
      ros::node::publish<M>(_topic, msg);
    }
    
  private:
    
    /**
     * Adds a reference
     */
    void addRef();
    /**
     * Deletes a reference.
     */
    bool decRef();

    static ROSNodeId s_id;
    ROSNodeId m_id;
    unsigned int m_refCount;
  };
}


#endif
