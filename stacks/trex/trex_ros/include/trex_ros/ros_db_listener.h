#ifndef _ROS_DB_LISTENER_H
#define _ROS_DB_LISTENER_H

/** ros_db_listener.h
 * ROSDbListener is a listener used to accumulate changes to a EUROPA
 * PlanDatabase and then broadcast the changes over ROS. It uses the ROS
 * parameter server to configure what kind and amount of data to broadcast.
 *
 * The different types of information available are:
 *
 * Note that this class will only publish differences in order to save bandwidth
 * and server-side CPU time.
 */

#include <ros/ros.h>

#include "PlanDatabaseListener.hh"

namespace trex_ros {
  class ROSDbListener : public EUROPA::PlanDatabaseListener {
    public:
      ROSDbListener();
      ~ROSDbListener();

      // Token Operations
      
      // @brief Handle creation - scope specification
      void notifyAdded(const EUROPA::ObjectId& object, const EUROPA::TokenId& token);

      // @brief Handle merge
      void notifyMerged(const EUROPA::TokenId& token);

      // @brief Handle activation
      void notifyActivated(const EUROPA::TokenId& token);

      // @brief handle deletion - scope buffer removal
      void notifyRemoved(const EUROPA::TokenId& token);

      // @brief Handle notification of commitment
      void notifyCommitted(const EUROPA::TokenId& token);

      // @brief Handle notification of cancel
      void notifyDeactivated(const EUROPA::TokenId& token);

      // @brief Handle notification of cancel
      void notifySplit(const EUROPA::TokenId& token);

      // @brief For logging purposes, notify when rejecting a token
      void notifyRejected(const EUROPA::TokenId& token);

      // @brief Handle notification of termination
      void notifyTerminated(const EUROPA::TokenId& token);
      
    private:
      
  };
}

#endif
