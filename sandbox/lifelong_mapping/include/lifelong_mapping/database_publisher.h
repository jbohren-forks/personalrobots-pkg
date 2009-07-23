#ifndef LIFELONG_MAPPING_DATABASE_PUBLISHER_H
#define LIFELONG_MAPPING_DATABASE_PUBLISHER_H

#include <ros/publisher.h>

#include "lifelong_mapping/typed_database_item.h"

namespace lifelong_mapping {

class DatabasePublisher
{
public:
  DatabasePublisher();
  DatabasePublisher(const DatabasePublisher& rhs);

  uint32_t getNumSubscribers() const;

  std::string getTopic() const;

  void publish(const ros::Message& message, uint32_t node_id) const;
  void publish(const ros::Message::ConstPtr& message, uint32_t node_id) const;

  void shutdown();

private:
  ros::Publisher publisher_;
  boost::shared_ptr<DatabaseItemFactory> factory_;

  friend class GraphClient;
};

} //namespace lifelong_mapping

#endif
