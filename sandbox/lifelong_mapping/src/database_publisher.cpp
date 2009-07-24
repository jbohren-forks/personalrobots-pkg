#include "lifelong_mapping/database_publisher.h"

namespace lifelong_mapping {

DatabasePublisher::DatabasePublisher()
{
}

DatabasePublisher::DatabasePublisher(const DatabasePublisher& rhs)
{
  publisher_ = rhs.publisher_;
  factory_ = rhs.factory_;
}

uint32_t DatabasePublisher::getNumSubscribers() const
{
  return publisher_.getNumSubscribers();
}

std::string DatabasePublisher::getTopic() const
{
  return publisher_.getTopic();
}

void DatabasePublisher::publish(const ros::Message& message, uint32_t node_id) const
{
  publisher_.publish( factory_->create(message, node_id) );
}

void DatabasePublisher::publish(const ros::Message::ConstPtr& message, uint32_t node_id) const
{
  publish(*message, node_id);
}

void DatabasePublisher::shutdown()
{
  publisher_.shutdown();
}

} //namespace lifelong_mapping
