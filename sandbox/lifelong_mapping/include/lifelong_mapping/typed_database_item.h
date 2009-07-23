#ifndef LIFELONG_MAPPING_DATABASE_ITEM_H
#define LIFELONG_MAPPING_DATABASE_ITEM_H

#include <ros/message.h>
#include <boost/type_traits/remove_cv.hpp>
#include <boost/type_traits/remove_reference.hpp>

#include "lifelong_mapping/DatabaseItem.h"

namespace lifelong_mapping {

// Wire-compatible with DatabaseItem
template <typename M>
class TypedDatabaseItem : public ros::Message
{
public:
  typedef typename boost::remove_reference<M>::type DataMessage;
  
  typedef boost::shared_ptr<TypedDatabaseItem> Ptr;
  typedef boost::shared_ptr<TypedDatabaseItem const> ConstPtr;

  uint32_t node_id;
  M data;

  TypedDatabaseItem()
    : ros::Message(),
      node_id(0)
  {
  }

  TypedDatabaseItem(uint32_t node_id, DataMessage& data)
    : ros::Message(),
      node_id(node_id),
      data(data)
  {
  }

  TypedDatabaseItem(const TypedDatabaseItem& copy)
    : ros::Message(),
      node_id(copy.node_id),
      data(copy.data)
  {
  }

  TypedDatabaseItem& operator=(const TypedDatabaseItem& copy)
  {
    if (this == &copy)
      return *this;
    node_id = copy.node_id;
    data = copy.data;
    return *this;
  }

  virtual ~TypedDatabaseItem()
  {
  }

  inline static std::string __s_getDataType()
  {
    return DatabaseItem::__s_getDataType();
    //return DataMessage::__s_getDataType() + "WithNodeId";
  }

  inline static std::string __s_getMD5Sum()
  {
    // @todo: what should really happen here?
    return DatabaseItem::__s_getMD5Sum();
    //return DataMessage::__s_getMD5Sum();
  }

  inline static std::string __s_getMessageDefinition()
  {
    return "uint32 node_id\n" + DataMessage::__s_getDataType() + " data\n";
  }

  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }

  inline uint32_t serializationLength() const
  {
    return 8 + data.serializationLength();
  }

  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    SROS_SERIALIZE_PRIMITIVE(write_ptr, node_id);
    uint32_t data_len = data.serializationLength();
    SROS_SERIALIZE_PRIMITIVE(write_ptr, data_len);
    return data.serialize(write_ptr, seq);
  }

  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, node_id);
    uint32_t data_len;
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, data_len);
    return data.deserialize(read_ptr);
  }
};


class DatabaseItemFactory
{
public:
  virtual ~DatabaseItemFactory() {}
  
  virtual ros::MessageConstPtr create(const ros::Message& data, uint32_t node_id) = 0;

  virtual std::string getDataType() = 0;
  virtual std::string getMD5Sum() = 0;
};

template <typename M>
class DatabaseItemFactoryT : public DatabaseItemFactory
{  
public:
  // NB: We wrap a reference to M to avoid potentially expensive copying.
  typedef TypedDatabaseItem<M&> ItemType;
  
  virtual ros::MessageConstPtr create(const ros::Message& data, uint32_t node_id)
  {
    // const-ness restored by MessageConstPtr return type, so this is "safe"
    M* data_ptr = const_cast<M*>(dynamic_cast<const M*>(&data));
    ros::MessageConstPtr item( new ItemType(node_id, *data_ptr) );
    return item;
  }

  virtual std::string getDataType()
  {
    return ItemType::__s_getDataType();
  }
  
  virtual std::string getMD5Sum()
  {
    return ItemType::__s_getMD5Sum();
  }
};

} //namespace lifelong_mapping

#endif
