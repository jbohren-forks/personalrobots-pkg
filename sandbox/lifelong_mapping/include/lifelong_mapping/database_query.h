#ifndef LIFELONG_MAPPING_DATABASE_QUERY_H
#define LIFELONG_MAPPING_DATABASE_QUERY_H

#include <string>
#include <vector>

namespace lifelong_mapping {

// FIXME: allow multiple topics in one query

struct DbQueryRaw
{
  // Inputs
  std::string topic;
  std::string data_type;
  std::string md5;
  std::vector<uint32_t> node_ids;

  // Outputs
  struct Record
  {
    uint32_t length;
    uint8_t* data;

    Record(uint8_t* data, uint32_t length)
      : length(length), data(data)
    {}
  };
  std::vector<Record> records;

  template <typename M> void setMessageType()
  {
    data_type = M::__s_getDataType();
    md5 = M::__s_getMD5Sum();
  }
};

template <typename M>
struct DbQuery
{
  // Inputs
  std::string topic;
  std::vector<uint32_t> node_ids;

  // Outputs
  std::vector<typename M::Ptr> records;

  DbQueryRaw asRawQuery();
  void deserializeRecords(const DbQueryRaw& raw);
};

template <typename M> DbQueryRaw DbQuery<M>::asRawQuery()
{
  DbQueryRaw raw_query;
  raw_query.topic = topic;
  raw_query.setMessageType<M>();
  raw_query.node_ids = node_ids;
  return raw_query;
}

template <typename M> void DbQuery<M>::deserializeRecords(const DbQueryRaw& raw)
{
  records.resize(raw.records.size());
  for (size_t i = 0; i < records.size(); ++i) {
    records[i].reset( new M );
    records[i]->deserialize(raw.records[i].data);
  }
}

} //namespace lifelong_mapping

#endif
