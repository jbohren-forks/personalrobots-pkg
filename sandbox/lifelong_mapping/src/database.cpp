#include "lifelong_mapping/database.h"

#include <boost/scoped_array.hpp>

namespace lifelong_mapping {

// Little-endian integers do not sort well as byte strings
int compare_int(Db *dbp, const Dbt *a, const Dbt *b)
{
  uint32_t ai, bi;
  // Copy keys into appropriately aligned memory
  memcpy(&ai, a->get_data(), sizeof(uint32_t));
  memcpy(&bi, b->get_data(), sizeof(uint32_t));
  return ai - bi;
}

int get_topic(Db *sdbp, const Dbt *pkey, const Dbt *pdata, Dbt *skey)
{
  uint8_t *read_ptr = (uint8_t*)pdata->get_data() + sizeof(uint32_t);
  uint32_t topic_len;
  SROS_DESERIALIZE_PRIMITIVE(read_ptr, topic_len);
  skey->set_data(read_ptr);
  skey->set_size(topic_len);

  return 0;
}

int get_node_id(Db *sdbp, const Dbt *pkey, const Dbt *pdata, Dbt *skey)
{
  skey->set_data(pdata->get_data());
  skey->set_size(sizeof(uint32_t));
  return 0;
}

Database::Database(const std::string& path)
  : env_(0)
{
  static const u_int32_t ENV_OPEN_FLAGS =
    DB_CREATE     | // If the env does not exist, create it
    DB_INIT_LOCK  | // Initialize locking
    DB_INIT_LOG   | // Initialize logging
    DB_INIT_MPOOL | // Initialize the cache
    DB_INIT_TXN;    // Initialize transactions

  // FIXME: revisit transaction durability question
  static const u_int32_t ENV_FLAGS =
    DB_AUTO_COMMIT | // Use auto commit on single operations
    DB_TXN_NOSYNC;   // Do not force log data to disk on transaction commit

  // FIXME: use DB_READ_[UN]COMMITTED?
  static const u_int32_t DB_FLAGS = DB_CREATE | DB_AUTO_COMMIT;
  static const char DB_FILE_NAME[] = "data.db";

  static const u_int32_t SEQ_FLAGS = DB_CREATE | DB_THREAD;

  // Open environment
  env_.open(path.c_str(), ENV_OPEN_FLAGS, 0); //throws
  env_.set_flags(ENV_FLAGS, 1);

  // Open primary database
  db_.reset(new Db(&env_, 0));
  db_->set_bt_compare(compare_int);
  db_->open(NULL,         // Transaction pointer
            DB_FILE_NAME, // File name
            "messages",   // Logical db name
            DB_BTREE,     // Database type
            DB_FLAGS,     // Open flags
            0);           // File mode. Using defaults

  // Open sequence object (for enumerating IDs persistently)
  sequence_db_.reset(new Db(&env_, 0));
  sequence_db_->open(NULL, DB_FILE_NAME, "sequence", DB_BTREE, DB_FLAGS, 0);
  sequence_.reset(new DbSequence(sequence_db_.get(), 0));
  sequence_->initial_value(1);
  uint32_t zero = 0;
  Dbt seq_key(&zero, sizeof(uint32_t));
  sequence_->open(NULL, &seq_key, SEQ_FLAGS);

  // FIXME: Use immutable keys?
  // Open topic secondary index
  topic_index_.reset(new Db(&env_, 0));
  topic_index_->set_flags(DB_DUPSORT);
  topic_index_->set_dup_compare(compare_int);
  topic_index_->open(NULL, DB_FILE_NAME, "topic_index", DB_BTREE, DB_FLAGS, 0);
  db_->associate(NULL, topic_index_.get(), get_topic, DB_AUTO_COMMIT);
  
  // Open node ID secondary index
  node_index_.reset(new Db(&env_, 0));
  node_index_->set_bt_compare(compare_int);
  node_index_->set_flags(DB_DUPSORT);
  node_index_->set_dup_compare(compare_int);
  node_index_->open(NULL, DB_FILE_NAME, "node_index", DB_BTREE, DB_FLAGS, 0);
  db_->associate(NULL, node_index_.get(), get_node_id, DB_AUTO_COMMIT);

  // FIXME: Open timestamp secondary index
}

Database::~Database()
{
  // FIXME: make sure no transactions are open
  // these all throw
  node_index_->close(0);
  topic_index_->close(0);
  sequence_->close(0);
  sequence_db_->close(0);
  db_->close(0);
  env_.close(0);
}

uint32_t Database::insert(const ros::Message& msg, const std::string& topic, uint32_t node_id)
{
  uint32_t item_id = nextId();
  insert(msg, topic, node_id, item_id);
  return item_id;
}

void Database::insert(const ros::Message& msg, const std::string& topic, uint32_t node_id, uint32_t item_id)
{
  // Serialize message
  uint32_t topic_len = topic.length();
  uint32_t length = sizeof(uint32_t) + (4 + topic_len) + msg.serializationLength();
  boost::scoped_array<uint8_t> buffer( new uint8_t[length] );
  uint8_t *write_ptr = buffer.get();
  SROS_SERIALIZE_PRIMITIVE(write_ptr, node_id);
  SROS_SERIALIZE_PRIMITIVE(write_ptr, topic_len);
  SROS_SERIALIZE_BUFFER(write_ptr, topic.c_str(), topic_len);
  msg.serialize(write_ptr, 0); // FIXME: what is 2nd param?

  // Put in database
  Dbt key(&item_id, sizeof(uint32_t));
  Dbt data(buffer.get(), length);
  db_->put(NULL, &key, &data, 0); // uses auto commit
}

bool Database::get(ros::Message& msg, uint32_t item_id, std::string& topic, uint32_t& node_id)
{
  // FIXME: type-safety, maybe check data type and MD5 sum?

  Dbt key(&item_id, sizeof(uint32_t)), data;

  DbTxn *txn = NULL;
  env_.txn_begin(NULL, &txn, 0);
  int ret = db_->get(txn, &key, &data, 0); //throws
  if (ret != 0)
    return false;

  uint8_t *read_ptr = (uint8_t*)data.get_data();
  SROS_DESERIALIZE_PRIMITIVE(read_ptr, node_id);
  uint32_t topic_len;
  SROS_DESERIALIZE_PRIMITIVE(read_ptr, topic_len);
  topic.assign((char*)read_ptr, topic_len);
  read_ptr += topic_len;
  msg.deserialize(read_ptr);

  txn->commit(0);
  return true;
}

uint32_t Database::nextId()
{
  db_seq_t next_id;
  sequence_->get(NULL, 1, &next_id, 0);
  return next_id;
}

} //namespace lifelong_mapping
