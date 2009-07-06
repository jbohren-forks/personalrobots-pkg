#include "lifelong_mapping/database.h"

#include <boost/scoped_array.hpp>

namespace lifelong_mapping {

int compare_int(Db *dbp, const Dbt *a, const Dbt *b)
{
  int ai, bi;
  // Copy keys into appropriately aligned memory
  memcpy(&ai, a->get_data(), sizeof(int));
  memcpy(&bi, b->get_data(), sizeof(int));
  return ai - bi;
}

Database::Database(const std::string& path)
  : env_(0)
{
  static const u_int32_t ENV_FLAGS =
    DB_CREATE    | // If the env does not exist, create it
    DB_INIT_CDB  | // Use concurrent data store
    DB_INIT_MPOOL; // Initialize the cache
  
  static const u_int32_t DB_FLAGS = DB_CREATE;
  static const char DB_FILE_NAME[] = "data.db";

  env_.open(path.c_str(), ENV_FLAGS, 0); //throws

  db_.reset(new Db(&env_, 0));
  db_->set_flags(DB_DUP); // Support duplicate (same key) records
  db_->set_bt_compare(compare_int);
  db_->open(NULL,         // Transaction pointer
            DB_FILE_NAME, // File name
            NULL,         // Logical db name
            DB_BTREE,     // Database type
            DB_FLAGS,     // Open flags
            0);           // File mode. Using defaults
}

Database::~Database()
{
  db_->close(0); //throws
  env_.close(0); //throws
}

void Database::insert(int node_id, const ros::Message& msg)
{
  // Serialize message
  uint32_t length = msg.serializationLength();
  boost::scoped_array<uint8_t> buffer( new uint8_t[length] );
  msg.serialize(buffer.get(), 0); // TODO: what is 2nd param?

  // Put in database
  Dbt key(&node_id, sizeof(int));
  Dbt data(buffer.get(), length);
  db_->put(NULL, &key, &data, 0);
}

bool Database::get(int node_id, ros::Message& msg)
{
  // TODO: type-safety, maybe check MD5 sum

  Dbt key(&node_id, sizeof(int)), data;
  int ret = db_->get(NULL, &key, &data, 0);
  if (ret != 0)
    return false;
  
  msg.deserialize((uint8_t*)data.get_data());
  return true;
}

} //namespace lifelong_mapping
