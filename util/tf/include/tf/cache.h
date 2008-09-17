#ifndef TF_CACHE_H
#define TF_CACHE_H

#include <list>
#include "rosthread/mutex.h"
#include "tf/data.h"
#include "tf/exceptions.h"

#include "LinearMath/btTransform.h"

#include <sstream>

namespace tf
{
  
/** \brief Storage for transforms and their parent */
class  TransformStorage : public Stamped<btTransform>
{
public:
  TransformStorage(){};
  TransformStorage(const Stamped<btTransform>& data, unsigned int parent_id): Stamped<btTransform>(data), parent_frame_id(parent_id){}; 
  unsigned int parent_frame_id;
};


/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. */
class TimeCache
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const unsigned long long DEFAULT_MAX_STORAGE_TIME = 10ULL * 1000000000ULL; //!< default value of 10 seconds storage
  static const unsigned long long DEFAULT_MAX_EXTRAPOLATION_TIME = 10000000000ULL; //!< default max extrapolation of 10 seconds
  

  TimeCache(bool interpolating = true, uint64_t max_storage_time = DEFAULT_MAX_STORAGE_TIME, 
            uint64_t max_extrapolation_time = DEFAULT_MAX_EXTRAPOLATION_TIME):
    interpolating_(interpolating),
    max_storage_time_(max_storage_time),
    max_extrapolation_time_(max_extrapolation_time)
    {};
  

  int64_t getData(uint64_t time, TransformStorage & data_out); //returns distance in time to nearest value

  void insertData(const TransformStorage& new_data)
    {
      storage_lock_.lock();
      std::list<TransformStorage >::iterator it = storage_.begin();
      while(it != storage_.end())
      {
        if (it->stamp_ <= new_data.stamp_)
          break;
        it++;
      }
      storage_.insert(it, new_data);
      storage_lock_.unlock();
      
      pruneList();
    };


  void interpolate(const TransformStorage& one, const TransformStorage& two, uint64_t time, TransformStorage& output);  //specific implementation for each T


  void clearList() { storage_lock_.lock(); storage_.clear(); storage_lock_.unlock();};

 private:
  std::list<TransformStorage > storage_; 

  bool interpolating_;
  uint64_t max_storage_time_;
  uint64_t max_extrapolation_time_;
  
  ros::thread::mutex storage_lock_;

  /// A helper function for getData 
  //Assumes storage is already locked for it
  uint8_t findClosest(TransformStorage& one, TransformStorage& two, uint64_t target_time, int64_t &time_diff);

  void pruneList()
    {

      storage_lock_.lock();
      uint64_t latest_time = storage_.begin()->stamp_;
      
      while(!storage_.empty() && storage_.back().stamp_ + max_storage_time_ < latest_time)
      {
        storage_.pop_back();
      }

      storage_lock_.unlock();
    };

  

};


}
#endif // TF_CACHE_H
