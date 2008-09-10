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
  

  void clearCache(void);
  
  int64_t getData(uint64_t time, Stamped<btTransform> & data_out); //returns distance in time to nearest value

  void insertData(const Stamped<btTransform>& new_data)
    {
      storage_lock_.lock();
      std::list<Stamped<btTransform> >::iterator it = storage_.begin();
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


  /** \todo implement this */  
  void interpolate(const Stamped<btTransform>& one, const Stamped<btTransform>& two, uint64_t time, Stamped<btTransform>& output);  //specific implementation for each T

 private:
  std::list<Stamped<btTransform> > storage_; 

  bool interpolating_;
  uint64_t max_storage_time_;
  uint64_t max_extrapolation_time_;
  
  ros::thread::mutex storage_lock_;

  /// A helper function for getData 
  //Assumes storage is already locked for it
  uint8_t findClosest(Stamped<btTransform>& one, Stamped<btTransform>& two, uint64_t target_time, int64_t &time_diff);

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

  
  void clearList() { storage_lock_.lock(); storage_.clear(); storage_lock_.unlock();};

};

  int64_t TimeCache::getData(uint64_t time, Stamped<btTransform> & data_out) //returns distance in time to nearest value
  //bool Pose3DCache::getValue(Pose3DStorage& buff, unsigned long long time, long long  &time_diff)
  {
    btTransform a,b;
    Stamped<btTransform> p_temp_1(a,0,"UNUSED");
    Stamped<btTransform> p_temp_2(b,0,"UNUSED");

    int num_nodes;
    int64_t time_diff;
    storage_lock_.lock();
    try
    {
      num_nodes = findClosest(p_temp_1,p_temp_2, time, time_diff);
      if (num_nodes == 0)
      {
        throw LookupException("No Data for this link");
      }
      else if (num_nodes == 1)
      {
        data_out = p_temp_1;
      }
      else
      {
        if(interpolating_)
          interpolate(p_temp_1, p_temp_2, time, data_out);
        else
          data_out = p_temp_1;
        
        
      }
    }
    //  catch (TFException &ex)
    catch (...)
    {
      storage_lock_.unlock();
      throw;
    }
    storage_lock_.unlock();
    
    return time_diff;

  };


uint8_t TimeCache::findClosest(Stamped<btTransform>& one, Stamped<btTransform>& two, uint64_t target_time, int64_t &time_diff)
{
  //No values stored
  if (storage_.empty())
  {
    return 0;
  }

  //If time == 0 return the latest
  if (target_time == 0)
  {
    one = storage_.front();
    time_diff = storage_.front().stamp_; ///@todo what should this be?? difference from "now"?
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end())
  {
    one = *(storage_.begin());
    time_diff = target_time - storage_.begin()->stamp_;
    return 1;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  std::list<Stamped<btTransform> >::iterator it = storage_.begin();
  while(it != storage_.end())
  {
    if (it->stamp_ <= target_time)
      break;
    it++;
  }
  //Catch the case it is the first value in the list
  if (it == storage_.begin())
  {
    one = *it;
    two = *(++it);
    time_diff = target_time - storage_.begin()->stamp_;
    if ((unsigned int) time_diff > max_extrapolation_time_) //Guarenteed in the future therefore positive
    {
      std::stringstream ss;
      ss << "Extrapolation Too Far in the future: target_time = "<< (target_time)/1000000000.0 <<", closest data at "
         << (one.stamp_)/1000000000.0 << " and " << (two.stamp_)/1000000000.0 <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_)/1000000000.0 <<" at "<< (target_time - one.stamp_)/1000000000.0<< " and " << (target_time - two.stamp_)/1000000000.0 <<" respectively.";
      throw ExtrapolationException(ss.str());
    }
    return 2;
  }

  //Catch the case where it's in the past
  if (it == storage_.end())
  {
    one = *(--it);
    two = *(--it);
    time_diff = target_time - one.stamp_;
    if (time_diff < -(long long)max_extrapolation_time_) //Guarenteed in the past
    {
      std::stringstream ss;
      ss << "Extrapolation Too Far in the past: target_time = "<< (target_time)/1000000000.0 <<", closest data at "
         << (one.stamp_)/1000000000.0 << " and " << (two.stamp_)/1000000000.0 <<" which are farther away than max_extrapolation_time "
         << (max_extrapolation_time_)/1000000000.0 <<" at "<< (-target_time + one.stamp_)/1000000000.0<< " and " << (-target_time + two.stamp_)/1000000000.0 <<" respectively."; //sign flip since in the past
      throw ExtrapolationException(ss.str());
    }
    return 2;
  }

  //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  one = *(it); //Older
  two = *(--it); //Newer
  if (fabs(target_time - one.stamp_) < fabs(target_time - two.stamp_))
    time_diff = target_time - one.stamp_;
  else
    time_diff = target_time - two.stamp_;
  return 2;


};

void TimeCache::interpolate(const Stamped<btTransform>& one, const Stamped<btTransform>& two, uint64_t time, Stamped<btTransform>& output)
{ 
  //Calculate the ratio
  btScalar ratio = ((double) ((int64_t)time - (int64_t)one.stamp_)) / ((double) ((int64_t)two.stamp_ - (int64_t)one.stamp_));
  
  //Interpolate translation
  btVector3 v;
  v.setInterpolate3(one.data_.getOrigin(), two.data_.getOrigin(), ratio);
  output.data_.setOrigin(v);
  
  //Interpolate rotation
  btQuaternion q1,q2;
  one.data_.getBasis().getRotation(q1);
  two.data_.getBasis().getRotation(q2);
  output.data_.setRotation(slerp( q1, q2 , ratio));
};



}
#endif // TF_CACHE_H
