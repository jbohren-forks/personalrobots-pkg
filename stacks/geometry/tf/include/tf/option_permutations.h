/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/** \author Tully Foote */

#ifndef TF_OPTIONPERMUTATIONS_H
#define TF_OPTIONPERMUTATIONS_H

namespace tf
{
/** \brief A base class for storing pointers to generic data types
 */
class OptionBase
{
public:
  virtual void reset() =0;
  virtual bool step() =0;
  virtual ~OptionBase() {};
};


/**\brief A class to hold a set of option values and currently used state
 *  This class holds 
 */
template<class T>
class Option : public OptionBase
{
public:
  Option(const std::vector<T>& options, T* output)
  {
    options_ = options;
    output_ = output;
    reset();
  }
  
  virtual ~Option(){};

  void reset(){
    current_element_ = options_.begin();
    *output_ = *current_element_;
  };
  
  bool step()
  {
    current_element_++;
    if (current_element_ == options_.end())
      return false;
    *output_ = *current_element_;
    return true;
  };

private:
  /// Local storage of the possible values
  std::vector<T> options_;
  /// The output variable
  T* output_;
  typedef typename std::vector<T>::iterator vecTit;
  /// The last updated element
  vecTit current_element_;

};

/** \brief A class to provide easy permutation of options
 * This class provides a way to collapse independent 
 * permutations of options into a single loop.  
 */
class Permuter
{
public:
  /** \brief Destructor to clean up allocated data */
  virtual ~Permuter(){ clearAll();};


  /** \brief Add a set of values and an output to the iteration
   * @param values The set of possible values for this output
   * @param output The value to set at each iteration
   */
  template<class T>
  void addOption(const std::vector<T>& values, T* output)
  {
    options_.push_back(static_cast<OptionBase*> (new Option<T>(values, output)));
    reset();
  };


  /** \brief Reset the internal counters */
  void reset(){
    for (unsigned int level= 0; level < options_.size(); level++)
      options_[level]->reset();
  };

  /** \brief Iterate to the next value in the iteration
   * Returns true unless done iterating.  
   */
  bool step()
  {
    // base case just iterating
    for (unsigned int level= 0; level < options_.size(); level++)
    {
      if(options_[level]->step())
      {
        //printf("stepping level %d returning true \n", level);
        return true;
      }
      else
      {
        //printf("reseting level %d\n", level);
        options_[level]->reset();
      }
    }
    return false;
  };

  //\todo add mutex to be thread safe
  /** \brief Clear all stored data */
  void clearAll()
  {
    for ( unsigned int i = 0 ; i < options_.size(); i++)
    {
      delete options_[i];
    }
    options_.clear();
  };

private:
  std::vector<OptionBase*> options_; ///< Store all the option objects
};


}

#endif //TF_OPTIONPERMUTATIONS_H
