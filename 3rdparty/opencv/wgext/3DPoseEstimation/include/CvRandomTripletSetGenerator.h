/*
 * CvRandomTripletSetGenerator.h
 *
 *  Created on: Sep 15, 2008
 *      Author: jdchen
 */

#ifndef CVRANDOMTRIPLETSETGENERATOR_H_
#define CVRANDOMTRIPLETSETGENERATOR_H_

#include <opencv/cxcore.h>

#include <boost/unordered_set.hpp>

/// a struct to hold the triplet set of integers
class CvTripletSet {
public:
  CvTripletSet(int a, int b, int c) {t[0]=a;t[1]=b;t[2]=c;}
  inline int& operator[] (const int i) { return t[i];}
  inline bool operator == (CvTripletSet const& a) {
    return a.t[0] == t[0] && a.t[1] == t[1] && a.t[2] == t[2];
  }
private:
  int t[3];
};

bool operator == (CvTripletSet const& a, CvTripletSet const& b);

/**
 * A generator of random triplet set of integers.
 * Each element of the triplet is a random integer number that is independently
 * and uniformly distributed over the same specified range.
 * If no duplicate is allowed, then the size of the range is restricted to
 * 21 bits (so that, for efficiency, we can encode 3 integer of 21-bit integers
 * into a 64-bit integer, and use a hash set of 64-bit integer for duplicate
 * checking.
 */
class CvRandomTripletSetGenerator {
public:
  CvRandomTripletSetGenerator(
      /// seed for the random number generator
      const int64 seed,
      /// The smallest integer in a random set
      const int lower,
      /// The largest integer in a random set
      const int upper,
      /// If yes, no duplicated set is generated throughout the life span
      /// of this object.
      const bool noDuplicate=true);
  virtual ~CvRandomTripletSetGenerator();
  /// generate a random set
  /// @return a copy of a random triplet set
  CvTripletSet newSet();
protected:
  /// Random number generator
  CvRNG mRandomNumberGenerator;
  /// The smallest integer
  const int    mLower;
  /// The number of possible integer random numbers, (largest-smallest+1)
  const int    mRange;
  /// No duplicate
  const bool   mNoDuplicate;
  /// The total number of non-duplicated sets
  const unsigned int    mMaxNumSets;
  /// Encode the triplet into a 64 bit long
  uint64 mEncodeSetInLong(CvTripletSet const & triplet);
  boost::unordered_set<int64> mSetOfSets;
  /// return a new set, maybe a duplicate of a previously generated one.
  CvTripletSet newSetWithDuplicate();
  /// sort the triplet into increasing order
  void _sort(CvTripletSet& s);
};

#endif /* CVRANDOMTRIPLETSETGENERATOR_H_ */
