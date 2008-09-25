/*
 * LevMarqPartitioned.h
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#ifndef LEVMARQPARTITIONED_H_
#define LEVMARQPARTITIONED_H_

#include "LevMarq2.h"

namespace cv {
namespace willow {

/// a sparse Levenberg Marquardt optimization, in which the parameter
/// is partitioned into two parts to take advantage of the sparseness
/// in the Jacobian's.
class LevMarqPartitioned: public LevMarq {
public:
  LevMarqPartitioned();
  virtual ~LevMarqPartitioned();
};

}
}

#endif /* LEVMARQPARTITIONED_H_ */
