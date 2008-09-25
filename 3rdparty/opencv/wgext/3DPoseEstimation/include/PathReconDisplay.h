/*
 * PathReconDisplay.h
 *
 *  Created on: Sep 8, 2008
 *      Author: jdchen
 */

#ifndef CVPATHRECONDISPLAY_H_
#define CVPATHRECONDISPLAY_H_

namespace cv { namespace willow {
/**
 * A wrapper class to perform path reconstruction and display images
 * along the way (Under reconstruction).
 */
class PathReconDisplay {
public:
  PathReconDisplay();
  virtual ~PathReconDisplay();
};
}
}
#endif /* CVPATHRECONDISPLAY_H_ */
