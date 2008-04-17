//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef LIBTF_HH
#define LIBTF_HH
#include <iostream>
#include <iomanip>
#include <newmat/newmat.h>
#include <newmat/newmatio.h>
#include <math.h>
#include <vector>
#include "Quaternion3D.hh"

class RefFrame
{
public:

  /* Constructor */
  RefFrame();

  /* Set the parameters for this frame */
  void setParamsQuaternion3D(double, double, double, double, double, double, double);
  void setParamsEulers(double, double, double, double, double, double);
  void setParamsDH(double, double, double, double);
  
  /* Get the parent node */
  inline unsigned int getParent(){return parent;};

  /* Return tha parent node */
  inline void setParent(unsigned int parentID){parent = parentID;};

  /* Generate and return the transform associated with gettingn into this frame */
  NEWMAT::Matrix getMatrix();
  
  /* Generate and return the transform associated with getting out of this frame.  */
  NEWMAT::Matrix getInverseMatrix();


private:
  /* Storage of the parametsrs */
  Quaternion3D myQuat;

  /* Storage of the parent */
  unsigned int parent;

};

class TransformReference
{
public:
  /************* Constants ***********************/
  static const unsigned int ROOT_FRAME = 1;  //Hard Value for ROOT_FRAME
  static const unsigned int NO_PARENT = 0;  //Value for NO_PARENT

  static const unsigned int MAX_NUM_FRAMES = 100;   /* The maximum number of frames possible */
  static const unsigned int MAX_GRAPH_DEPTH = 100;   /* The maximum number of times to descent before determining that graph has a loop. */

  /* Constructor */
  TransformReference();

  /********** Mutators **************/
  /* Set a new frame or update an old one. */
  void set(unsigned int framid, unsigned int parentid, double,double,double,double,double,double);
  // Possible exceptions TransformReference::LookupException

  /*********** Accessors *************/

  /* Get the transform between two frames by frame ID.  */
  NEWMAT::Matrix get(unsigned int target_frame, unsigned int source_frame);
  // Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
  // TransformReference::MaxDepthException

  /* Debugging function that will print to std::cout the transformation matrix */
  void view(unsigned int target_frame, unsigned int source_frame);
  // Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException, 
  // TransformReference::MaxDepthException






  /************ Possible Exceptions ****************************/

  /* An exception class to notify of bad frame number */
  class LookupException : public std::exception
  {
  public:
    virtual const char* what() const throw()    { return "InvalidFrame"; }
  } InvalidFrame;

  /* An exception class to notify of no connection */
  class ConnectivityException : public std::exception
  {
  public:
    virtual const char* what() const throw()    { return "No connection between frames"; }
  private:
  } NoFrameConnectivity;

  /* An exception class to notify that the search for connectivity descended too deep. */
  class MaxDepthException : public std::exception
  {
  public:
    virtual const char* what() const throw()    { return "Search exceeded max depth.  Probably a loop in the tree."; }
  private:
  } MaxSearchDepth;

private:
  /******************** Internal Storage ****************/

  /* The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  RefFrame* frames[MAX_NUM_FRAMES];


  /* This struct is how the list of transforms are stored before being passed to computeTransformFromList. */
  typedef struct 
  {
    std::vector<unsigned int> inverseTransforms;
    std::vector<unsigned int> forwardTransforms;
  } TransformLists;

  /************************* Internal Functions ****************************/
  
  /* An accessor to get a frame, which will throw an exception if the frame is no there. */
  inline RefFrame* getFrame(unsigned int frame_number) { if (frames[frame_number] == NULL) throw InvalidFrame; else return frames[frame_number];};

  /* Find the list of connected frames necessary to connect two different frames */
  TransformLists  lookUpList(unsigned int target_frame, unsigned int source_frame);
  
  /* Compute the transform based on the list of frames */
  NEWMAT::Matrix computeTransformFromList(TransformLists list);

};
#endif //LIBTF_HH
