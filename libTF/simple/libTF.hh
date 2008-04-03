#ifndef LIBTF_HH
#define LIBTF_HH
#include <iostream>
#include <iomanip>
#include <newmat/newmat.h>
#include <newmat/newmatio.h>
#include <math.h>
#include <vector>

class RefFrame
{
public:

  /* The type of frame
   * This determines how many different parameters to expect to be updated, versus fixed */
  enum ParamTypeEnum {SIXDOF, DH };//how to tell which mode it's in  //todo add this process i'm going to start with 6dof only

  /* Constructor */
  RefFrame();

  /* Set the parameters for this frame */
  void setParamsXYZYPR(double, double, double, double, double, double);
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
  /* Storage of the parametsrs 
   * NOTE: Depending on if this is a 6dof or DH parameter the storage will be different. */
  double params[6]; 

  /* A helper function to build a homogeneous transform based on 6dof parameters */
  static  bool fill_transformation_matrix(NEWMAT::Matrix& matrix_pointer, double ax,
					  double ay, double az, double yaw,
					  double pitch, double roll);

  /* A helper function to build a homogeneous transform based on DH parameters */
  static bool fill_transformation_matrix_from_dh(NEWMAT::Matrix& matrix, double theta,
					  double length, double distance, double alpha);

  /* Storage of the parent */
  unsigned int parent;

  ParamTypeEnum paramType;

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
