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
  //  static enum FrameType {SIXDOF, DH //how to tell which mode it's in  //todo add this process i'm going to start with 6dof only

  /* Constructor */
  RefFrame();

  /* Set the parameters for this frame */
  void setParams(double, double, double, double, double, double);
  
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
  double params[6];

  /* A helper function to build a homogeneous transform based on 6dof parameters */
  static  bool fill_transformation_matrix(NEWMAT::Matrix& matrix_pointer, double ax,
					  double ay, double az, double yaw,
					  double pitch, double roll);
  /* Storage of the parent */
  unsigned int parent;

};

class TransformReference
{
public:
  static const unsigned int MAX_NUM_FRAMES = 100;

  /* Set a new frame or update an old one. */
  void set(unsigned int framid, unsigned int parentid, double,double,double,double,double,double);

  /* Get the transform between two frames.  */
  NEWMAT::Matrix get(unsigned int target_frame, unsigned int source_frame);

  /* Constructor */
  TransformReference();

  /* Debugging function that will print to std::cout the transformation matrix */
  void view(unsigned int target_frame, unsigned int source_frame);
  
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
private:
  
  inline RefFrame* getFrame(unsigned int frame_number) { if (frames[frame_number] == NULL) throw InvalidFrame; else return frames[frame_number];};
  /* The frames that the tree can be made of */
  RefFrame* frames[MAX_NUM_FRAMES];


  /* This struct is how the list of transforms are stored  before being generated. */
  typedef struct 
  {
    std::vector<unsigned int> inverseTransforms;
    std::vector<unsigned int> forwardTransforms;
  } TransformLists;

  /* Find the list of connected frames necessary to connect two different frames */
  TransformLists  lookUpList(unsigned int target_frame, unsigned int source_frame);
  
  /* Compute the transform based on the list of frames */
  NEWMAT::Matrix computeTransformFromList(TransformLists list);

};
#endif //LIBTF_HH
