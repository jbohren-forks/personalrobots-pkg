#ifndef COLOR_GUI_H
#define COLOR_GUI_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Button.H>

#include "opencv/cv.h"

#include "cmvision.h"

namespace boost
{
  class mutex;
}

class ColorGui : public Fl_Window
{
  /// \brief Constructor
  public: ColorGui(ros::NodeHandle *nh);

  /// \brief Destructor
  public: virtual ~ColorGui();

  /// \brief Run the app
  public: void Run();

  /// \brief The image callback
  public: void imageCB(const sensor_msgs::ImageConstPtr& msg);

  /// \brief Handle a mouse click
  public:void HandleMouseRelease();

  /// \brief Handle events
  public: int handle(int event);

  /// \brief Reset the color selection
  public: static void ResetCB( Fl_Widget *w, void *data);

  private: ros::NodeHandle *nodeHandle;
  private: ros::Subscriber subscriber;
  private: int mousePosX, mousePosY;

  private: Fl_Box *imageBox;

  private: int width, height;
  private: unsigned char *rgbImage;
  private: unsigned char *uyvyImage;

  private: Fl_Value_Output *rgbOutput[3];
  private: Fl_Value_Output *yuvOutput[3];

  private: Fl_Button *resetButton;

  private: Fl_Output *yuvRangeOutput[3];

  private: boost::mutex *mutex;

  private: CMVision *vision;
};

#endif
