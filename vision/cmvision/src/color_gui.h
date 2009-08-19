#ifndef COLOR_GUI_H
#define COLOR_GUI_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>

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

  private: ros::NodeHandle *nodeHandle;

  private: Fl_Box *imageBox;
};

#endif
