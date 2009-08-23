#include <stdio.h>
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Float_Input.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Round_Button.H>

#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"

#ifndef NULL
#define NULL 0
#endif
// Convert radians to degrees
 #define RTOD(r) ((r) * 180 / M_PI)

// Convert degrees to radians
#define DTOR(d) ((d) * M_PI / 180)


class MainWindow : public Fl_Window
{
  public: MainWindow()
          : Fl_Window(0,0,300,140, "Quaternion Calc")
          {
            Fl::scheme("plastic");
            int y = 5;

            this->degreeButton = new Fl_Round_Button(5,y,40,20,"Degrees");
            this->degreeButton->callback(&MainWindow::UnitButtonCB, this);
            this->degreeButton->value(1);

            this->radiansButton = new Fl_Round_Button(90,y,40,20,"Radians");
            this->radiansButton->callback(&MainWindow::UnitButtonCB, this);
            this->radiansButton->value(0);
            y += 25;

            this->rpyInput[0] = new Fl_Float_Input(50,y,80,20,"Roll:");
            this->rpyInput[0]->value("0.0");
            y += 25;

            this->rpyInput[1] = new Fl_Float_Input(50,y,80,20,"Pitch:");
            this->rpyInput[1]->value("0.0");
            y += 25;
            
            this->rpyInput[2] = new Fl_Float_Input(50,y,80,20,"Yaw:");
            this->rpyInput[2]->value("0.0");
            y += 25;

            y = 45;
            this->rpy2QuatButton = new Fl_Button(145,y,30,20,"@->");
            this->rpy2QuatButton->callback( &MainWindow::Rpy2QuatCB, this);
            y += 25;

            this->quat2RPYButton = new Fl_Button(145,y,30,20,"@4->");
            this->quat2RPYButton->callback( &MainWindow::Quat2RpyCB, this);

            y = 30;
            this->quatInput[0] = new Fl_Float_Input(200,y,80,20,"X:");
            this->quatInput[0]->value("0.0");
            y += 25;
            this->quatInput[1] = new Fl_Float_Input(200,y,80,20,"Y:");
            this->quatInput[1]->value("0.0");
            y += 25;
            this->quatInput[2] = new Fl_Float_Input(200,y,80,20,"Z:");
            this->quatInput[2]->value("0.0");
            y += 25;
            this->quatInput[3] = new Fl_Float_Input(200,y,80,20,"U:");
            this->quatInput[3]->value("1.0");
            y += 25;

            this->end();
            this->show();
          }

  public: virtual ~MainWindow()
          {
            int i;
            for (i=0; i < 3; i++)
            {
              delete this->rpyInput[i];
              this->rpyInput[i] = NULL;
            }
            delete [] this->rpyInput;

          }

  public: static void UnitButtonCB(Fl_Widget *w, void *data)
          {
            MainWindow *self = (MainWindow*)data;

            float roll = atof(self->rpyInput[0]->value());
            float pitch = atof(self->rpyInput[1]->value());
            float yaw = atof(self->rpyInput[2]->value());

            if (self->radiansButton == w)
            {
              if (self->degreeButton->value() == 1)
              {
                self->degreeButton->value(0);

                roll = DTOR(roll);
                pitch = DTOR(pitch);
                yaw = DTOR(yaw);
              }
            }
            else 
            {
              if (self->radiansButton->value() == 1)
              {
                self->radiansButton->value(0);
                roll = RTOD(roll);
                pitch = RTOD(pitch);
                yaw = RTOD(yaw);
              }
            }

            ((Fl_Round_Button*)w)->value(1);

            char str[50];
            sprintf(str,"%7.4f", roll);

            self->rpyInput[0]->value(str);

            sprintf(str,"%7.4f", pitch);
            self->rpyInput[1]->value(str);

            sprintf(str,"%7.4f", yaw);
            self->rpyInput[2]->value(str);

          }

  public: static void Rpy2QuatCB(Fl_Widget *w, void *data)
          {
            MainWindow *self = (MainWindow*)data;
            float roll = atof(self->rpyInput[0]->value());
            float pitch = atof(self->rpyInput[1]->value());
            float yaw = atof(self->rpyInput[2]->value());

            if (self->degreeButton->value() == 1)
            {
              roll = DTOR(roll);
              pitch = DTOR(pitch);
              yaw = DTOR(yaw);
            }

            btQuaternion quat(yaw, pitch, roll);

            char str[50];

            sprintf(str,"%7.4f",quat.getX());
            self->quatInput[0]->value(str);

            sprintf(str,"%7.4f",quat.getY());
            self->quatInput[1]->value(str);

            sprintf(str,"%7.4f",quat.getZ());
            self->quatInput[2]->value(str);

            sprintf(str,"%7.4f",quat.getW());
            self->quatInput[3]->value(str);
          }

  public: static void Quat2RpyCB(Fl_Widget *w, void *data)
          {
            MainWindow *self = (MainWindow*)data;
            float x, y, z, u;
            btScalar br, bp, by;
            float roll,pitch,yaw;

            x = atof(self->quatInput[0]->value());
            y = atof(self->quatInput[1]->value());
            z = atof(self->quatInput[2]->value());
            u = atof(self->quatInput[3]->value());

            btQuaternion quat(x,y,z,u);
            btTransform tf(quat);
            tf.getBasis().getEulerYPR(by, bp, br);
            roll = br;
            pitch = bp;
            yaw = by;

            if (self->degreeButton->value() == 1)
            {
              roll = RTOD(roll);
              pitch = RTOD(pitch);
              yaw = RTOD(yaw);
            }

            char str[50];
            sprintf(str,"%7.4f",roll);
            self->rpyInput[0]->value(str);

            sprintf(str,"%7.4f",pitch);
            self->rpyInput[1]->value(str);

            sprintf(str,"%7.4f",yaw);
            self->rpyInput[2]->value(str);
          }

  private: Fl_Round_Button *degreeButton;
  private: Fl_Round_Button *radiansButton;

  private: Fl_Float_Input *rpyInput[3];
  private: Fl_Float_Input *quatInput[4];

  private: Fl_Button *rpy2QuatButton;
  private: Fl_Button *quat2RPYButton;
};

int main(int argc, char **argv)
{
  MainWindow win;

  return Fl::run();
}
