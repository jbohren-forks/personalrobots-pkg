// 
// FLTK window definitions
//

#ifndef flwin_h
#define flwin_h

#define index xx		// do we need this????  FLTK sometimes has problems
#undef index
#include "FL/Fl.H"
#include "FL/Fl_Group.H"
#include "FL/Fl_Window.H"
#include "FL/fl_draw.H"
#include "FL/Fl_Double_Window.H"
#include "FL/Fl_Browser.H"
#include "FL/Fl_Overlay_Window.H"
#include "FL/Fl_Select_Browser.H"
#include "FL/Fl_Text_Display.H"
#include "FL/Fl_Text_Buffer.H"
#include "FL/Fl_Menu_Bar.H"
#include "FL/Fl_File_Chooser.H"
#include <string.h>


// for MSVC this should be defined correctly
#define IMPORT 

// pixel types
#define MONOCHROME 0
#define RGB24      1
#define DISPARITY  2

class imWindow : public Fl_Window
{
public:
  IMPORT imWindow(int x, int y, int h, int w);
  IMPORT void ClearImage();
  IMPORT void DeleteImage();
  IMPORT void draw();           // drawing routine 
  IMPORT void DisplayImage(unsigned char *im, int w, int h, int ls, int type = MONOCHROME, 
                           int nd = 64, int s = 1, double gam = 0.0, int x = 0, int y = 0);
  // draw overlays
  virtual void DrawOverlay(int which, void *); // draw overlays with overlay fn; can be overridden

  // if we want to do something with mouse clicks...
  void ButtonHandler(int (*fn)(int, int, int, int, int, imWindow *))
  { bhandler = fn; };
  // if we want to do something with keyboard presses...
  void KeyHandler(int (*fn)(int, int, imWindow *))
  { khandler = fn; };
  // set up overlay function
  void DrawOverlayFn(void (*fn)(imWindow *, int which, void *ovArg))
  { ofn = fn; };

  IMPORT void line(int x1, int y1, int x2, int y2); // draw line on overlay
  IMPORT void rect(int x1, int y1, int x2, int y2); // draw rectangle on overlay
  void color(int c)             // set the color
  { myColor = c; };

  // return the image pixel coords of the interest region
  void imageRegion(int *x, int *y, int *w, int *h);
  int intX, intY, intW, intH; // interest rectangle

  int dwidth, dheight;          // display window size, doesn't change

  // scale image coords to window coords
  IMPORT int Win2ImX(int x);
  IMPORT int Win2ImY(int y);
  IMPORT int Im2WinX(int x);
  IMPORT int Im2WinY(int y);

  // saving to a file
  // saves to a file sequence, if no arguments are given
  IMPORT void Save(char *fname = NULL, int num = -1);
  int saveNum;
  char *saveName;		// base file name

private: 
  void drawit();                // draws the pixmap on the screen
  int width, height;            // displayed image size
  int xoff, yoff;               // displayed image offset withing display window
  int skipw, linesize;          // pixel decimation, line size of original image 
  unsigned char *pixelData;
  int pixelType;
  int curSize, bufSize;         // internal buffering
  int ndisp;
  double gamma;			// for gamma conversion
  unsigned char gamtab[256];
  // to check key and mouse events
  IMPORT int handle(int);       // has to be IMPORTed so subclasses can see it
  bool moving;                  // dragging the mouse
  int (*bhandler)(int, int, int, int, int, imWindow *);
  int (*khandler)(int, int, imWindow *);
  void (* ofn)(imWindow *, int which, void *ovarg); // set this to be the overlay drawing fn
  int myColor;
  void *ovArg;                  // stored arg to overlay fn
  int myWhich;                  // stored which image for overlay fn
  unsigned char *saveBuf;	// buffer for saved images
};


//
// Draws a single string, int, or double value, with an associated label
// Can change formatting and type at any time; value() fn is overloaded
// value(char *) acts as formatter for numeric values
// If formatter is changed, the numeric value must be resent
//

class Fl_Value : public Fl_Widget 
{

public:
  Fl_Value(int x, int y, int w, int h, const char *l=0, const char *val = "");
  void val(char *v); 
  void val(int x);
  void val(double x);
  void setcolor(Fl_Color x) { c = x; }

private:
  void draw()
  { 
    fl_color(color());
    fl_rectf(x(),y(),w(),h());  // erase background
    fl_font(FL_HELVETICA_BOLD, labelsize());
    fl_color(c);
    fl_draw(buf, x(), y()+ h()/2 + fl_height()/2 - fl_descent() );
  }
  Fl_Color c;
  char str[256];
  char buf[256];
};


IMPORT int fltk_check(void);

#endif

