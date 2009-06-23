// THIS IS AN APPLICATION TO LABEL RECTANGLES ON IMAGES
// It produces a file consisting of lines of imagename x y w h
// Gary Bradski (c) 11/16/08 for Willow Garage.  Released under standard BSD license.
//
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>


using namespace std;
//GLOBALS
void postString(std::string wname, std::string prompt,int x=10, int y=30, bool clr = true);
IplImage* img0 = 0, *img = 0, *io = 0; //from disk, copy, controls
CvFont iofont;
std::string current_image;
char rect[256];
//left mouse
CvPoint pt1 = {-1,-1};
CvPoint pt2 = {-1, -1};
bool rectangle = false;
int draw_it = 0; //0 not drawing, 1 show mouse movement, 2 draw the rectangle
//right mouse
CvPoint rpt1,rpt2,drpt = {0,0},cpt1,cpt2;
bool rmouse = false;
int mquad = 0; 	//Mouse quadrent on rectange. 0 => not on rectangle (not really a quadrent, d'oh!)
				// 11-13 topL topMiddle topR; 21-23 midL midM midR; 31-33 botL botM botR
				
//Function to find what quadrent of a rectangle you're in
// 11-13 topL topMiddle topR; 21-23 midL midM midR; 31-33 botL botM botR;  Else: 0
int rectQuadFinder(CvPoint p1, CvPoint p2, CvPoint mypt)
{
	//Put points in 1:topL, 2:botR position
	CvPoint tl,br;
	if(p1.x < p2.x) { tl.x = p1.x; br.x = p2.x;} else {tl.x = p2.x; br.x = p1.x;}
	if(p1.y < p2.y) { tl.y = p1.y; br.y = p2.y;} else {tl.y = p2.y; br.y = p1.y;}
	//Check if we're outside of the rectangle
	if((mypt.x < tl.x)||(mypt.x > br.x)||(mypt.y<tl.y)||(mypt.y>br.y))
		return 0;
	//Find X quad 
	int quad = 0;
	int lx = (br.x - tl.x)/5;
	if(mypt.x <= tl.x + lx)
	 	quad = 1;
	else if(mypt.x >= br.x - lx)
		quad = 3;
	else
		quad = 2;
	//Find Y quad
	int ly = (br.y - tl.y)/5;
	if(mypt.y < tl.y + ly)
		quad += 10;
	else if (mypt.y >= br.y - ly)
		quad += 30;
	else
		quad += 20;
	return quad;
}

// MOUSE
void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !img || !img0)
        return;
//  LEFT MOUSE DRAWS RECTANGLE:
    if(event == CV_EVENT_LBUTTONDOWN && (draw_it == 0)) //Starting a rectangle
    {
    	rectangle = false;
    	cvCopy(img0,img);
    	pt1 = cvPoint(x,y);
    	pt2 = cvPoint(x+1,y+1);
		draw_it = 1;
	}
	if(event == CV_EVENT_MOUSEMOVE && (draw_it == 1)) //Drawing a rectangle
	{
		pt2 = cvPoint(x,y);
		cvRectangle( img, pt1, pt2, CV_RGB(-1,-1,255));
		cvShowImage("image",img);
		postString("Controls", current_image,10, 30, true);
		sprintf(rect,"x1=%d, y1=%d, x2=%d, y2=%d",pt1.x,pt1.y,pt2.x,pt2.y);
		postString("Controls", rect, 10, 75, false);
	} 
    if( event == CV_EVENT_LBUTTONUP && (draw_it == 1) )
    {
    	pt2 = cvPoint(x,y);
		cvCopy(img0,img);
		cvRectangle( img, pt1, pt2, CV_RGB(255,255,255));
		cvShowImage("image",img);
		postString("Controls", current_image,10, 30, true);
		sprintf(rect,"x1=%d, y1=%d, x2=%d, y2=%d",pt1.x,pt1.y,pt2.x,pt2.y);
		//Put points in cannonical order
		CvPoint tl,br;
		if(pt1.x < pt2.x) { tl.x = pt1.x; br.x = pt2.x;} else {tl.x = pt2.x; br.x = pt1.x;}
		if(pt1.y < pt2.y) { tl.y = pt1.y; br.y = pt2.y;} else {tl.y = pt2.y; br.y = pt1.y;}
		pt1 = tl;
		pt2 = br;		
		//Show and go
		postString("Controls", rect, 10, 75, false);
		rectangle = true;
		draw_it = 0;    	
	}
// RIGHT MOUSE MOVES RECTANGLE:
	if(rectangle)
	{
		if(event == CV_EVENT_RBUTTONDOWN)
		{
			rpt1 = cvPoint(x,y);
			drpt = cvPoint(0,0);
			mquad = rectQuadFinder(pt1, pt2, rpt1);
			rmouse = true;
		}
		if((event == CV_EVENT_MOUSEMOVE) && (rmouse))
		{
			rpt2 = cvPoint(x,y);
			drpt = cvPoint(rpt2.x - rpt1.x,rpt2.y - rpt1.y);
		}
		if((event == CV_EVENT_RBUTTONUP) && (rmouse))
		{
			rpt2 = cvPoint(x,y);
			drpt = cvPoint(rpt2.x - rpt1.x,rpt2.y - rpt1.y);
		}
		if(rmouse)
		{
			cvCopy(img0,img);
			//Deal only with topleft and bottom right points
			CvPoint tl,br;
			if(pt1.x < pt2.x) { tl.x = pt1.x; br.x = pt2.x;} else {tl.x = pt2.x; br.x = pt1.x;}
			if(pt1.y < pt2.y) { tl.y = pt1.y; br.y = pt2.y;} else {tl.y = pt2.y; br.y = pt1.y;}
			cpt1 = tl,	cpt2 = br; 

			//HANDLE WHICH SECTION OF THE RECTANGLE WE'RE ADJUSTING
			switch(mquad)
			{
				case 11: //top left
					cpt1.x = tl.x + drpt.x;
					cpt1.y = tl.y + drpt.y;
					break;
				case 12: // top
					cpt1.y = tl.y + drpt.y;
					break;
				case 13: // top right
					cpt2.x = br.x + drpt.x;
					cpt1.y = tl.y + drpt.y;
					break;
				case 21: // left
					cpt1.x = tl.x + drpt.x;
					break;
				case 22: // middle
					cpt1.x = tl.x + drpt.x; cpt1.y = tl.y + drpt.y;
					cpt2.x = br.x + drpt.x; cpt2.y = br.y + drpt.y;
					break;
				case 23: // right
					cpt2.x = br.x + drpt.x;
					break;
				case 31: // bottom left
					cpt1.x = tl.x + drpt.x;
					cpt2.y = br.y + drpt.y;
					break;
				case 32: // bottom
					cpt2.y = br.y + drpt.y;
					break;
				case 33: // bottom right
					cpt2.x = br.x + drpt.x;
					cpt2.y = br.y + drpt.y;
					break;
			}
			//Draw and show it
			cvRectangle( img, cpt1, cpt2, CV_RGB(255,255,255));
			cvShowImage("image",img);
			postString("Controls", current_image,10, 30, true);
			sprintf(rect,"x=%d, y=%d, w=%d, h=%d",cpt1.x, cpt1.y, cpt2.x-cpt1.x, cpt2.y-cpt1.y);
			postString("Controls", rect, 10, 75, false);
			if(event == CV_EVENT_RBUTTONUP)
			{
				pt1 = cpt1;
				pt2 = cpt2;
				mquad = 0;
				rmouse = false;
			}
		}
	} else 
	{
		rmouse = false;
		mquad = 0;
		drpt = cvPoint(0,0);
	}
}

//POST MESSAGE
void postString(std::string wname, std::string prompt,int x, int y, bool clr)
{	
   	CvPoint orgprompt = cvPoint(x,y);
   	if(clr)
		cvSetZero(io);
	cvPutText(io, prompt.c_str(), orgprompt, &iofont,CV_RGB(100,100,255)); //Write string
	cvShowImage(wname.c_str(),io);
}  	

//GET USER INPUT
std::string cvGetString(std::string wname, std::string prompt, std::string init,int std1_win0 = 0, bool clr = true)
{
	  std::string str=init;
	   CvPoint orgprompt = cvPoint(10,30);
	   CvPoint org = cvPoint(10,75);
	  if(std1_win0)
	  {
		printf("%s\n%s",prompt.c_str(),str.c_str());
		fflush(stdout);
	  }
	  else 
	  {
		cvSetZero(io);
		cvPutText(io, prompt.c_str(), orgprompt, &iofont,CV_RGB(0,100,255)); //Write string
		cvPutText(io, str.c_str(), org, &iofont,CV_RGB(255,255,255)); //Write string
		cvShowImage(wname.c_str(),io);
	  }  	
	 int c = 0, slen;
	 //COLLECT USER INPUT, IGNORE WHITESPACE, ESC OUT
	 while (1) 
	 {
		c = cvWaitKey(0) & 0xFF;
		if(c == 27) {str.erase(); break;}
		if((c == 0)||(c > 128)) continue;
		if((c == 13)||(c == 10)) break; //Carriage return and/or line feed => accept this label
		if((c == ' ')||(c == '\t')) continue; //Ignore white space
		slen = str.length();
		if(!std1_win0)
			cvPutText(io, str.c_str(), org, &iofont,CV_RGB(0,0,0)); //Blank everything
		else
		{
				printf("\r");
				for(int u=0; u<slen; ++u)
					printf(" ");
				printf("\r");
		}			
		if(c == 8) //backspace
		{
			//OVERLAY MASK ONTO IMAGE
			if(std1_win0)
			{
				printf("\r");
				for(int u=0; u<slen; ++u)
					printf(" ");
				printf("\r");
			}
			if(slen)
				str.erase(slen - 1);
		}
		else
		{
			str.append(1, (char)c);
		}
		if(std1_win0)
		{
			printf("%s\r",str.c_str());
			fflush(stdout);
		}else
		{
			cvPutText(io, str.c_str(), org, &iofont,CV_RGB(255,255,255)); //Write string
			cvShowImage(wname.c_str(),io);
		}
	} //End while
		//CLEAN UP AND OUT
		if(std1_win0)
		{
			printf("string %s\n",str.c_str());
			printf("\n");
		}
		else
		{
//			set_name = 0;
//			cvSetTrackbarPos("Set Name", "bg_controls", set_name);
			cvSetZero(io);
			cvPutText(io, prompt.c_str(), orgprompt, &iofont,CV_RGB(0,100,255)); //Write string
			cvPutText(io, str.c_str(), org, &iofont,CV_RGB(255,255,255)); //Write string
			if(clr) cvZero(io);
			cvShowImage(wname.c_str(),io);
		}
	  return str;
	}

//INSTRUCTIONS
void help()
{
   printf("\n"
   "B0X SEGMENTATION APPLICATION, for instance to create a training set for the Viola-Jones Classifier\n"
   "CALL:  ./mark_rectangle <list_of_images> [output_list] [sub_window_base_name]\n\n"
   "WHERE:\n"
   "\t<list_of_images>     is just a list of [path]/image_names, one per line\n"
   "\t[output_list]        will write a file containing lines of:\n"
   "\t                        [path]/image_name 1 x y w h  -- I.e. the training format for Viola-Jones\n"
   "\t                        The default output file name is default.txt\n"
   "\t[sub_window_base...] optional.  With large images, to speed up training, providing a base name\n"
   "\t                        here will cause the application to also cut out a small image around the \n"
   "\t                        user selected rectangle and name them sequentially \"base####.jpg\" together\n"
   "\t                        with an output file in Viola-Jones format as above but relative to these smaller\n"
   "\t                        images\n\n"
   "DIRECTIONS\n"
   "\tUse the left mouse button (click and drag) to draw a rectangle\n"
   "\tUse the right mouse button to move or resize that rectangle by its middle, edges or corners\n"
   "\tUse the control pannel to\n"
   "\t\tClear    the rectangle you just drew (you can also start again by just left click and drag)\n"    
   "\t\tRisize   the image you are viewiing by powers of two (helps when dealing with different sized images)\n"
   "\t\tSKIP     if you decide to not use one of the images, you click the slide bar to skip it\n"
   "\t\tWrite    once you have a rectangle that you are happy with, click the slide bar to store it's rectangle\n\n"
   "HOT KEYS (focus must be on one of the images, not the consol):\n"
            "\tESC - quit the program\n"
            "\tq,Q - quit the program\n"
            "\th,H - display this help\n\n"
 	);
}

/////////// SLIDERS ////////////

//controls
int _Clear = 0, _Skip = 0, _Write = 0;

void on_Clear(int s)
{
 	draw_it = 0;
 	rectangle = false;
 	rmouse = false;
 	mquad = 0;
 	drpt = cvPoint(0,0);
 	pt1 = cvPoint(-1,-1);
 	pt2 = cvPoint(-1,-1);
	_Clear = 0;
	if(img && img0) 
 	{
 		cvCopy(img0,img);
 		cvShowImage("image",img);
	}
	cvSetTrackbarPos("Clear", "Controls", _Clear);
	postString("Controls", "Rectangle Cleared");
	cvShowImage("Controls",io);
}
void on_Skip(int s){}
void on_Write(int w){}

//image
int _Resize = 1;
void on_Resize(int s)
{
	if(!img0) return;
	if(s == 0) 
		cvResizeWindow("image", img->width/4, img->height/4 );
	if(s == 1) 
		cvResizeWindow("image", img->width/2, img->height/2 );		
	if(s == 2)
		cvResizeWindow("image", img->width, img->height );
	if(s == 3) 
		cvResizeWindow("image", img->width*2, img->height*2 );
	if(s == 4) 
		cvResizeWindow("image", img->width*4, img->height*4 );
	cvShowImage("image", img);
}
		
		
				
//////////  MAIN //////////////////////
int main( int argc, char** argv )  
{
	//Command line
	if (argc < 2) { help(); return -1; }
	std::vector<std::string> file;
	std::string line;
	file.clear();
	std::ifstream infile (argv[1], std::ios_base::in);
	while (getline(infile, line, '\n'))
	{
		file.push_back (line);
	}
	int num_files = (int)file.size();
	infile.close();

	help();     
	std::cout << "Read " << file.size() << " lines.\n";
	  //output file	
	std::string ofilename = "default.txt";
	if(argc >= 3)
		ofilename = argv[2];
	std::ofstream ofile(ofilename.c_str());
	  //output sub-images
	bool write_sub_image = false;
	std::string oI = "Img_";
	std::ofstream subofile;
	if(argc >= 4)
	{
		write_sub_image = true;
		oI = argv[3];
		char ctmp[256];
		sprintf(ctmp,"%s.txt",argv[3]);
		subofile.open(ctmp);
	}

    //Control panel
	cvInitFont(&iofont, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2);
	io = cvCreateImage(cvSize(1000,100),IPL_DEPTH_8U, 3);  
  	cvNamedWindow("Controls",0);
  	cvCreateTrackbar("Clear","Controls", &_Clear, 1, on_Clear);
    cvCreateTrackbar("Resize","Controls",&_Resize,5,on_Resize);
  	cvCreateTrackbar("SKIP","Controls", &_Skip, 1, on_Skip);
  	cvCreateTrackbar("Write","Controls", &_Write, 1, on_Write);
  	
	//Display window
    cvNamedWindow( "image", 0 );  	
 
    cvSetMouseCallback( "image", on_mouse, 0 );
 	 	
  	//Image read loop
//    CvRNG rng = cvRNG(-1);
	for(int i=0; i<num_files; ++i)
	{
		if( (img0 = cvLoadImage(file[i].c_str(),1)) == 0 )
		{
			std::cerr << "Couldn't read image " << file[0].c_str() << std::endl;
			return -1;
		}
		current_image = file[i];
		postString("Controls", file[i]);
		img = cvCloneImage( img0 );
		printf("%s: w,h=%d, %d\n", file[i].c_str(), img->width, img->height);
	 
		//INITIALIZE CONTROL AND DISPLAY:
		cvShowImage( "Controls",io);	
		_Clear = 0, _Skip = 0, _Write = 0;
		mquad = 0;
		rectangle = false;
		rmouse = false;
		pt1 = cvPoint(-1,-1);
		pt2 = cvPoint(-1,-1);
		drpt = cvPoint(0,0);
		cvSetTrackbarPos("Clear", "Controls", _Clear);
		cvSetTrackbarPos("SKIP", "Controls", _Skip);
		cvSetTrackbarPos("Write", "Controls", _Write);

		cvShowImage( "image", img );
		on_Resize(_Resize);
		//DEAL WITH THIS IMAGE:
		for(;;)
		{
			if(_Skip) break;
			if(_Write)
			{
				//Write away
				if(rectangle)
				{
					int w = pt2.x - pt1.x;
					int h = pt2.y - pt1.y;
					if(write_sub_image) //For large images, to speed training, we can output smaller subimages
					{
						//Construct sub-image ROI
						int w2 = w/2, h2 = h/2;
						int rx = pt1.x - w2;
						if(rx < 0) rx = 0;
						int ry = pt1.y - h2;
						if(ry < 0) ry = 0;
						CvPoint npt1 = cvPoint(pt1.x - rx, pt1.y - ry);
						CvPoint npt2 = cvPoint(pt2.x - rx, pt2.y - ry);
						int rx2 = pt2.x + w2;
						if(rx2 >= img->width) rx2 = img->width - 1;
						int rw = rx2 - rx;
						int ry2 = pt2.y + h2;
						if(ry2 >= img->height) ry2 = img->height - 1;
						int rh = ry2 - ry;
						CvRect subrect = cvRect(rx,ry,rw,rh);
						cvSetImageROI( img0, subrect );
						IplImage *subI = cvCreateImage(cvSize(rw,rh),IPL_DEPTH_8U,3);
						cvCopy(img0,subI);
						cvResetImageROI(img0);
						char iname[256];
						sprintf(iname,"%s%.4d.jpg",oI.c_str(),i);
						cvSaveImage(iname,subI);
						subofile << iname << " 1 " << pt1.x - rx << " " << pt1.y - ry << " " << w << " " << h << endl;
						cvReleaseImage(&subI);
					} 
					ofile << current_image << " 1 " << pt1.x << " " << pt1.y << " " <<  w << " " << h << endl;
				break;				
				} //writing with a selected rectangle
				else //oops, trying to write with no selected rectangle
				{
					cout << "\nWARNING: You tried to write without selecting a rectangle" << endl;
					_Write = 0;
					cvSetTrackbarPos("Write", "Controls", _Write);
				}
			}
			//Key board interaction
			int c = cvWaitKey(5)&0xFF;
			if( (c == 27)||(c == 'q')||(c == 'Q'))
				return 0;
			switch(c)
			{
				case 'h':
				case 'H':
					help();
					break;		
			}
		}//end for dealing with an images
		cvReleaseImage(&img);
		cvReleaseImage(&img0);
	}//end for all images
	cout << "\nWrote file to " << ofilename << "\n" << endl;
	if(write_sub_image)
		cout << "\nWrote " << oI << ".txt and images: " << oI << "*.jpg\n" << endl;
    return 0;
}

