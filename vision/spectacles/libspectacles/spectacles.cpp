#include <stdexcept>
#include "spectacles/spectacles.h"
#include "tinyxml-2.5.3/tinyxml.h"
#include "opencv/highgui.h"

using namespace spectacles;

labeled_imageset::labeled_imageset()
{
}

labeled_imageset::~labeled_imageset()
{
  for (size_t i = 0; i < images.size(); i++)
    delete images[i];
  images.clear();
}

bool labeled_imageset::load(const string &labelfile, 
                            const vector<string> &imagefiles)
{
  TiXmlDocument labeldoc;
  if (!labeldoc.LoadFile(labelfile.c_str()))
  {
    printf("couldn't parse label file: %s\n", labelfile.c_str());
    return false;
  }
  vector<labeled_image> labels;
  int img_idx = 0;
  for (TiXmlElement *label_ele =
       labeldoc.RootElement()->FirstChildElement("Object2dFrame");
       label_ele; label_ele = label_ele->NextSiblingElement("Object2dFrame"))
  {
    labeled_image *li = new labeled_image();
    for (TiXmlElement *obj_ele = label_ele->FirstChildElement("Object");
         obj_ele; obj_ele = obj_ele->NextSiblingElement("Object"))
    {
      labelrect r;
      obj_ele->QueryDoubleAttribute("x", &r.x);
      obj_ele->QueryDoubleAttribute("y", &r.y);
      obj_ele->QueryDoubleAttribute("w", &r.w);
      obj_ele->QueryDoubleAttribute("h", &r.h);
      r.name = obj_ele->Attribute("name");
      li->labels.push_back(r);
    }
    li->image = cvLoadImage(imagefiles[img_idx++].c_str());
    if (!li->image)
    {
      printf("%s\n", ("couldn't load image " + imagefiles[img_idx-1]).c_str());
      return false;
    }
    images.push_back(li);
  }
  printf("loaded %d images\n", images.size());
  return true;
}

IplImage *labeled_image::create_annotated_image()
{
  IplImage *img = cvCloneImage(image);
  CvFont font;
  cvInitFont(&font, CV_FONT_VECTOR0, 0.7, 0.8, 0.0, 2.0);
  for (size_t l = 0; l < labels.size(); l++)
  {
    labelrect r = labels[l];
    cvRectangle(img, cvPoint(r.x, r.y), cvPoint(r.x+r.w, r.y+r.h),
                CV_RGB(0, 255, 0));
    cvPutText(img, r.name.c_str(), cvPoint(r.x, r.y), &font, CV_RGB(0,255,0));
  }
  return img;
}

bool labeled_imageset::load(const string &labelfile, int argc, 
                            char **argv, int start)
{
  vector<string> names;
  for (int i = start; i < argc; i++)
    names.push_back(argv[i]);
  return load(labelfile, names);
}

