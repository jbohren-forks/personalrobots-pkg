#include <cstdio>
#include <cassert>
#include <vector>
#include <string>
#include <deque>
#include <sys/types.h>
#include <dirent.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
using std::vector;
using std::string;
using std::deque;

class Centroid
{
public:
  float col;
  int row, val, noisy;
  Centroid(float _col, int _row, int _val) 
  : col(_col), row(_row), val(_val), noisy(0) { }
};

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    printf("usage: laser_extractor DIRNAME\n");
    return 1;
  }
  const string fbase = argv[1];
  DIR *dir = opendir(fbase.c_str());
  assert(dir);
  vector<string> filenames;
  for (struct dirent *e = readdir(dir); e; e = readdir(dir))
    if (strstr(e->d_name, "jpg"))
      filenames.push_back(e->d_name);
  sort(filenames.begin(), filenames.end());

//  IplImage *ref_image = cvLoadImage((fbase + "/" + filenames[0]).c_str(), 
//                                    CV_LOAD_IMAGE_GRAYSCALE);
//  assert(ref_image);
  cvNamedWindow("borg");
  deque<IplImage *> images;
  for (size_t i = 0; i < filenames.size(); i++)
  {
    //printf("%d / %d\n", i, filenames.size());
    IplImage *image = cvLoadImage((fbase + "/" + filenames[i]).c_str(),
                                  CV_LOAD_IMAGE_GRAYSCALE);
    string theta_str = filenames[i].substr(11, 12);
    double theta = atof(theta_str.c_str());
    if (images.size() == 0)
    {
      images.push_back(image);
      continue;
    }
    IplImage *ref_image = images.front();
    IplImage *diff_image = cvCreateImage(cvGetSize(image), image->depth, 1);
    for (int row = 0; row < image->height; row++)
      for (int col = 0; col < image->width; col++)
      {
        int cur  = CV_IMAGE_ELEM(image,     unsigned char, row, col);
        int prev = CV_IMAGE_ELEM(ref_image, unsigned char, row, col);
        if (cur - prev > 15) // the laser always injects light
          CV_IMAGE_ELEM(diff_image, unsigned char, row, col) = cur - prev;
        else
          CV_IMAGE_ELEM(diff_image, unsigned char, row, col) = 0;
      }
    //cvSmooth(diff_image, diff_image, CV_MEDIAN, 3);
    vector<Centroid> centroids;
    for (int row = 0; row < image->height; row++)
    {
      vector< vector<int> > clusters;
      vector<int> cluster;
      bool in_cluster = false;
      for (int col = 0; col < image->width; col++)
      {
        int v = CV_IMAGE_ELEM(diff_image, unsigned char, row, col);
        if (v)
        {
          cluster.push_back(col);
          in_cluster = true;
        }
        if (!v || col == image->width-1)
        {
          if (in_cluster)
          {
            clusters.push_back(cluster);
            in_cluster = false;
            cluster.clear();
          }
        }
      }
      if (clusters.size() == 0)
        continue; // nothing happened on this row
      float max_avg = -1, max_centroid = -1;
      for (size_t c = 0; c < clusters.size(); c++)
      {
        float avg = 0, centroid = 0;
        for (size_t i = 0; i < clusters[c].size(); i++)
        {
          unsigned char v = CV_IMAGE_ELEM(diff_image, unsigned char, 
                                          row, clusters[c][i]);
          //printf("%d = %d    ", clusters[c][i], v);
          avg += v;
          centroid += v * clusters[c][i];
        }
        centroid /= avg;
        avg /= clusters[c].size();
        if (avg > max_avg)
        {
          max_avg = avg;
          max_centroid = centroid;
        }
      }
      centroids.push_back(Centroid(max_centroid, row, 
           CV_IMAGE_ELEM(ref_image, unsigned char, row, (int)max_centroid)));
      //printf("centroid = %.2f\n", max_centroid);
    }
    // forward pass
    const double MIN_SLOPE = 0.5;
    const int MAX_ROW_SKIP = 1;
    for (vector<Centroid>::iterator c = centroids.begin(); 
         c + 1 != centroids.end(); ++c)
    {
      vector<Centroid>::iterator next_c = c + 1;
      //printf("%d\n", next_c->row - c->row);
      if (fabs((next_c->row - c->row) / (next_c->col - c->col)) < MIN_SLOPE ||
          next_c->row - c->row > MAX_ROW_SKIP)
        c->noisy++;
    }
    // backward pass
    for (vector<Centroid>::reverse_iterator c = centroids.rbegin();
         c + 1 != centroids.rend(); ++c)
    {
      vector<Centroid>::reverse_iterator next_c = c + 1;
      //double slope = fabs((next_c->row - c->row) / (next_c->col - c->col));
      if (fabs((c->row - next_c->row) / (next_c->col - c->col)) < MIN_SLOPE ||
          c->row - next_c->row > MAX_ROW_SKIP)
        c->noisy++;
    }
    // toss points that are marked noisy by both forward and backward passes
    for (vector<Centroid>::iterator c = centroids.begin();
         c != centroids.end();)
    {
      if (c->noisy == 2)
        c = centroids.erase(c);
      else
        ++c;
    }
    IplImage *cent_image = cvCreateImage(cvGetSize(image), image->depth, 3);
    cvConvertImage(diff_image, cent_image);
    for (vector<Centroid>::iterator c = centroids.begin();
         c != centroids.end(); ++c)
    {
      if (c->noisy == 0)
      {
        CV_IMAGE_ELEM(cent_image, unsigned char, c->row, 
                      3*((int)c->col)+1) = 255;
        printf("%f %d %f %f %f %f\n", theta, c->row, c->col,
               c->val / 255.0, c->val / 255.0, c->val / 255.0);
      }
      else
        CV_IMAGE_ELEM(cent_image, unsigned char, c->row, 
                      3*((int)c->col)+2) = 255;

    }
    
    cvShowImage("borg", cent_image);
    cvWaitKey(10);
    cvReleaseImage(&diff_image);
    cvReleaseImage(&cent_image);
    if (images.size() >= 5)
    {
      images.pop_front();
      cvReleaseImage(&ref_image);
    }
    images.push_back(image);
  }
  while (!images.empty())
  {
    IplImage *image = images.front();
    images.pop_front();
    cvReleaseImage(&image);
  }
  closedir(dir);

  return 0;
}

