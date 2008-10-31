// This file is copied over and modified from Hai's scratch project of cloud_transform

#include "cloud_transform.h"
#include <exception>
#include <ctime>
#include "CvStereoCamModel.h"

#define IMG_DEPTH uchar
const static int OCC_FILLED = 1;
const static int OCC_EMPTY  = 0;
const static bool VERBOSE   = true;
const static bool DISPLAY   = true;

using namespace std;

class NoPointsToAverage: public exception
{
  virtual const char* what() const throw()
  {
    return "Error: function asked for the average of 0 points.";
  }
} average_exception;

laser_scan read_lrf_dat_file(const char *filename)
{
    ifstream fin(filename);
    if (fin.good() == false) {
    	cerr << "Cannot open dat file: "<< filename << endl;
    	laser_scan ls;
    	ls.points = NULL;
    	ls.intensity = NULL;
    }
    string s;
    vector<CvPoint3D32f> *points    = new vector<CvPoint3D32f>();
    vector<double>       *intensity = new vector<double>();

    while(getline(fin, s))
    {
        if (s[0] != '#')
        {
            istringstream ss(s);
            double x, y, z, i;
            ss >> x >> y >> z >> i;
            points->push_back(cvPoint3D32f(x,y,z));
            intensity->push_back(i);
        } else
        {
            if (VERBOSE)
            {
                printf("Ignored comment...");
                cout << s << endl;
            }
        }
    }
    laser_scan ret;
    ret.points = points;
    ret.intensity = intensity;
    return ret;
}

bool not_in_list(int xi, int yi, vector<CvPoint> &neighbors)
{
    for (uint i = 0; i < neighbors.size(); i++)
    {
        if (neighbors[i].x == xi && neighbors[i].y == yi)
            return false;
    }
    return true;
}

CvPoint3D32f average_3d_points(vector<CvPoint3D32f> points3D)
{
    if (points3D.size() == 0)
        throw average_exception;

    CvPoint3D32f p = cvPoint3D32f(0.0, 0.0, 0.0);
    for (unsigned int i = 0; i < points3D.size(); i++)
    {
        CvPoint3D32f current_point = points3D.at(i);
        p.x += current_point.x;
        p.y += current_point.y;
        p.z += current_point.z;
    }

    p.x = p.x / (float) points3D.size();
    p.y = p.y / (float) points3D.size();
    p.z = p.z / (float) points3D.size();
    return p;
}

bool fill_missing_pixel(int x, int y, IplImage *original_d16_c1, IplImage *occupancy_d8_c1)
{
    int image_width  = original_d16_c1->width;
    int image_height = original_d16_c1->height;

    //Search for neighbors
    vector<CvPoint> neighbors;

    //top
    int xt = x;
    int yt = max(0, min(image_height, y+1));
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));

    //bottom
    xt = x;
    yt = max(0, min(image_height, y-1));
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));

    //left
    xt = max(0, min(image_width, x+1));
    yt = y;
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));


    //right
    xt = max(0, min(image_width, x-1));
    yt = y;
    if (OCC_FILLED == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yt))[xt])
        neighbors.push_back(cvPoint(xt,yt));

    //Check if we can use edge sensing
    bool         left_neighbor,  right_neighbor,  top_neighbor,  bottom_neighbor;
    float        left_v,         right_v,         top_v,         bottom_v;
    CvPoint      left_neighborp, right_neighborp, top_neighborp, bottom_neighborp;
    //CvPoint3D32f left_loc,       right_loc,       top_loc,       bottom_loc;

    left_neighbor = right_neighbor = top_neighbor = bottom_neighbor = false;
    for (unsigned int i = 0; i < neighbors.size(); i++)
    {
        if (neighbors.at(i).x < x && neighbors.at(i).y == y)
        {
            left_neighbor  = true;
            left_neighborp = neighbors.at(i);
            left_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*left_neighborp.y))[left_neighborp.x];
        }
        if (neighbors.at(i).x > x && neighbors.at(i).y == y)
        {
            right_neighbor  = true;
            right_neighborp = neighbors.at(i);
            right_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*right_neighborp.y))[right_neighborp.x];
        }
        if (neighbors.at(i).y < y && neighbors.at(i).x == x)
        {
            top_neighbor  = true;
            top_neighborp = neighbors.at(i);
            top_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*top_neighborp.y))[top_neighborp.x];
        }
        if (neighbors.at(i).y > y && neighbors.at(i).x == x)
        {
            bottom_neighbor  = true;
            bottom_neighborp = neighbors.at(i);
            bottom_v         = ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*bottom_neighborp.y))[bottom_neighborp.x];
        }
    }

    if (left_neighbor && right_neighbor && top_neighbor && bottom_neighbor)
    {
        //Edge sensing
        //http://scien.stanford.edu/class/psych221/projects/99/tingchen/algodep/edgesense.html
        float horizontal_gradient = (left_v + right_v) / 2.0;
        float vertical_gradient   = (top_v + bottom_v) / 2.0;
        float threshold = (horizontal_gradient + vertical_gradient) / 2.0;
        if (horizontal_gradient < threshold && vertical_gradient > threshold)
        {
            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
                (IMG_DEPTH) lrint((left_v + right_v) / 2.0);
        } else if (horizontal_gradient > threshold && vertical_gradient < threshold)
        {
            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
                (IMG_DEPTH) lrint((top_v + bottom_v) / 2.0);
        } else {
            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
                (IMG_DEPTH) lrint((top_v + bottom_v + left_v + right_v) / 4.0);
        }
        return true;
    } else if (left_neighbor && right_neighbor)
    {
        ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] =
            (IMG_DEPTH) lrint((left_v + right_v) / 2.0);
        return true;
    } else if (top_neighbor && bottom_neighbor)
    {
        ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] = (IMG_DEPTH) lrint((top_v + bottom_v) / 2.0);
        return true;
    } else {
        if (neighbors.size() > 1)
        {
            //Calculcate weights
            vector<float> weights;
            double sum_intensity    = 0;
            double sum_weights      = 0;
            float size              = std::pow(.5f, 2);

            for (unsigned int i = 0; i < neighbors.size(); i++)
            {
                CvPoint p = neighbors[i];
                float weight = exp(-(std::pow((float)(p.x-x), 2) + pow((float)(p.y-y), 2)) / size);
                weight = max((float).01, weight);
                sum_intensity += weight * ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*p.y))[p.x];
                sum_weights   += weight;

            }

            ((IMG_DEPTH*)(original_d16_c1->imageData + original_d16_c1->widthStep*y))[x] = (IMG_DEPTH) (sum_intensity / sum_weights);
            return true;
        } else {
            return false;
        }
    }
    return false;
}

void fill_missing_pixels(IplImage *original_d16_c1, IplImage *occupancy_d8_c1)
{
    int count = 0;
    bool making_progress = true;
    vector<CvPoint> *failed  = new vector<CvPoint>();
    vector<CvPoint> *failed2 = new vector<CvPoint>();
    for (int yi=0; yi < original_d16_c1->height; yi++)
    for (int xi=0; xi < original_d16_c1->width;  xi++)
    {
        if (OCC_EMPTY == ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*yi))[xi])
            failed->push_back(cvPoint(xi,yi));
    }

    while (making_progress)
    {
        making_progress = false;
        vector<CvPoint>::iterator iter;
        vector<CvPoint> succeeded;
        for (iter = failed->begin(); iter != failed->end(); iter++)
        {
            CvPoint p = *iter;
            if (fill_missing_pixel(p.x, p.y, original_d16_c1, occupancy_d8_c1))
            {
                making_progress = true;
                succeeded.push_back(cvPoint(p.x, p.y));
            } else {
                failed2->push_back(cvPoint(p.x, p.y));
            }

            if (DISPLAY && count % 10000 == 0)
            {
            	cvShowImage("img", original_d16_c1);
            	cvWaitKey(5);
            }
            count++;
        }
        //printf("found %d pixels to fill\n", succeeded.size());

        //Mark pixels as occupied here instead of above, as results won't be affected by
        //update order of fill-in sweeps
        for (iter = succeeded.begin(); iter != succeeded.end(); iter++)
        {
            CvPoint p = *iter;
            ((char*)(occupancy_d8_c1->imageData + occupancy_d8_c1->widthStep*p.y))[p.x] = OCC_FILLED;
        }

        delete failed;
        failed = failed2;
        failed2 = new vector<CvPoint>();
    }

    delete failed;
    delete failed2;
}

// transform the points so that x to the right, y is down, and z forward.
CvPoint3D32f transform_to_cv_frame(CvPoint3D32f pt)
{
#if 0 // assume that x forward, z points up, and y to the left, aka ROS/Vedere(?)
    double x = -pt.y;
    double y = -pt.z;
    double z =  pt.x;
#else // now x is down, y to the left, z forward (WG default Hokuyo pose as of 08-08-08)
    double x = -pt.y;
    double y =  pt.x;
    double z =  pt.z;
#endif

    return cvPoint3D32f(x, y, z);
}

CvStereoCamModel StereoCam(729., 729., 4.381214e+004/729.,  3.239809e+002, 3.239809e+002, 2.478744e+002);

CvPoint transform_to_screen1(CvPoint3D32f pt, int image_width, int image_height){
	double _xyz[3];
	double _xyz1[3];
	double _uvd[3];
	CvMat xyz, xyz1;
	CvMat uvd;
	_xyz[0] = pt.x*1000.;
	_xyz[1] = pt.y*1000.;
	_xyz[2] = pt.z*1000.;
	cvInitMatHeader(&xyz, 1, 3, CV_64F, _xyz);
	cvInitMatHeader(&xyz1, 1, 3, CV_64F, _xyz1);
	cvInitMatHeader(&uvd, 1, 3, CV_64F, _uvd);

	// a rotation matrix to turn from x forward, z up to
	// z forward and y down
	double _X2Z[9] = {
			0, -1,  0,
			0,  0, -1,
			1,  0,  0
	};
	CvMat X2Z = cvMat(3, 3, CV_64F, _X2Z);

	cvGEMM(&xyz, &X2Z, 1.0, NULL, 0, &xyz1, CV_GEMM_B_T);

	StereoCam.cartToDisp(xyz1, uvd);

	int x = lrint(_uvd[0]);
	int y = lrint(_uvd[1]);
	return cvPoint(x, y);
}

CvPoint transform_to_screen(CvPoint3D32f pt, int image_width, int image_height)
{
#if 1
    double focal_length_x = 320;
    double focal_length_y = 320;
    double camera_center_x = (float) image_width / 2.0;
    double camera_center_y = (float) image_height / 2.0;
#else
    double focal_length_x = 320.;
    double focal_length_y = 320.;
    double camera_center_x = (float) 323.9809;
    double camera_center_y = (float) 247.8744;
#endif

    int x = lrint(((focal_length_x * pt.x) / pt.z) + camera_center_x);
    int y = lrint(((focal_length_y * pt.y) / pt.z) + camera_center_y);
    return cvPoint(x,y);
}

bool in_bounds(CvPoint pt, int image_width, int image_height)
{
    if ((0 <= pt.x) && (pt.x < image_width) && (0 <= pt.y) && (pt.y < image_height))
        return true;
    else
        return false;
}

uint int2uchar(uint n)
{
    return (uint) lrint(((double) n) / 25.0);
}

unsigned char double2uchar(double n, double maxx, double minn)
{
    return (uchar) lrint(255.0 * ((n - minn) / maxx));
}

double magnitude(CvPoint3D32f p)
{
  return std::pow(std::pow((double)p.x, 2.0) + std::pow((double)p.y, 2.0) + std::pow((double)p.z, 2.0), 0.5);
}

synthetic_image image_from_point_cloud(laser_scan &ls, int image_width, int image_height)
{
    if (DISPLAY)
    {
        cvNamedWindow("img", 1);
    }
    //Output image
    IplImage *img       = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_8U, 1);
    //Whether the corresponding pixel in 'img' is filled
    IplImage *occupancy = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_8U, 1);
    cvSet(occupancy, cvScalar(OCC_EMPTY));

    ////////////////////////////////////////////////////////////////////////////////////////
    // Project 3D xyz points into camera uv points and plot associated intensity in image
    ////////////////////////////////////////////////////////////////////////////////////////
    //Turn 3D points into image points
    IplImage *sums = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_64F, 1);
    cvSet(sums, cvScalar(0));
    IplImage *hits = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_32S, 1);
    cvSet(hits, cvScalar(0));
    synthetic_image syn_image;

    int out_bound_pixels = 0;
    int behind_camera    = 0;

    int max_hits = 0;
    unsigned int max_sum = 0;
    if (VERBOSE)
        cout << "reading scans...\n";
    //For each 3D point
    if (VERBOSE)
        cout << "looping over points " << ls.points->size() << "...\n";
    for (unsigned int i = 0; i < ls.points->size(); i++)
    {
        CvPoint3D32f scan_point = ls.points->at(i);
        CvPoint3D32f cvpt = transform_to_cv_frame(scan_point);
        if (cvpt.z > 0 && magnitude(scan_point) > 0.08)
        {
            CvPoint screen_pt = transform_to_screen(cvpt, image_width, image_height);
            if (in_bounds(screen_pt, image_width, image_height))
            {
                unsigned int associated_intensity = ls.intensity->at(i);

                //Add to sums
                if (associated_intensity > 0)
                {
                    ((char*)         (occupancy->imageData + occupancy->widthStep*screen_pt.y))[screen_pt.x] = OCC_FILLED;
                    double current_intensity = ((double*)(sums->imageData + sums->widthStep*screen_pt.y))[screen_pt.x];
                    ((double *)      (sums->imageData + sums->widthStep*screen_pt.y))          [screen_pt.x] = current_intensity + associated_intensity;
                    ((unsigned int *)(hits->imageData + hits->widthStep*screen_pt.y))          [screen_pt.x] += 1;

                    //double current_x = ((double *)(loc3D->imageData + loc3D->widthStep*screen_pt.y))[screen_pt.x*3];
                    //double current_y = ((double *)(loc3D->imageData + loc3D->widthStep*screen_pt.y))[screen_pt.x*3+1];
                    //double current_z = ((double *)(loc3D->imageData + loc3D->widthStep*screen_pt.y))[screen_pt.x*3+2];

                    //((double *) (loc3D->imageData + loc3D->widthStep*screen_pt.y))[screen_pt.x*3]   = current_x + scan_point.x;
                    //((double *) (loc3D->imageData + loc3D->widthStep*screen_pt.y))[screen_pt.x*3+1] = current_y + scan_point.y;
                    //((double *) (loc3D->imageData + loc3D->widthStep*screen_pt.y))[screen_pt.x*3+2] = current_z + scan_point.z;
                    int cur_hits = ((unsigned int*)(hits->imageData + hits->widthStep*screen_pt.y))[screen_pt.x];
                    if (max_hits < cur_hits)
                    {
                        max_hits = cur_hits;
                        max_sum = max_hits;
                    }

                    mapped_points p;
                    p.points2d = screen_pt;
                    p.points3d = scan_point;
                    syn_image.points.push_back(p);
                }

            } else
            {
                out_bound_pixels++;
            }
        } else
            behind_camera++;
    }

    if (VERBOSE)
    {
        cout << "out_bound_pixels: " << out_bound_pixels << endl;
        cout << "behind_camera: "    << behind_camera << endl;
        cout << "max hits: "    << max_hits << endl;
        cout << "averging hits...\n";
    }

    //Average hits in each cell and set to new image
    for (int y = 0; y < sums->height; y++)
    for (int x = 0; x < sums->width;  x++)
    {
        double current_intensity       = ((double*)(sums->imageData + sums->widthStep*y))[x];
        unsigned int num_hits          = ((unsigned int*)(hits->imageData + hits->widthStep*y))[x];
        if (current_intensity != 0)
            ((IMG_DEPTH*) (img->imageData + img->widthStep*y))[x] =
                int2uchar(lrint((double) current_intensity / (double) num_hits));
        else
            ((IMG_DEPTH*) (img->imageData + img->widthStep*y))[x] =  0;
    }

    if (DISPLAY)
    {
        //Show image before filling it in
        cvShowImage("img", img);
        printf("Waiting key...\n");
        cvWaitKey(33);
    }

    //Loop over unfilled pixels & set them to average of 4 neighbors
    fill_missing_pixels(img, occupancy);

    if (DISPLAY)
        cvShowImage("img", img);

    syn_image.image = img;
    return syn_image;
}

#if 0
int main(int argc, char **argv)
{
    laser_scan ls = read_lrf_dat_file(argv[1]);
    synthetic_image si = image_from_point_cloud(ls, 640, 480);
    printf("Number of 3D points used in image %d out of %d laser scan points, %.2f percent.\n",
            si.points.size(), ls.points->size(), 100.0*si.points.size() / (float) ls.points->size());
    if (DISPLAY)
    {
        printf("Waiting key...\n");
        cvWaitKey(300);
    }
    cvSaveImage(argv[2], si.image);
    //cvSaveImage("/tmp/foo.png", si.points);
}
#endif


