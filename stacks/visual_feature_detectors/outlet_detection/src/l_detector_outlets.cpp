#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <algorithm>
#include <iostream>
#include <vector>

using namespace cv;

#include "outlet_detection/affine_transform.h"
#include "outlet_detection/l_detector_outlets.h"


static const char config_filename[] = "outlet_config.yml";
static const char outlet_model_filename[] = "outlet_model.xml";
static const int blurKSize = 3;
static const double sigma = 0;

void read_cv_point_by_name(CvFileStorage* fs, CvFileNode* parent, const char* name, CvPoint& pt)
{
	CvFileNode* node = cvGetFileNodeByName(fs, parent, name);
	if (node)
	{
		cvReadRawData(fs, node, &pt, "ii");
	}
	else
	{
		pt.x = -1;
		pt.y = -1;
	}
}


void read_training_base(const char* config_path, char* outlet_filename, 
						vector<feature_t>& train_features)
{
	char config[1024];
	sprintf(config,"%s/%s",config_path,config_filename);
	CvMemStorage* storage = cvCreateMemStorage();

	CvFileStorage* fs = cvOpenFileStorage(config, storage, CV_STORAGE_READ);

	CvFileNode* outlet_node = cvGetFileNodeByName(fs, 0, "outlet");
	const char* str = cvReadStringByName(fs, outlet_node, "outlet filename");
	strcpy(outlet_filename, str);


	CvPoint pt;

	int index = 1;
	char feature_name[10];
	while (1)
	{		
		sprintf(feature_name, "power%d", index++);
		read_cv_point_by_name(fs, outlet_node, feature_name, pt);
		if ((pt.x == -1)&&(pt.y==-1))
			break;
		train_features.push_back(feature_t(pt, 1, 0));
	}

	index = 1;
	while (1)
	{		
		sprintf(feature_name, "ground%d", index++);
		read_cv_point_by_name(fs, outlet_node, feature_name, pt);
		if ((pt.x == -1)&&(pt.y==-1))
			break;
		train_features.push_back(feature_t(pt, 1, 1));
	}
	cvReleaseFileStorage(&fs);

	cvReleaseMemStorage(&storage);
}


void l_detector_initialize(PlanarObjectDetector& detector, LDetector& ldetector, const char* outlet_config_path, Mat& object, Size patchSize)
{
	//char object_filename[1024];
	//sprintf(object_filename,"%s/%s",outlet_config_path,outlet_filename);



	//int blurKSize = 3;
	//double sigma = 0;
	//object = imread( object_filename, CV_LOAD_IMAGE_GRAYSCALE );
	PatchGenerator gen(0,256,15,true,0.6,1.5,-CV_PI,CV_PI,-CV_PI,CV_PI);



	char outlet_model[1024];
	sprintf(outlet_model,"%s/%s",outlet_config_path,outlet_model_filename);
	printf("Trying to open model file...");
	FileStorage fs(outlet_model, FileStorage::READ);

	if( fs.isOpened() )
	{
		printf("done\n");
		printf("Reading model file...");
		detector.read(fs.getFirstTopLevelNode());
		printf("done\n");
		// FileStorage fs2("outlet_model_copy.xml", FileStorage::WRITE);
		// detector.write(fs2, "outlet-detector");
	}
	else
	{
		printf("unable to open\nDetector training...");
		Vector<Mat> objpyr;
		Vector<KeyPoint> objKeypoints;

		//GaussianBlur(object, object, Size(blurKSize, blurKSize), sigma, sigma);
		buildPyramid(object, objpyr, ldetector.nOctaves-1);
		ldetector.setVerbose(true);
		ldetector.getMostStable2D(object, objKeypoints, 100, gen);
		detector.setVerbose(true);

		detector.train(objpyr, objKeypoints, patchSize.width, 100, 11, 10000, ldetector, gen);
		printf("done\n");
		if( fs.open(outlet_model, FileStorage::WRITE) )
		{
			printf("Writing model file...");
			detector.write(fs, "outlet-detector");
			printf("done\n");
		}
		
	}
	fs.release();
}

int l_detect_outlets(Mat& _image, Mat& object, const char* outlet_config_path, PlanarObjectDetector& detector, LDetector& ldetector, vector<feature_t>& train_features,
					 vector<outlet_t>& holes, const char* output_path, const char* output_filename)
{
	int i;
	char path[1024];


	//vector<feature_t> train_features;
	vector<feature_t> dst_outlet;

	//IplImage* _ipl_image = _image.;

	//Mat color_image = _image;
	Mat image(_image.rows,_image.cols, CV_8UC1);


	//image = Scalar(0.);
	cvtColor(_image,image,CV_BGR2GRAY);

	//cvNamedWindow("1");
	//imshow("1",image);
	//waitKey(0);
	//double imgscale = 1;
	//resize(_image, image, Size(), 1./imgscale, 1./imgscale, INTER_CUBIC);
	Vector<Mat> imgpyr;

	//GaussianBlur(image, image, Size(blurKSize, blurKSize), sigma, sigma);
	buildPyramid(image, imgpyr, ldetector.nOctaves-1);

	Vector<KeyPoint> objKeypoints, imgKeypoints;

	Vector<Point2f> dst_corners;
	Mat correspond( object.rows + image.rows,image.cols > object.cols ? image.cols : object.cols, CV_8UC3);
	correspond = Scalar(0.);
	Mat part(correspond, Rect(0, 0, object.cols, object.rows));
	cvtColor(object, part, CV_GRAY2BGR);
	part = Mat(correspond, Rect(0, object.rows, image.cols, image.rows));
	cvtColor(image, part, CV_GRAY2BGR);


	//#if 1    
	Vector<int> pairs;
	Mat H;
	double t = (double)getTickCount();
	objKeypoints = detector.getModelPoints();
	ldetector(imgpyr, imgKeypoints, 3000);

	std::cout << "Object keypoints: " << objKeypoints.size() << "\n";
	std::cout << "Image keypoints: " << imgKeypoints.size() << "\n";
	bool found = detector(imgpyr, imgKeypoints, H, dst_corners, &pairs);
	t = (double)getTickCount() - t;
	printf("%gms\n", t*1000/getTickFrequency());
	vector<CvPoint> train_points;
	vector<CvPoint> features_points;
	train_points.clear();
	features_points.clear();

	for( i = 0; i < (int)pairs.size(); i += 2 )
	{
		train_points.push_back(objKeypoints[pairs[i]].pt);
		features_points.push_back(imgKeypoints[pairs[i+1]].pt);
	}

	CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
	FindAffineTransform(train_points, features_points, homography);
	dst_outlet.clear();
	MapFeaturesAffine(train_features, dst_outlet, homography);
	//reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);
	cvReleaseMat(&homography);

	if ((int)(dst_outlet.size()) > 0)
	{
		holes.clear();
		outlet_t outlet;

		for (i=0;i<(int)dst_outlet.size()/3;i++)
		{
			outlet.hole1 = dst_outlet[2*i].pt;
			outlet.hole2 = dst_outlet[2*i+1].pt;
			outlet.ground_hole =dst_outlet[2*(int)dst_outlet.size()/3+i].pt;
			holes.push_back(outlet);
		}
	}

	//#if defined(_VERBOSE)
	if (output_path)
	{
		if( found )
		{
			for( i = 0; i < 4; i++ )
			{
				Point r1 = dst_corners[i%4];
				Point r2 = dst_corners[(i+1)%4];
				line( correspond, Point(r1.x, r1.y+object.rows),
					Point(r2.x, r2.y+object.rows), Scalar(0,0,255) );
			}
		}

		for( i = 0; i < (int)pairs.size(); i += 2 )
		{
			line( correspond, objKeypoints[pairs[i]].pt,
				imgKeypoints[pairs[i+1]].pt + Point2f(0,object.rows),
				Scalar(0,255,0) );
		}

		//#endif

		// imshow( "Object Correspondence", correspond );
		Mat objectColor;
		cvtColor(object, objectColor, CV_GRAY2BGR);
		for( i = 0; i < (int)objKeypoints.size(); i++ )
		{
			circle( objectColor, objKeypoints[i].pt, 2, Scalar(0,0,255), -1 );
			circle( objectColor, objKeypoints[i].pt, (1 << objKeypoints[i].octave)*15, Scalar(0,255,0), 1 );
		}
		Mat imageColor;
		cvtColor(image, imageColor, CV_GRAY2BGR);
		for( i = 0; i < (int)imgKeypoints.size(); i++ )
		{
			circle( imageColor, imgKeypoints[i].pt, 2, Scalar(0,0,255), -1 );
			circle( imageColor, imgKeypoints[i].pt, (1 << imgKeypoints[i].octave)*15, Scalar(0,255,0), 1 );
		}

		sprintf(path,"%s/correspondence/%s",output_path,output_filename);
		//sprintf(command,"mkdir %s",path);
		//system(path);
		imwrite(path, correspond );

		sprintf(path,"%s/features/%s",output_path,output_filename);
		imwrite(path, imageColor );




		Mat resImg = _image;
		for(int i = 0; i < (int)dst_outlet.size(); i++)
		{
			Scalar pointColor = dst_outlet[i].class_id == 0 ? Scalar(0,255,50) : Scalar(255,0,50);	
			line(resImg, Point((int)(dst_outlet[i].pt.x+7), (int)(dst_outlet[i].pt.y)), Point((int)(dst_outlet[i].pt.x-7),(int)(dst_outlet[i].pt.y)),pointColor,2); 
			line(resImg, Point((int)(dst_outlet[i].pt.x), (int)(dst_outlet[i].pt.y+7)), Point((int)(dst_outlet[i].pt.x), (int)(dst_outlet[i].pt.y-7)),pointColor,2); 


		}
		sprintf(path,"%s/outlets/%s",output_path,output_filename);

		imwrite(path,resImg );
		//resImg.release();
	}
	//#endif //_VERBOSE

	return (int)(holes.size());	

}