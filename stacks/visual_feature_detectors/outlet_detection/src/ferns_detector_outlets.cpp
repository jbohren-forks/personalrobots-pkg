#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <algorithm>
#include <iostream>
#include <vector>

using namespace cv;

#include "outlet_detection/affine_transform.h"
#include "outlet_detection/ferns_detector_outlets.h"


static const char config_filename[] = "outlet_config.yml";
static const char outlet_model_filename[] = "outlet_model.xml";


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


void ferns_l_detector_initialize(PlanarObjectDetector& detector, LDetector& ldetector, const char* outlet_config_path, Mat& object, Size patchSize)
{
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

		GaussianBlur(object, object, Size(blurKSize, blurKSize), sigma, sigma);
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

int ferns_l_detect_outlets(Mat& _image, Mat& object, const char* outlet_config_path, PlanarObjectDetector& detector, LDetector& ldetector, vector<feature_t>& train_features,
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

	GaussianBlur(image, image, Size(blurKSize, blurKSize), sigma, sigma);
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

#if defined(_VERBOSE)
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
#endif //_VERBOSE

	return (int)(holes.size());	

}

//-----------------------
//Ferns detector general implementation
int ferns_detector_load(FernClassifier& detector, const char* outlet_config_path)
{
	char outlet_model[1024];
	sprintf(outlet_model,"%s/%s",outlet_config_path,outlet_model_filename);
	printf("Trying to open model file...");
	FileStorage fs(outlet_model, FileStorage::READ);

	if( fs.isOpened() )
	{
		printf("done\n");
		printf("Reading model file...");
		detector.read(fs.getFirstTopLevelNode());
		printf("Done\n");
		fs.release();
		return 1;
	}
	else
	{
		printf("unable to open\n");
	}
	fs.release();
	return 0;
}

void ferns_detector_initialize(Vector<Point2f>& object_keypoints, FernClassifier& detector, const char* outlet_config_path, Mat& object, Size patchSize,PatchGenerator& gen)
{
	//PatchGenerator gen(0,256,15,true,0.6,1.5,-CV_PI,CV_PI,-CV_PI,CV_PI);

	char outlet_model[1024];
	sprintf(outlet_model,"%s/%s",outlet_config_path,outlet_model_filename);


	//GaussianBlur(object, object, Size(blurKSize, blurKSize), sigma, sigma);


	detector.setVerbose(true);
	Vector< Ptr<Mat> > refimgs = Vector< Ptr<Mat> >();
	Vector<int> labels=Vector<int>();
	for (int i=0;i<(int)object_keypoints.size();i++)
	{
		Mat* mat = new Mat(object);
		refimgs.push_back(mat);
	}

	detector.train(object_keypoints,refimgs,labels,
		0, patchSize.width,FernClassifier::DEFAULT_SIGNATURE_SIZE,100, 11, 10000,0,gen);

	printf("done\n");
	FileStorage fs;
	if( fs.open(outlet_model, FileStorage::WRITE) )
	{
		printf("Writing model file...");
		detector.write(fs, "outlet-detector");
		printf("done\n");
	}

	fs.release();
	refimgs.clear();
}
//-
bool ferns_get_pairs(FernClassifier& detector, Mat& img, const Vector<Point2f>& obj_keypoints, const Vector<Point2f>& img_keypoints, Rect modelROI,
                                      Mat& _H, Vector<Point2f>& corners, Vector<int>* pairs)
{
    int i, j, m = (int)obj_keypoints.size(), n = (int)img_keypoints.size();
    Vector<int> bestMatches(m, -1);
    Vector<float> maxLogProb(m, -FLT_MAX);
    Vector<float> signature;
    Vector<Point2f> fromPt, toPt;
    
    for( i = 0; i < n; i++ )
    {
        Point2f kpt = img_keypoints[i];
       // CV_Assert(0 <= kpt.octave && kpt.octave < (int)img.size());
       // kpt.pt.x /= (float)(1 << kpt.octave);
      //  kpt.pt.y /= (float)(1 << kpt.octave);
        int k = detector(img, kpt, signature);

        if( k >= 0 && (bestMatches[k] < 0 || signature[k] > maxLogProb[k]) )
        {
            maxLogProb[k] = signature[k];
            bestMatches[k] = i;
        }
    }
    
    if(pairs)
        pairs->resize(0);
    
    for( i = 0; i < m; i++ )
        if( bestMatches[i] >= 0 )
        {
            fromPt.push_back(obj_keypoints[i]);
            toPt.push_back(img_keypoints[bestMatches[i]]);
        }
    
    if( fromPt.size() < 4 )
        return false;
    
    Vector<bool> mask;
    _H = findHomography(fromPt, toPt, mask, RANSAC, 10);
    if( _H.data )
    {
        const Mat_<double>& H = _H;
        corners.resize(4);
        for( i = 0; i < 4; i++ )
        {
            Point2f pt((float)(modelROI.x + (i == 0 || i == 3 ? 0 : modelROI.width)),
                       (float)(modelROI.y + (i <= 1 ? 0 : modelROI.height)));
            double w = 1./(H(2,0)*pt.x + H(2,1)*pt.y + H(2,2));
            corners[i] = Point2f((float)((H(0,0)*pt.x + H(0,1)*pt.y + H(0,2))*w),
                                 (float)((H(1,0)*pt.x + H(1,1)*pt.y + H(1,2))*w));
        }
    }
    
    for( i = j = 0; i < m; i++ )
        if( bestMatches[i] >= 0 && mask[j++] )
        {
            pairs->push_back(i);
            pairs->push_back(bestMatches[i]);
        }
    
    
    return _H.data != 0;
}
//-
int ferns_detect_outlets(Mat& _image, Mat& object, const char* outlet_config_path, FernClassifier& detector, Vector<Point2f>& object_keypoints, Vector<Point2f>& image_keypoints, vector<feature_t>& train_features,
                            vector<outlet_t>& holes, const char* output_path, const char* output_filename)
{
	int i;
	char path[1024];

	vector<feature_t> dst_outlet;

	Mat image(_image.rows,_image.cols, CV_8UC1);

	cvtColor(_image,image,CV_BGR2GRAY);

	Vector<Point2f> dst_corners;
	Mat correspond( object.rows + image.rows,image.cols > object.cols ? image.cols : object.cols, CV_8UC3);
	correspond = Scalar(0.);
	Mat part(correspond, Rect(0, 0, object.cols, object.rows));
	cvtColor(object, part, CV_GRAY2BGR);
	part = Mat(correspond, Rect(0, object.rows, image.cols, image.rows));
	cvtColor(image, part, CV_GRAY2BGR);


	Vector<int> pairs;
	Mat H;
	double t = (double)getTickCount();


	std::cout << "Object keypoints: " << object_keypoints.size() << "\n";
	std::cout << "Image keypoints: " << image_keypoints.size() << "\n";
	
	Rect modelROI(0,0,object.rows,object.cols);
	bool found = ferns_get_pairs(detector,image,object_keypoints,image_keypoints,modelROI, H, dst_corners, &pairs);
	t = (double)getTickCount() - t;
	printf("%gms\n", t*1000/getTickFrequency());
	vector<CvPoint> train_points;
	vector<CvPoint> features_points;
	train_points.clear();
	features_points.clear();
	dst_outlet.clear();
	holes.clear();

	if (found)
	{
		for( i = 0; i < (int)pairs.size(); i += 2 )
		{
			train_points.push_back(object_keypoints[pairs[i]]);
			features_points.push_back(image_keypoints[pairs[i+1]]);
		}

		CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
		FindAffineTransform(train_points, features_points, homography);
		MapFeaturesAffine(train_features, dst_outlet, homography);
		//reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);
		cvReleaseMat(&homography);
	}

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
#if defined(_VERBOSE)
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
			line( correspond, object_keypoints[pairs[i]],
				image_keypoints[pairs[i+1]] + Point2f(0,object.rows),
				Scalar(0,255,0) );
		}


		Mat objectColor;
		cvtColor(object, objectColor, CV_GRAY2BGR);
		for( i = 0; i < (int)object_keypoints.size(); i++ )
		{
			circle( objectColor, object_keypoints[i], 2, Scalar(0,0,255), -1 );
			circle( objectColor, object_keypoints[i], 7, Scalar(0,255,0), 1 );
		}
		Mat imageColor;
		cvtColor(image, imageColor, CV_GRAY2BGR);
		for( i = 0; i < (int)image_keypoints.size(); i++ )
		{
			circle( imageColor, image_keypoints[i], 2, Scalar(0,0,255), -1 );
			circle( imageColor, image_keypoints[i], 7, Scalar(0,255,0), 1 );
		}

		sprintf(path,"%s/correspondence/%s",output_path,output_filename);
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
	}
#endif //_VERBOSE

	return (int)(holes.size());	
}
//Ferns detector end
//--------------------