/*
* Outlets size estimation
* Author: Alexey Latyshev
*/

#ifdef WIN32
#include "cv.h"
#include "highgui.h"
#else
#include "opencv/cv.h"
#include "opencv/highgui.h"
#endif
#include <stdio.h>
#include <ctype.h>
#include "CalculateRealCoordinates.h"
#include "outlet_detector.h"

typedef struct CvBackgroundData
{
	int    count;
	char** filename;
} CvBackgroundData;

static
CvBackgroundData* icvCreateBackgroundData( const char* filename)
{
	CvBackgroundData* data = NULL;

	const char* dir = NULL;
	char full[500];
	char* imgfilename = NULL;
	size_t datasize = 0;
	int    count = 0;
	FILE*  input = NULL;
	char*  tmp   = NULL;
	int    len   = 0;

	assert( filename != NULL );

	dir = strrchr( filename, '\\' );
	if( dir == NULL )
	{
		dir = strrchr( filename, '/' );
	}
	if( dir == NULL )
	{
		imgfilename = &(full[0]);
	}
	else
	{
		strncpy( &(full[0]), filename, (dir - filename + 1) );
		imgfilename = &(full[(dir - filename + 1)]);
	}

	input = fopen( filename, "r" );
	if( input != NULL )
	{
		count = 0;
		datasize = 0;

		/* count */
		while( !feof( input ) )
		{
			*imgfilename = '\0';
			if( !fscanf( input, "%s", imgfilename ))
				break;
			len = strlen( imgfilename );
			if( len > 0 )
			{
				if( (*imgfilename) == '#' ) continue; /* comment */
				count++;
				datasize += sizeof( char ) * (strlen( &(full[0]) ) + 1);
			}
		}
		if( count > 0 )
		{
			//rewind( input );
			fseek( input, 0, SEEK_SET );
			datasize += sizeof( *data ) + sizeof( char* ) * count;
			data = (CvBackgroundData*) cvAlloc( datasize );
			memset( (void*) data, 0, datasize );
			data->count = count;
			data->filename = (char**) (data + 1);
			tmp = (char*) (data->filename + data->count);
			count = 0;
			while( !feof( input ) )
			{
				*imgfilename = '\0';
				if( !fscanf( input, "%s", imgfilename ))
					break;
				len = strlen( imgfilename );
				if( len > 0 )
				{
					if( (*imgfilename) == '#' ) continue; /* comment */
					data->filename[count++] = tmp;
					strcpy( tmp, &(full[0]) );
					tmp += strlen( &(full[0]) ) + 1;
				}
			}
		}
		fclose( input );
	}

	return data;
}


//Load camera params from yaml file
int LoadCameraParams(char* filename, CvMat** intrinsic_matrix, CvMat** distortion_coeffs)
{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==NULL) return 0;

	*intrinsic_matrix = (CvMat*)cvReadByName( fs,0,"camera_matrix");
	*distortion_coeffs = (CvMat*)cvReadByName( fs,0,"distortion_coefficients");

	return 1;
}


int LoadApplicationParams(char* filename,
						  char** IMAGES_LIST_PATH,
						  char** LOG_FILE_PATH,
						  char** CAMERA_PARAMETERS_PATH,
						  char** OUTLETS_COORDINATES_PATH,
						  CvSize* innerCornersCount,
						  float* SQUARE_WIDTH,
					      float* SQUARE_HEIGHT,
						  float* MAX_REPROJECTION_ERROR)
{
	CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
	if (fs==NULL) return 0;

	*SQUARE_WIDTH = cvReadRealByName( fs,0,"SQUARE_WIDTH",0);
	*SQUARE_HEIGHT = cvReadRealByName( fs,0,"SQUARE_HEIGHT",0);

	*MAX_REPROJECTION_ERROR = cvReadRealByName( fs,0,"MAX_REPROJECTION_ERROR",0);


	if (cvReadStringByName( fs,0,"LOG_FILE_PATH"))
	{
		*LOG_FILE_PATH = new char[500];
		strcpy(*LOG_FILE_PATH,cvReadStringByName( fs,0,"LOG_FILE_PATH"));//output directory for images and selected regions path
	}
	else
		return 0;

	if (cvReadStringByName( fs,0,"IMAGES_LIST_PATH"))
	{
		*IMAGES_LIST_PATH = new char[500];
		strcpy(*IMAGES_LIST_PATH,cvReadStringByName( fs,0,"IMAGES_LIST_PATH"));//output directory for images and selected regions path
	}

	if (cvReadStringByName( fs,0,"OUTLETS_COORDINATES_PATH"))
	{
		*OUTLETS_COORDINATES_PATH = new char[500];
		strcpy(*OUTLETS_COORDINATES_PATH,cvReadStringByName( fs,0,"OUTLETS_COORDINATES_PATH"));//output directory for images and selected regions path
	}

	//chessboard inner corners count (width x height)
	int CHESSBOARD_WIDTH=cvReadIntByName( fs,0,"CHESSBOARD_WIDTH");
	int CHESSBOARD_HEIGHT=cvReadIntByName( fs,0,"CHESSBOARD_HEIGHT");
	*innerCornersCount=cvSize(CHESSBOARD_WIDTH,CHESSBOARD_HEIGHT);

	if (cvReadStringByName( fs,0,"CAMERA_PARAMETERS_PATH"))
	{
		*CAMERA_PARAMETERS_PATH = new char[500];
		strcpy(*CAMERA_PARAMETERS_PATH, cvReadStringByName( fs,0,"CAMERA_PARAMETERS_PATH"));//camera parameters filename
	}
	else
		return 0;
	return 1;
}


//Outlets tuple (4 outlets)
int main( int argc, char** argv )
{
	if (argc > 1)
	{
		char* IMAGES_LIST_PATH = 0;
		char* LOG_FILE_PATH = 0;
		char* CAMERA_PARAMETERS_PATH = 0;
		char* OUTLETS_COORDINATES_PATH = 0;
		CvMat* CameraMatrix = 0;
		CvMat* DistortionCoeffs = 0;
		CvSize innerCornersCount;
		float SQUARE_WIDTH;
		float SQUARE_HEIGHT;
		float MAX_REPROJECTION_ERROR=0;
		if (LoadApplicationParams(argv[1],&IMAGES_LIST_PATH,&LOG_FILE_PATH,&CAMERA_PARAMETERS_PATH,&OUTLETS_COORDINATES_PATH,&innerCornersCount,
			&SQUARE_WIDTH,&SQUARE_HEIGHT,&MAX_REPROJECTION_ERROR))
		{
			FILE* log;
			log = fopen(LOG_FILE_PATH,"a");
			fprintf(log,"------------Log started-------------\n");
			printf("------------Log started-------------\n");

			if (LoadCameraParams(CAMERA_PARAMETERS_PATH,&CameraMatrix,&DistortionCoeffs))
			{
				printf("\nCamera parameters loaded successfully\n");
				fprintf(log,"\nCamera parameters loaded successfully\n");
			}
			else
			{
				fprintf(log,"\nUnable to load parameters\n");
				printf("\nUnable to load parameters\n");
				fclose(log);
				return 0;
			}


			CvBackgroundData* data = icvCreateBackgroundData(IMAGES_LIST_PATH);
			if (data==NULL)
			{
				fprintf(log,"\nUnable to load images list\n");
				printf("\nUnable to images list\n");
				fclose(log);
				return 0;
			}
			fclose(log);

			CvPoint2D32f* points = new CvPoint2D32f[2*4];
			IplImage* img;
			CvPoint2D32f* coords;
			vector<outlet_t> outlets;


			CvPoint2D32f* outletcenters = new CvPoint2D32f[4];
			for (int i=0;i<4;i++)
			{
				outletcenters[i].x=0.0f;
				outletcenters[i].y=0.0f;
			}

			CvPoint2D32f* leftholes = new CvPoint2D32f[data->count];
			CvPoint2D32f* rightholes = new CvPoint2D32f[data->count];
			float* cameraDist = new float[data->count];
			float* upDist3D = new float[data->count];
			float* downDist3D = new float[data->count];
			float* upDist = new float[data->count];
			float* downDist = new float[data->count];

			int counter = 0;

			CvPoint3D32f* pointsCameraCoords;
			for (int i=0;i<data->count;i++)
			{
				log = fopen(LOG_FILE_PATH,"a");
				img = cvLoadImage(data->filename[i],1);
				if (img==NULL)
				{
					fprintf(log,"Unable to open image %s\n",data->filename[i]);
					printf("Unable to open image %s\n",data->filename[i]);
				}
				else
				{
                    const int img_width = 2450;
                    int img_height = img->height*img_width/img->width;
                    IplImage* _img = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_8U, 3);
                    cvResize(img, _img);
                    cvReleaseImage(&img);
                    img = _img;
                    
					int ret = detect_outlet_tuple(img, CameraMatrix, 0, outlets, outlet_template_t(), 0, 0);

					
					if (ret==0)
					{
						fprintf(log,"Unable to find tuple on image %s\n",data->filename[i]);
						printf("Unable to find tuple on image %s\n",data->filename[i]);
					}
					else
					{
						for (int i=0;i<4;i++)
						{

							points[2*i].x = (float)(outlets[i].hole1.x);
							points[2*i].y = (float)(outlets[i].hole1.y);
							points[2*i+1].x = (float)(outlets[i].hole2.x);
							points[2*i+1].y = (float)(outlets[i].hole2.y);
						}



						coords = CalculateRealCoordinates(img,CameraMatrix,DistortionCoeffs,innerCornersCount,SQUARE_WIDTH,SQUARE_HEIGHT,points,4*2,MAX_REPROJECTION_ERROR,&pointsCameraCoords);

						if (coords == NULL)
						{
							fprintf(log,"Unable to find chessboard on image %s\n",data->filename[i]);
							printf("Unable to find chessboard on image %s\n",data->filename[i]);
						}
						else
						{
							fprintf(log,"Coordinates calculated successfully on image %s\n",data->filename[i]);
							printf("Coordinates calculated successfully on image %s\n",data->filename[i]);

							for (int i=0;i<4;i++)
							{
								outletcenters[i].x+=coords[2*i].x;
								outletcenters[i].x+=coords[2*i+1].x;
								outletcenters[i].y+=coords[2*i].y;
								outletcenters[i].y+=coords[2*i+1].y;
							}

							leftholes[counter].x = coords[0].x;
							leftholes[counter].y = coords[0].y;
							rightholes[counter].x = coords[1].x;
							rightholes[counter].y = coords[1].y;
							cameraDist[counter] = sqrt((outlets[0].coord_hole1.x - pointsCameraCoords[0].x)*(outlets[0].coord_hole1.x - pointsCameraCoords[0].x)+
								(outlets[0].coord_hole1.y - pointsCameraCoords[0].y)*(outlets[0].coord_hole1.y - pointsCameraCoords[0].y)+
								(outlets[0].coord_hole1.z - pointsCameraCoords[0].z)*(outlets[0].coord_hole1.z - pointsCameraCoords[0].z));
							upDist[counter] = sqrt((coords[0].x-coords[2].x)*(coords[0].x-coords[2].x)+
								(coords[0].y-coords[2].y)*(coords[0].y-coords[2].y));

							downDist[counter] = sqrt((coords[4].x-coords[6].x)*(coords[4].x-coords[6].x)+
								(coords[4].y-coords[6].y)*(coords[4].y-coords[6].y));

							upDist3D[counter] = sqrt((pointsCameraCoords[0].x-pointsCameraCoords[2].x)*(pointsCameraCoords[0].x-pointsCameraCoords[2].x)+
								(pointsCameraCoords[0].y-pointsCameraCoords[2].y)*(pointsCameraCoords[0].y-pointsCameraCoords[2].y)+
								(pointsCameraCoords[0].z-pointsCameraCoords[2].z)*(pointsCameraCoords[0].z-pointsCameraCoords[2].z));

							downDist3D[counter++] = sqrt((pointsCameraCoords[4].x-pointsCameraCoords[6].x)*(pointsCameraCoords[4].x-pointsCameraCoords[6].x)+
								(pointsCameraCoords[4].y-pointsCameraCoords[6].y)*(pointsCameraCoords[4].y-pointsCameraCoords[6].y)+
								(pointsCameraCoords[4].z-pointsCameraCoords[6].z)*(pointsCameraCoords[4].z-pointsCameraCoords[6].z));

							fprintf(log,"First hole 3D coordinates mismatch: %f\n",cameraDist[counter-1]);
							fprintf(log,"Up/Down left holes distance(3D): %f/%f\n",upDist3D[counter-1],downDist3D[counter-1]);
							fprintf(log,"Up/Down left holes distance(2D): %f/%f\n",upDist[counter-1],downDist[counter-1]);

							fprintf(log,"\n");
							printf("First hole 3D coordinates mismatch: %f\n",cameraDist[counter-1]);
							printf("Up/Down left holes distance(3D): %f/%f\n",upDist3D[counter-1],downDist3D[counter-1]);
							printf("Up/Down left holes distance(2D): %f/%f\n",upDist[counter-1],downDist[counter-1]);
							printf("\n");
							delete[] pointsCameraCoords;
						}
						vector<outlet_t>().swap(outlets);
					}
				}
				cvReleaseImage(&img);
				fclose(log);
			}
			//		delete[] refPointsCameraCoords;
			float meanx = 0.0f;
			float meany = 0.0f;
			float meandist = 0.0f;
			float devx =0.0f;
			float devy =0.0f;
			float devdist = 0.0f;
			float meanCameraDist = 0.0f;
			float devCameraDist = 0.0f;
			float meanUpDist3D = 0.0f;
			float meanDownDist3D = 0.0f;
			float devUpDist3D = 0.0f;
			float devDownDist3D = 0.0f;
			float meanUpDist = 0.0f;
			float meanDownDist = 0.0f;
			float devUpDist = 0.0f;
			float devDownDist = 0.0f;


			for (int i=0;i<counter;i++)
			{
				meanx+=leftholes[i].x;
				meany+=leftholes[i].y;
				meandist+=sqrt((leftholes[i].x-rightholes[i].x)*(leftholes[i].x-rightholes[i].x)+(leftholes[i].y-rightholes[i].y)*(leftholes[i].y-rightholes[i].y));
				meanCameraDist+=cameraDist[i];
				meanUpDist3D+=upDist3D[i];
				meanDownDist3D+=downDist3D[i];
				meanUpDist+=upDist[i];
				meanDownDist+=downDist[i];

			}
			meanx/=counter;
			meany/=counter;
			meandist/=counter;
			meanCameraDist/=counter;
			meanUpDist3D/=counter;
			meanDownDist3D/=counter;
			meanUpDist/=counter;
			meanDownDist/=counter;


			for (int i=0;i<counter;i++)
			{
				devx+=(leftholes[i].x-meanx)*(leftholes[i].x-meanx);
				devy+=(leftholes[i].y-meany)*(leftholes[i].y-meany);
				devdist+=(sqrt((leftholes[i].x-rightholes[i].x)*(leftholes[i].x-rightholes[i].x)+(leftholes[i].y-rightholes[i].y)*(leftholes[i].y-rightholes[i].y))-meandist)*
					(sqrt((leftholes[i].x-rightholes[i].x)*(leftholes[i].x-rightholes[i].x)+(leftholes[i].y-rightholes[i].y)*(leftholes[i].y-rightholes[i].y))-meandist);
				devCameraDist+=(cameraDist[i]-meanCameraDist)*(cameraDist[i]-meanCameraDist);
				devUpDist3D+=(upDist3D[i]-meanUpDist3D)*(upDist3D[i]-meanUpDist3D);
				devDownDist3D+=(downDist3D[i]-meanDownDist3D)*(downDist3D[i]-meanDownDist3D);
				devUpDist+=(upDist[i]-meanUpDist)*(upDist[i]-meanUpDist);
				devDownDist+=(downDist[i]-meanDownDist)*(downDist[i]-meanDownDist);

			}
			devx=sqrt(devx/(counter-1));
			devy=sqrt(devy/(counter-1));
			devdist=sqrt(devdist/(counter-1));
			devCameraDist=sqrt(devCameraDist/(counter-1));
			devUpDist3D=sqrt(devUpDist3D/(counter-1));
			devDownDist3D=sqrt(devDownDist3D/(counter-1));
			devUpDist=sqrt(devUpDist/(counter-1));
			devDownDist=sqrt(devDownDist/(counter-1));


			log = fopen(LOG_FILE_PATH,"a");
			fprintf(log,"Total measurments made: %d\nMeanX: %f  |  MeanY: %f\nStandart deviation X: %f |  Standart deviation Y: %f\n\nMean distance: %f  |  Standart deviation distance: %f\n\nMean first hole 3D distance: %f  |  Standart deviation first hole 3D distance: %f\n\n",
				counter,meanx,meany,devx,devy,meandist,devdist,meanCameraDist,devCameraDist);
			fprintf(log,"Mean Up/Down left holes distance(3D): %f/%f\nStandart deviance Up/Down left holes distance(3D): %f/%f\n\n",meanUpDist3D,meanDownDist3D,devUpDist3D,devDownDist3D);
			fprintf(log,"Mean Up/Down left holes distance(2D): %f/%f\nStandart deviance Up/Down left holes distance(2D): %f/%f\n\n",meanUpDist,meanDownDist,devUpDist,devDownDist);

			printf("Total measurments made: %d\nMeanX: %f  |  MeanY: %f\nStandart deviation X: %f  |  Standart deviation Y: %f\n\nMean distance: %f  |  Standart deviation distance: %f\n\nMean first hole 3D distance: %f  |  Standart deviation first hole 3D distance: %f\n\n",
				counter,meanx,meany,devx,devy,meandist,devdist,meanCameraDist,devCameraDist);
			printf("Mean Up/Down left holes distance(3D): %f/%f\nStandart deviance Up/Down left holes distance(3D): %f/%f\n\n",meanUpDist3D,meanDownDist3D,devUpDist3D,devDownDist3D);
			printf("Mean Up/Down left holes distance(2D): %f/%f\nStandart deviance Up/Down left holes distance(2D): %f/%f\n\n",meanUpDist,meanDownDist,devUpDist,devDownDist);

			for (int i=0;i<4;i++)
			{
				outletcenters[i].x/=(counter*2);
				outletcenters[i].y/=(counter*2);
			}

			outlet_template_t* templ = new outlet_template_t(4,outletcenters);
			if (OUTLETS_COORDINATES_PATH)
			{
				templ->save(OUTLETS_COORDINATES_PATH);
			}

		}
	}
	else
	{
		printf("\nUsage: OutletsMeasurement.exe [args.yml]");
	}
	return 0;
}