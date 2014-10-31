// Calling convention:
// calibration board_w board_h number_of_views //
// Hit 'p' to pause/unpause, ESC to quit //

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace cv;
using namespace std;

int n_boards = 0; //Will be set by input list
const int board_dt = 20; //Wait 20 frames per chessboard view int board_w;
int board_h;
int board_w;

void showImages(
      IplImage *image,
      IplImage* mapx,
      IplImage* mapy,
      CvCapture* capture);

int findAllCorners(
      IplImage *image,
      int board_n,
      CvSize board_sz,
      CvCapture* capture,
      CvMat* image_points,
      CvMat* object_points,
      CvMat* point_counts);

void findCornersOnBoard(
      IplImage *image,
      IplImage *gray_image,
      CvSize board_sz,
      CvPoint2D32f* corners,
      int &successes,
      int &corner_count,
      int board_n,
      CvMat* image_points,
      CvMat* object_points,
      CvMat* point_counts);

// ------------------------------------------ //

void showImages(
      IplImage *image,
      IplImage* mapx,
      IplImage* mapy,
      CvCapture* capture)
{
	cvNamedWindow("Calibration");
    cvNamedWindow("Undistort");
	int c = 0;
    while(image && c != 27)
    {
        IplImage *t = cvCloneImage(image);
        cvShowImage("Calibration", image);
        cvRemap(t, image, mapx, mapy);
        cvReleaseImage(&t);
        cvShowImage("Undistort", image);

        //Handle pause/unpause
        c = cvWaitKey(15);
        if (c == 'p')
        {
            c = 0;
            while (c != 'p' && c != 27)
				c = cvWaitKey(250);
        }

        image = cvQueryFrame(capture);
    }
}

void findCornersOnBoard(
      IplImage *image,
      IplImage *gray_image,
      CvSize board_sz,
      CvPoint2D32f* corners,
      int &successes,
      int &corner_count,
      int board_n,
      CvMat* image_points,
      CvMat* object_points,
      CvMat* point_counts)
{
	//Find chessboard corners:
    int found = cvFindChessboardCorners(
        image,
        board_sz,
        corners,
        &corner_count,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    //Get Subpixel accuracy on those corners
    cvCvtColor(image, gray_image, CV_BGR2GRAY);
    cvFindCornerSubPix(
        gray_image,
        corners,
        corner_count,
        cvSize(11,11),
        cvSize(-1,-1),
        cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

    //Draw it
			/*
    cvDrawChessboardCorners(
        image,
        board_sz,
        corners,
        corner_count,
        found);
			*/
    //cvShowImage("Calibration", image);

    // If we got a good board, add it to our data
    if (corner_count == board_n)
    {
		int step = successes*board_n;
		for (int i=step, j=0; j<board_n; ++i,++j)
		{
			CV_MAT_ELEM(*image_points, float, i, 0) = corners[j].x;
			CV_MAT_ELEM(*image_points, float, i, 1) = corners[j].y;
			CV_MAT_ELEM(*object_points,float, i, 0) = j/board_w;
			CV_MAT_ELEM(*object_points,float, i, 1) = j%board_w;
			CV_MAT_ELEM(*object_points,float, i, 2) = 0.0f;
		}
		CV_MAT_ELEM(*point_counts, int, successes,0) = board_n;
		successes++;
    }
}

int findAllCorners(
      IplImage *image,
      int board_n,
      CvSize board_sz,
      CvCapture* capture,
      CvMat* image_points,
      CvMat* object_points,
      CvMat* point_counts)
{
	int corner_count;
	CvPoint2D32f* corners = new CvPoint2D32f[board_n];
	IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);
	int successes = 0, frame = 0;
	while(successes < n_boards)
    {
        //Skip every board_dt frames to allow user to move chessboard
        if(frame++ % board_dt == 0)
        {
			printf("%i", successes);
			findCornersOnBoard(image, gray_image, board_sz, corners,
							successes, corner_count, board_n,
							image_points, object_points, point_counts);
        }
        int c = -1;
        //Handle pause/unpause and ESC int c = cvWaitKey(15);
        if(c == 'p')
        {
            c = 0;
            while(c != 'p' && c != 27) c = cvWaitKey(250);
        }

        image = cvQueryFrame(capture); //Get next image
    } //END COLLECTION WHILE LOOP.
	return successes;
}

int main(int argc, char* argv[])
{
    if(argc != 4)
    {
        printf("ERROR: Wrong number of input parameters\n");
        return -1;
    }

    board_w = atoi(argv[1]);
    board_h = atoi(argv[2]);
    n_boards = atoi(argv[3]);

    int board_n = board_w * board_h;
    CvSize board_sz = cvSize(board_w, board_h);

    CvCapture* capture = cvCreateCameraCapture(0);
    assert(capture);

    CvMat* image_points         = cvCreateMat(n_boards*board_n, 2, CV_32FC1);
    CvMat* object_points        = cvCreateMat(n_boards*board_n, 3, CV_32FC1);
    CvMat* point_counts         = cvCreateMat(n_boards, 1, CV_32SC1);
    CvMat* intrinsic_matrix     = cvCreateMat(3, 3, CV_32FC1);
    CvMat* distortion_coeffs    = cvCreateMat(5, 1, CV_32FC1);


    //int corner_count;

    IplImage *image = cvQueryFrame(capture);

    // CAPTURE CORNER VIEWS LOOP UNTIL WE'VE GOT n_boards
    // SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND) //

	int successes = findAllCorners(image, board_n, board_sz, capture, image_points, object_points, point_counts);

    //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
    CvMat* object_points2 = cvCreateMat(successes*board_n,3,CV_32FC1);
    CvMat* image_points2 = cvCreateMat(successes*board_n,2,CV_32FC1);
    CvMat* point_counts2 = cvCreateMat(successes,1,CV_32SC1); //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES

    //Below, we write out the details*board_n in the next two loops. We could
    //instead have written:
    //image_points->rows = object_points->rows = successes*board_n;
    //point_counts->rows = successes;

    for(int i = 0; i<successes*board_n; ++i)
    {
        CV_MAT_ELEM(*image_points2,  float, i, 0) = CV_MAT_ELEM( *image_points,  float, i, 0);
        CV_MAT_ELEM(*image_points2,  float, i, 1) = CV_MAT_ELEM( *image_points,  float, i, 1);
        CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0);
        CV_MAT_ELEM(*object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1);
        CV_MAT_ELEM(*object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2);
    }
    for(int i=0; i<successes; ++i)
    {
        //These are all the same number
        CV_MAT_ELEM( *point_counts2, int, i, 0) = CV_MAT_ELEM( *point_counts, int, i, 0);
    }

    cvReleaseMat(&object_points);
    cvReleaseMat(&image_points);
    cvReleaseMat(&point_counts);

    // At this point we have all of the chessboard corners we need.
    // Initialize the intrinsic matrix such that the two focal
    // lengths have a ratio of 1.0
    //
    CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
    CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

    //CALIBRATE THE CAMERA!
    cvCalibrateCamera2(
            object_points2,
            image_points2,
            point_counts2,
            cvGetSize(image),
            intrinsic_matrix,
            distortion_coeffs,
            NULL,
            NULL,
            0); //CV_CALIB_FIX_ASPECT_RATIO

    // SAVE THE INTRINSICS AND DISTORTIONS
    cvSave("Intrinsics.xml",intrinsic_matrix);
    cvSave("Distortion.xml",distortion_coeffs);

    // EXAMPLE OF LOADING THESE MATRICES BACK IN:
    //CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
    //CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");

    // Build the undistort map that we will use for all
    // subsequent frames.
    //
    IplImage* mapx = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage* mapy = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    cvInitUndistortMap(intrinsic_matrix, distortion_coeffs, mapx, mapy);

    // Just run the camera to the screen, now showing the raw and
    // the undistorted image.
    //
	showImages(image, mapx, mapy, capture);

    return 0;
}
