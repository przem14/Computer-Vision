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
      Mat intrinsic,
      Mat distortion);

int findAllCorners(
      Mat image,
      int board_n,
      Size board_sz,
      VideoCapture capture,
      vector<vector<Point2f> > &image_points,
      vector<vector<Point3f> > &object_points);

void findCornersOnBoard(
      Mat image,
      Mat gray_image,
      Size board_sz,
      vector<Point2f> corners,
      int &successes,
      int board_n,
      vector<vector<Point2f> > &image_points,
      vector<vector<Point3f> > &object_points);

// ------------------------------------------ //

void showImages(
      Mat intrinsic,
      Mat distortion)
{
   Mat image;
   VideoCapture capture(0);
   capture >> image;
   namedWindow("Calibration");
   namedWindow("Undistort");
	int c = 0;
   while(!image.empty() && c != 27)
   {
      Mat t = image.clone();
      imshow("Calibration", image);
      undistort(t, image, intrinsic, distortion);
      imshow("Undistort", image);
      //Handle pause/unpause
      c = waitKey(15);
      if (c == 'p')
      {
         c = 0;
         while (c != 'p' && c != 27)
            c = waitKey(250);
      }
      capture >> image;
   }
}

void findCornersOnBoard(
      Mat image,
      Mat gray_image,
      Size board_sz,
      vector<Point2f> corners,
      int &successes,
      int board_n,
      vector<vector<Point2f> > &image_points,
      vector<vector<Point3f> > &object_points)
{
	//Find chessboard corners:
   bool found = findChessboardCorners(
                   image,
                   board_sz,
                   corners,
                   CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

   if (found)
   {
      //Get Subpixel accuracy on those corners
      cvtColor(image, gray_image, CV_BGR2GRAY);
      cornerSubPix(
         gray_image,
         corners,
         Size(11,11),
         Size(-1,-1),
         TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

      //Draw it
      drawChessboardCorners(
         image,
         board_sz,
         corners,
         found);
   }
   imshow("Calibration", image);
   waitKey(33);

   // If we got a good board, add it to our data
   if (corners.size() == board_n)
   {
      for(int j = 0; j < board_n; ++j)
      {
         image_points[successes][j]    = corners[j];
         object_points[successes][j].x = j / board_w;
         object_points[successes][j].y = j % board_w;
         object_points[successes][j].z = 0.0f;
      }
      successes++;
   }
}

int findAllCorners(
      Mat image,
      int board_n,
      Size board_sz,
      VideoCapture capture,
      vector<vector<Point2f> > &image_points,
      vector<vector<Point3f> > &object_points)
{
   vector<Point2f> corners;
	Mat gray_image(image.size(), CV_8UC1);
	int successes = 0, frame = 0;
	while(successes < n_boards)
   {
      //Skip every board_dt frames to allow user to move chessboard
      if(frame++ % board_dt == 0)
      {
         cout << "Successes: " << successes << "\n";
			findCornersOnBoard(
               image,
               gray_image,
               board_sz,
               corners,
               successes,
               board_n,
               image_points,
               object_points);
      }
      //Handle pause/unpause and ESC int c = cvWaitKey(15);
      int c = -1;
      if(c == 'p')
      {
         c = 0;
         while(c != 'p' && c != 27)
               c = waitKey(250);
      }
      capture >> image; //Get next image
   } //END COLLECTION WHILE LOOP.
	return successes;
}

int main(int argc, char* argv[])
{
   if(argc != 4)
   {
      cerr << "ERROR: Wrong number of input parameters\n";
      return -1;
   }

   board_w = atoi(argv[1]);
   board_h = atoi(argv[2]);
   n_boards = atoi(argv[3]);

   int board_n = board_w * board_h;
   Size board_sz = Size(board_w, board_h);

   VideoCapture capture(0);

   vector<vector<Point2f> >  image_points(n_boards, vector<Point2f>(board_n));
   vector<vector<Point3f> > object_points(n_boards, vector<Point3f>(board_n));
   Mat intrinsic_matrix(3, 3, CV_32FC1);
   Mat distortion_coeffs(5, 1, CV_32FC1);

   Mat image;
   capture >> image;

   // CAPTURE CORNER VIEWS LOOP UNTIL WE'VE GOT n_boards //

	int successes = findAllCorners(
                        image,
                        board_n,
                        board_sz,
                        capture,
                        image_points,
                        object_points);

   //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
   vector<vector<Point2f> >  image_points2(successes, vector<Point2f>(board_n));
   vector<vector<Point3f> > object_points2(successes, vector<Point3f>(board_n));

   for(int i = 0; i < successes; ++i)
      for(int k = 0; k < board_n; k++)
      {
         image_points2[i][k]  = image_points[i][k];
         object_points2[i][k] = object_points[i][k];
      }

   // At this point we have all of the chessboard corners we need.
   // Initialize the intrinsic matrix such that the two focal
   // lengths have a ratio of 1.0

   intrinsic_matrix.at<float>(0,0) = 1.0f;
   intrinsic_matrix.at<float>(1,1) = 1.0f;

   vector<Mat> rot;
   vector<Mat> trans;

   //CALIBRATE THE CAMERA!
   calibrateCamera(
         object_points,
         image_points,
         image.size(),
         intrinsic_matrix,
         distortion_coeffs,
         rot,
         trans);

   // SAVE THE INTRINSICS AND DISTORTIONS
   FileStorage fs_write("Calibration_params.xml", FileStorage::WRITE);
   fs_write << "intr" << intrinsic_matrix << "dist" << distortion_coeffs;
   fs_write.release();

   // EXAMPLE OF LOADING THESE MATRICES BACK IN:
   Mat intrinsic, distortion;
   FileStorage fs_read("Calibration_params.xml", FileStorage::READ);
   fs_read["intr"] >> intrinsic;
   fs_read["dist"] >> distortion;
   fs_read.release();

   // Build the undistort map that we will use for all
   // subsequent frames.
   //
   //Mat mapx(image.size(), CV_32FC1);
   //Mat mapy(image.size(), CV_32FC1);
   //cvInitUndistortMap(intrinsic_matrix, distortion_coeffs, mapx, mapy);

   // Just run the camera to the screen, now showing the raw and
   // the undistorted image.

   showImages(intrinsic, distortion);

   return 0;
}
