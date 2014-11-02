#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "CalibrationExceptions.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <string>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

const int board_dt = 20;

class Calibrator
{
public:
	Calibrator(int imagesAmount, int boardWidth, int boardHeight) noexcept;

	void execute() noexcept;
	void showImages(Mat intrinsic, Mat distortion);

private:
	Mat& getNextImage() throw (ImageReadError);
	void reinitCaptureFieldWithImagesPath(const string path) noexcept;
	void saveIntrinsicMatrixWithYmlExtension(const string path, const Mat& intrinsicMatrix)
		const noexcept;
	void saveDistortionCoeffsWithYmlExtension(const string path, const Mat& distortionCoeffs)
		const noexcept;
	void showSingleImage(const Mat &image, const Mat &intrinsic, const Mat &distortion);
	int handlePause();

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

	int _imagesAmount;
	int _boardWidth;
	int _boardHeight;
	VideoCapture _capture;
};


#endif /* CALIBRATOR_H_ */
