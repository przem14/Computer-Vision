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
	void Calibrator::showChessboardPoints(const Mat &image, const Size &boardSize, 
				const vector<Point2f> &corners, const bool &found);
	int findAllCorners(
	        Mat image,
	        const int &pointsOnBoardAmount,
	        const Size &boardSize,
	        vector<vector<Point2f> > &imagePoints,
	        vector<vector<Point3f> > &objectPoints);
	bool findCornersOnChessboard(const Mat &image, const Size &boardSize, vector<Point2f> &corners);
	void Calibrator::getSubpixelAccuracy(const Mat &image, Mat &grayImage, vector<Point2f> &corners);
	void Calibrator::SaveImagePoints(const int &successes, const int &pointsOnBoardAmount, const vector<Point2f> &corners,
	vector<vector<Point2f> > &imagePoints, vector<vector<Point3f> > &objectPoints);
	Mat Calibrator::CreateGrayImage(const Mat &image);
	void Calibrator::DisplayNumberOfSuccesses(const int &successes);
	void Calibrator::findCornersOnImage(
        const Mat &image,
        Mat &grayImage,
        const Size &boardSize,
        vector<Point2f> &corners,
        int &successes,
        const int &pointsOnBoardAmount,
        vector<vector<Point2f> > &imagePoints,
        vector<vector<Point3f> > &objectPoints);

	int _imagesAmount;
	int _boardWidth;
	int _boardHeight;
	VideoCapture _capture;
};


#endif /* CALIBRATOR_H_ */
