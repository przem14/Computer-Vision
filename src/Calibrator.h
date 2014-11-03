#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "CalibrationExceptions.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>
#include <utility>
#include <initializer_list>

using namespace std;
using namespace cv;

const int board_dt = 20;

class Calibrator
{
public:
	Calibrator(int imagesAmount, int boardWidth, int boardHeight) noexcept;

	void execute() noexcept;

private:
	Mat& getNextImage() throw (ImageReadError);
	void reinitCaptureFieldWithImagesPath(const string path) noexcept;

	void saveIntrinsicMatrixWithYmlExtension(const string path)
        const noexcept;
	void saveDistortionCoeffsWithYmlExtension(const string path)
        const noexcept;
	
	void presentImagesWithTheirsUndistortedCopy();
    void showImageAndItsUndistortedCopy() const noexcept;
    void showImages(const std::initializer_list 
                        <std::pair<const std::string&, const Mat&>> 
                        &imagesWithWindowsNames) const noexcept;
    Mat  createUndistortedImage(const Mat &image) const noexcept;
    void createWindows(const std::initializer_list<const std::string> &names) 
        const noexcept;

	int handlePause() const noexcept;

	void showChessboardPoints(const Mat &image,
                              const Size &boardSize,
                              const vector<Point2f> &corners,
                              const bool &found);

	int findAllCorners(
	        Mat image,
	        const int &pointsOnBoardAmount,
	        const Size &boardSize,
	        vector<vector<Point2f> > &imagePoints,
	        vector<vector<Point3f> > &objectPoints);
	bool findCornersOnChessboard(const Mat &image, const Size &boardSize, vector<Point2f> &corners);
	void getSubpixelAccuracy(const Mat &image, Mat &grayImage, vector<Point2f> &corners);
	void SaveImagePoints(const int &successes, const int &pointsOnBoardAmount, const vector<Point2f> &corners,
	vector<vector<Point2f> > &imagePoints, vector<vector<Point3f> > &objectPoints);
	Mat CreateGrayImage(const Mat &image);
	void DisplayNumberOfSuccesses(const int &successes);
	void findCornersOnImage(
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

    Mat _image;

    Mat _intrinsic;
    Mat _distortion;

    const std::string CALIBRATION_WINDOW_NAME = "Calibration";
    const std::string UNDISTORTED_WINDOW_NAME = "Undistort";

    const char PAUSE_KEY    = 'p';
    const char ESCAPE_KEY   = 27;
    const int  WAITING_TIME = 250;
};


#endif /* CALIBRATOR_H_ */
