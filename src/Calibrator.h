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

using std::vector;

class Calibrator
{
public:
    Calibrator(int imagesAmount, int boardWidth, int boardHeight) noexcept;

    void execute() noexcept;

private:
    cv::Mat& getNextImage() throw (ImageReadError);
    void reinitCaptureFieldWithImagesPath(const std::string &path) noexcept;

    void saveIntrinsicMatrixWithYmlExtension(const std::string &path)
        const noexcept;
    void saveDistortionCoeffsWithYmlExtension(const std::string &path)
        const noexcept;

    void presentImagesWithTheirsUndistortedCopy();

    void showImageAndItsUndistortedCopy() const noexcept;
    void showImages(const std::initializer_list 
                        <std::pair<const std::string&, const cv::Mat&>> 
                        &imagesWithWindowsNames) const noexcept;

    cv::Mat createUndistortedImage(const cv::Mat &image) const noexcept;

    void createWindows(const std::initializer_list<const std::string> &names) 
        const noexcept;

    int handlePause() const noexcept;

    void showChessboardPoints(const cv::Mat &image,
                              const cv::Size &boardSize,
                              const vector<cv::Point2f> &corners,
                              const bool &found);

    int findAllCorners(cv::Mat image,
                       const int &pointsOnBoardAmount,
                       const cv::Size &boardSize,
                       vector<vector<cv::Point2f> > &imagePoints,
                       vector<vector<cv::Point3f> > &objectPoints);
    bool findCornersOnChessboard(const cv::Mat &image, 
                                 const cv::Size &boardSize, 
                                 vector<cv::Point2f> &corners);
    void getSubpixelAccuracy(const cv::Mat &image, 
                             cv::Mat &grayImage, 
                             vector<cv::Point2f> &corners);
    void saveImagePoints(const int &successes, 
                         const int &pointsOnBoardAmount, 
                         const vector<cv::Point2f> &corners,
                         vector<vector<cv::Point2f>> &imagePoints, 
                         vector<vector<cv::Point3f>> &objectPoints);
    cv::Mat createGrayImage(const cv::Mat &image);
    void displayNumberOfSuccesses(const int &successes);
    void findCornersOnImage(const cv::Mat &image,
                            cv::Mat &grayImage,
                            const cv::Size &boardSize,
                            vector<cv::Point2f> &corners,
                            int &successes,
                            const int &pointsOnBoardAmount,
                            vector<vector<cv::Point2f> > &imagePoints,
                            vector<vector<cv::Point3f> > &objectPoints);



    int _imagesAmount;
    int _boardWidth;
    int _boardHeight;
    cv::VideoCapture _capture;

    cv::Mat _image;

    cv::Mat _intrinsic;
    cv::Mat _distortion;

    const std::string CALIBRATION_WINDOW_NAME = "Calibration";
    const std::string UNDISTORTED_WINDOW_NAME = "Undistort";

    const char PAUSE_KEY    = 'p';
    const char ESCAPE_KEY   = 27;
    const int  WAITING_TIME = 250;

    const int board_dt = 20;
};


#endif /* CALIBRATOR_H_ */
