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
#include <memory>

using std::vector;

using MatSharedPtr = std::shared_ptr<cv::Mat>;

class Calibrator
{
public:
    Calibrator(int imagesAmount, int boardWidth, int boardHeight) noexcept;

    void execute() noexcept;

    void reinitCaptureFieldWithImagesPath(const std::string &path) noexcept;

private:
    MatSharedPtr getNextImage() throw (ImageReadError);

    void saveIntrinsicMatrixWithYmlExtension(const std::string &path)
        const noexcept;
    void saveDistortionCoeffsWithYmlExtension(const std::string &path)
        const noexcept;

    void presentImagesWithTheirsUndistortedCopy();

    void showImageAndItsUndistortedCopy() const noexcept;

    MatSharedPtr createUndistortedImage() const noexcept;

    char handlePause() const noexcept;

    void handleEscInterruption(char pressedKey) const throw (InterruptedByUser);

    void showChessboardPoints(const cv::Size &boardSize,
                              const vector<cv::Point2f> &corners,
                              const bool &found);

    int findAllCorners(const int &pointsOnBoardAmount,
                       const cv::Size &boardSize,
                       vector<vector<cv::Point2f> > &imagePoints,
                       vector<vector<cv::Point3f> > &objectPoints);
    bool findCornersOnChessboard(const cv::Size &boardSize,
                                 vector<cv::Point2f> &corners);
    void getSubpixelAccuracy(MatSharedPtr grayImage,
                             vector<cv::Point2f> &corners);
    void saveImagePoints(const int &successes,
                         const int &pointsOnBoardAmount,
                         const vector<cv::Point2f> &corners,
                         vector<vector<cv::Point2f>> &imagePoints,
                         vector<vector<cv::Point3f>> &objectPoints);
    MatSharedPtr createGrayImage();
    void displayNumberOfSuccesses(const int &successes);
    void findCornersOnImage(MatSharedPtr grayImage,
                            const cv::Size &boardSize,
                            vector<cv::Point2f> &corners,
                            int &successes,
                            const int &pointsOnBoardAmount,
                            vector<vector<cv::Point2f> > &imagePoints,
                            vector<vector<cv::Point3f> > &objectPoints);



    int _imagesAmount;
    int _boardWidth;
    int _boardHeight;
    cv::VideoCapture _capture = cv::VideoCapture(0);

    MatSharedPtr _image = nullptr;

    cv::Mat _intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _distortion = cv::Mat(5, 1, CV_32FC1);

    const std::string CALIBRATION_WINDOW_NAME = "Calibration";
    const std::string UNDISTORTED_WINDOW_NAME = "Undistort";
    const std::string INTRINSIC_MATRIX_OUTPUT_FILE = "intrinsic_matrix.yml";
    const std::string DISTORTION_COEFFS_OUTPUT_FILE = "distortion_coeffs.yml";
    const std::string INTRINSIC_MATRIX_TITLE = "Intrinsic Matrix";
    const std::string DISTORTION_COEFFS_TITLE = "Distortion Coefficients";

    const char PAUSE_KEY    = 'p';
    const char ESCAPE_KEY   = 27;
    const int  PAUSE_TIME = 250;
    const int  WAITING_TIME = 30;

    const int BOARD_DT = 20;
};


#endif /* CALIBRATOR_H_ */
