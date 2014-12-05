#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "CommonExceptions.h"
#include "CalibrationData.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
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

    void reinitCaptureIfNecessary() noexcept;

    void presentImagesWithTheirsUndistortedCopy();
    void showImageAndItsUndistortedCopy() const noexcept;

    MatSharedPtr createUndistortedImage() const noexcept;

    char handlePause() const noexcept;
    void handleEscInterruption(char pressedKey) const throw (InterruptedByUser);

    void showChessboardPointsWhenFounded();
    void showChessboardPointsWhenNotFounded();

    void calibrateCamera() noexcept;

    void findAllCorners() noexcept;
    bool findCornersOnChessboard() noexcept;
    void findCornersOnImage() noexcept;

    void getSubpixelAccuracy() noexcept;

    void saveImagePoints() noexcept;

    MatSharedPtr createGrayImage() noexcept;
       
    void displayNumberOfSuccesses() noexcept;



    cv::VideoCapture _capture = cv::VideoCapture(0);
    CalibrationData  _calibrationData;

    MatSharedPtr _image = nullptr;
    MatSharedPtr _grayImage;
    vector<cv::Point2f> _corners;

    vector<vector<cv::Point2f>> _imagePoints;
    vector<vector<cv::Point3f>> _objectPoints;

    int _successes = 0;
    int _framesSkip = 20;

    bool _needReinitCapture = false;

    const std::string CALIBRATION_WINDOW_NAME = "Calibration";
    const std::string UNDISTORTED_WINDOW_NAME = "Undistort";
    const std::string INTRINSIC_MATRIX_OUTPUT_FILE = "intrinsic_matrix.yml";
    const std::string DISTORTION_COEFFS_OUTPUT_FILE = "distortion_coeffs.yml";

    const char PAUSE_KEY    = 'p';
    const char ESCAPE_KEY   = 27;
    const int  PAUSE_TIME   = 250;
    const int  WAITING_TIME = 50;
    const int  SHOWING_TIME = 1;
};


#endif /* CALIBRATOR_H_ */
