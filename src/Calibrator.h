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

    void setDisplayCorners(bool displayCorners) noexcept;
    void setShowUndistorted(bool showUndistorted) noexcept;

    void setSquareSize(double squareSize) noexcept;

protected:
    Calibrator() noexcept {}

    MatSharedPtr nextImage(cv::VideoCapture& capture)
        const throw (ImageReadError);

    MatSharedPtr createGrayImage() noexcept;
    void getSubpixelAccuracy() noexcept;

    void findCornersOnImage(const CalibrationData& calibrationData,
                            vector<vector<cv::Point2f>>& imagePoints) noexcept;

    void showCalibrationError(double error) const noexcept;



    MatSharedPtr _image = nullptr;
    MatSharedPtr _grayImage;
    vector<cv::Point2f> _corners;
    vector<vector<cv::Point3f>> _objectPoints;

    bool _displayCorners = true;
    bool _showUndistorted = true;

    double _squareSize = 1;

private:
    void reinitCaptureIfNecessary() noexcept;

    void presentImagesWithTheirsUndistortedCopy();
    void showImageAndItsUndistortedCopy() const noexcept;

    MatSharedPtr createUndistortedImage() const noexcept;

    char handlePause() const noexcept;
    void handleEscInterruption(char pressedKey) const throw (InterruptedByUser);

    void calibrateCamera() noexcept;

    void findAllCorners() noexcept;
    bool findCornersOnChessboard(const CalibrationData& calibrationData)
        noexcept;

    void showChessboardPointsWhenFound(
            const CalibrationData& calibrationData);
    void showChessboardPointsWhenNotFound(
            const CalibrationData &calibrationData);

    void saveImagePoints(const CalibrationData& calibrationData,
                         vector<vector<cv::Point2f>>& imagePoints) noexcept;

    void displayNumberOfSuccesses() noexcept;



    cv::VideoCapture _capture;
    std::string _captureSource = "";
    CalibrationData  _calibrationData;

    vector<vector<cv::Point2f>> _imagePoints;

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
