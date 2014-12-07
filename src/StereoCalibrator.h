#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <memory>

using std::vector;
using MatSharedPtr = std::shared_ptr<cv::Mat>;

class StereoCalibrator
{
public:
    StereoCalibrator(const std::string imagesLeft,
                     const std::string imagesRight,
                     int boardWidth,
                     int boardHeight) noexcept;

    void execute() noexcept;
    void bouguetsMethod(cv::Size imageSize, MatSharedPtr mx1, MatSharedPtr my1, MatSharedPtr mx2, MatSharedPtr my2,
                                      cv::Mat _R, cv::Mat _T, cv::Mat _R1, cv::Mat _R2);


private:
    double computeCalibrationError(cv::Mat _F, int nframes) noexcept;
    void showCalibrationError(cv::Mat _F, int nframes) noexcept;
    void loadSingleCalibrationResults(const std::string intrinsicL,
                                      const std::string distortionL,
                                      const std::string intrinsicR,
                                      const std::string distortionR) noexcept;

    cv::Mat _intrinsicLeft = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _distortionLeft = cv::Mat(5, 1, CV_32FC1);
    cv::Mat _intrinsicRight = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _distortionRight = cv::Mat(5, 1, CV_32FC1);

    vector<vector<cv::Point2f>> _points[2];
    std::string _imagesLeft;
    std::string _imagesRight;
    cv::VideoCapture _captureLeft;
    cv::VideoCapture _captureRight;
    int _boardWidth;
    int _boardHeight;
};

#endif // STEREOCALIBRATOR_H
