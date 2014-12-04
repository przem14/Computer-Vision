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
    double computeCalibrationError(cv::Mat _M1,
                                   cv::Mat _M2,
                                   cv::Mat _D1,
                                   cv::Mat _D2,
                                   cv::Mat _F,
                                   int nframes) noexcept;
    void showCalibrationError(cv::Mat _M1,
                              cv::Mat _M2,
                              cv::Mat _D1,
                              cv::Mat _D2,
                              cv::Mat _F,
                              int nframes) noexcept;

private:
    vector<vector<cv::Point2f>> _points[2];
    std::string _imagesLeft;
    std::string _imagesRight;
    cv::VideoCapture _captureLeft;
    cv::VideoCapture _captureRight;
    int _boardWidth;
    int _boardHeight;
};

#endif // STEREOCALIBRATOR_H
