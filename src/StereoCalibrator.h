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
                     int boardWidth, int boardHeight) noexcept;

    void execute() noexcept;

private:
    MatSharedPtr _grayImage;
    std::string _imagesLeft;
    std::string _imagesRight;
    cv::VideoCapture _captureLeft;
    cv::VideoCapture _captureRight;
    int _boardWidth;
    int _boardHeight;
};

#endif // STEREOCALIBRATOR_H
