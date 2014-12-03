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
    StereoCalibrator(const char* imageList, int boardWidth, int boardHeight)
        noexcept;

    void execute() noexcept;

private:
    const char* _imageList;
    int _boardWidth;
    int _boardHeight;
};

#endif // STEREOCALIBRATOR_H
