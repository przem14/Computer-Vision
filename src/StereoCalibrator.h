#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include "CalibrationData.h"
#include "DisplayManager.h"
#include "CommonExceptions.h"

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
                     int boardHeight) throw (FramesAmountMatchError);

    void execute() noexcept;

private:
    void initIntrinsicsAndDistortions() noexcept;

    void loadSingleCalibrationResults(const std::string intrinsicL,
                                      const std::string distortionL,
                                      const std::string intrinsicR,
                                      const std::string distortionR) noexcept;
    void precomputeMapForRemap(cv::Size imageSize,
                                     MatSharedPtr mx1, MatSharedPtr my1,
                                     MatSharedPtr mx2, MatSharedPtr my2,
                                     cv::Mat _R1, cv::Mat _R2,
                                     cv::Mat _P1, cv::Mat _P2);
    void bouguetsMethod(cv::Size imageSize,
                        MatSharedPtr mx1, MatSharedPtr my1,
                        MatSharedPtr mx2, MatSharedPtr my2,
                        cv::Mat _R, cv::Mat _T,
                        cv::Mat _R1, cv::Mat _R2);
    void hartleysMethod(cv::Size imageSize, int useUncalibrated,
                        MatSharedPtr mx1, MatSharedPtr my1,
                        MatSharedPtr mx2, MatSharedPtr my2,
                        cv::Mat _F, cv::Mat _R1, cv::Mat _R2);

    double computeCalibrationError(cv::Mat _F) noexcept;
    void showCalibrationError(cv::Mat _F) noexcept;

    std::string _imagesLeft;
    std::string _imagesRight;
    cv::VideoCapture _captureLeft;
    cv::VideoCapture _captureRight;
    CalibrationData _calibrationData;

    vector<vector<cv::Point2f>> _points[2];
    vector<vector<cv::Point3f>> _objectPoints;

    const int LEFT  = 0;
    const int RIGHT = 1;
};

#endif // STEREOCALIBRATOR_H
