#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include "StereoCalibrationData.h"
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
    void saveCalibrationResults() const noexcept;

    void precomputeMapForRemap(cv::Size imageSize,
                               MatSharedPtr mx1, MatSharedPtr my1,
                               MatSharedPtr mx2, MatSharedPtr my2);
    void bouguetsMethod(cv::Size imageSize,
                        MatSharedPtr mx1, MatSharedPtr my1,
                        MatSharedPtr mx2, MatSharedPtr my2);
    void hartleysMethod(cv::Size imageSize, int useUncalibrated,
                        MatSharedPtr mx1, MatSharedPtr my1,
                        MatSharedPtr mx2, MatSharedPtr my2);

    double computeCalibrationError() noexcept;
    void showCalibrationError() noexcept;

    std::string _imagesLeft;
    std::string _imagesRight;
    cv::VideoCapture _captureLeft;
    cv::VideoCapture _captureRight;
    StereoCalibrationData _calibrationData;

    vector<vector<cv::Point2f>> _points[2];
    vector<vector<cv::Point3f>> _objectPoints;

    const int LEFT  = 0;
    const int RIGHT = 1;

    const std::string STEREO_ROTATION_OUTPUT_FILE = "stereo_rotation.yml";
    const std::string STEREO_TRANSLATION_OUTPUT_FILE = "stereo_translation.yml";
    const std::string ESSENTIAL_MATRIX_OUTPUT_FILE = "essential_matrix.yml";
    const std::string FUNDAMENTAL_MATRIX_OUTPUT_FILE = "fundamental_matrix.yml";
    const std::string RECT_TRANSFORMS_OUTPUT_FILE = "rect_transforms.yml";
    const std::string PROJECTION_MATRICES_OUTPUT_FILE = "projection_matrices.yml";
    const std::string D2D_MAPPING_MATRIX_OUTPUT_FILE = "d2d_mapping_matrix.yml";
};

#endif // STEREOCALIBRATOR_H
