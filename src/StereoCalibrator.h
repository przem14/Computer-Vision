#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include "StereoCalibrationData.h"
#include "DisplayManager.h"
#include "CommonExceptions.h"
#include "Calibrator.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <memory>

using std::vector;
using MatSharedPtr = std::shared_ptr<cv::Mat>;

class StereoCalibrator : public Calibrator
{
public:
    StereoCalibrator(const std::string imagesLeft,
                     const std::string imagesRight,
                     int boardWidth,
                     int boardHeight) throw (FramesAmountMatchError);

    void execute() noexcept;

    void useBouguetsMethod() noexcept;
    void useHartleyMethod()  noexcept;

private:
    void initIntrinsicsAndDistortions() noexcept;

    void loadSingleCalibrationResults(const std::string intrinsicL,
                                      const std::string distortionL,
                                      const std::string intrinsicR,
                                      const std::string distortionR) noexcept;
    void saveCalibrationResults() const noexcept;

    void chooseNextImage(const int leftOrRight) noexcept;

    void findAllCorners() noexcept;

    void calibrateCameras() noexcept;

    void initOutputMapsAndImages() noexcept;

    void precomputeMapForRemap(const cv::Mat& cameraMatrix1,
                               const cv::Mat& cameraMatrix2) noexcept;
    void bouguetsMethod();
    void hartleysMethod();
    void computingRectification() noexcept;

    double computeAverageCalibrationError() noexcept;
    void showAverageCalibrationError() noexcept;



    bool _isBouguetsMethodChoosen = true;

    std::string _imagesLeft;
    std::string _imagesRight;
    cv::VideoCapture _captureLeft;
    cv::VideoCapture _captureRight;
    StereoCalibrationData _calibrationData;

    vector<vector<cv::Point2f>> _points[2];

    MatSharedPtr _rectifyMapX1;
    MatSharedPtr _rectifyMapY1;
    MatSharedPtr _rectifyMapX2;
    MatSharedPtr _rectifyMapY2;
    MatSharedPtr _remapedImage1;
    MatSharedPtr _remapedImage2;


    const int LEFT  = 0;
    const int RIGHT = 1;

    const float RESIZE_FACTOR = 0.625;

    const std::string CORNERS_WINDOW_TITLE = "Corners";

    const std::string STEREO_ROTATION_OUTPUT_FILE = "stereo_rotation.yml";
    const std::string STEREO_TRANSLATION_OUTPUT_FILE = "stereo_translation.yml";
    const std::string ESSENTIAL_MATRIX_OUTPUT_FILE = "essential_matrix.yml";
    const std::string FUNDAMENTAL_MATRIX_OUTPUT_FILE = "fundamental_matrix.yml";
    const std::string RECT_TRANSFORMS_OUTPUT_FILE = "rect_transforms.yml";
    const std::string PROJECTION_MATRICES_OUTPUT_FILE = "projection_matrices.yml";
    const std::string D2D_MAPPING_MATRIX_OUTPUT_FILE = "d2d_mapping_matrix.yml";

    const std::string RUNNING_CALIBRATION = "Running stereo calibration ...";
    const std::string CALIBRATION_DONE = " done";
};

#endif // STEREOCALIBRATOR_H
