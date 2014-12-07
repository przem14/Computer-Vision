#ifndef STEREOCALIBRATIONDATA_H
#define STEREOCALIBRATIONDATA_H

#include "CalibrationData.h"

class StereoCalibrationData : public CalibrationData
{
public:
    StereoCalibrationData(const int imagesAmount,
                          const int boardWidth,
                          const int boardHeight) noexcept;

    const cv::Mat& stereoRotation() const noexcept
    { return _stereoRotation; }
    const cv::Mat& stereoTranslation() const noexcept
    { return _stereoTranslation; }
    const cv::Mat& essentialMatrix() const noexcept
    { return _essentialMatrix; }
    const cv::Mat& fundamentalMatrix() const noexcept
    { return _fundamentalMatrix; }
    const cv::Mat& rectTransform1() const noexcept
    { return _rectTransform1; }
    const cv::Mat& rectTransform2() const noexcept
    { return _rectTransform2; }
    const cv::Mat& projectionMatrix1() const noexcept
    { return _projectionMatrix1; }
    const cv::Mat& projectionMatrix2() const noexcept
    { return _projectionMatrix2; }
    const cv::Mat& homographyMatrix1() const noexcept
    { return _homographyMatrix1; }
    const cv::Mat& homographyMatrix2() const noexcept
    { return _homographyMatrix2; }
    const cv::Mat& d2DMappingMatrix() const noexcept
    { return _d2DMappingMatrix; }

    void setStereoRotation(const cv::Mat& stereoRotation) noexcept
    { _stereoRotation = stereoRotation; }
    void setStereoTranslation(const cv::Mat& stereoTranslation) noexcept
    { _stereoTranslation = stereoTranslation; }
    void setEssentialMatrix(const cv::Mat& essentialMatrix) noexcept
    { _essentialMatrix = essentialMatrix; }
    void setFundamentalMatrix(const cv::Mat& fundamentalMatrix) noexcept
    { _fundamentalMatrix = fundamentalMatrix; }
    void setRectTransforms(const cv::Mat& rectTransform1,
                           const cv::Mat& rectTransform2) noexcept;
    void setProjectionMatrices(const cv::Mat& projectionMatrix1,
                               const cv::Mat& projectionMatrix2) noexcept;
    void setHomographyMatrices(const cv::Mat& homographyMatrix1,
                               const cv::Mat& homographyMatrix2) noexcept;
    void setD2DMappingMatrix(const cv::Mat& d2DMappingMatrix) noexcept
    { _d2DMappingMatrix = d2DMappingMatrix; }

    void saveStereoRotationWithYmlExtension(const std::string &path)
        const noexcept;
    void saveStereoTranslationWithYmlExtension(const std::string &path)
        const noexcept;
    void saveEssentialMatrixWithYmlExtension(const std::string &path)
        const noexcept;
    void saveFundamentalMatrixWithYmlExtension(const std::string &path)
        const noexcept;
    void saveRectTransformsWithYmlExtension(const std::string &path)
        const noexcept;
    void saveProjectionMatricesWithYmlExtension(const std::string &path)
        const noexcept;
    void saveD2DMappingMatrixWithYmlExtension(const std::string &path)
        const noexcept;

private:
    cv::Mat _stereoRotation     = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _stereoTranslation  = cv::Mat(3, 1, CV_32FC1);
    cv::Mat _essentialMatrix    = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _fundamentalMatrix  = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _rectTransform1     = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _rectTransform2     = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _projectionMatrix1  = cv::Mat(3, 4, CV_32FC1);
    cv::Mat _projectionMatrix2  = cv::Mat(3, 4, CV_32FC1);
    cv::Mat _homographyMatrix1  = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _homographyMatrix2  = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _d2DMappingMatrix   = cv::Mat(3, 4, CV_32FC1);

    const std::string STEREO_ROTATION_TITLE     = "Stereo Rotation Matrix";
    const std::string STEREO_TRANSLATION_TITLE  = "Stereo Translation Vector";
    const std::string ESSENTIAL_MATRIX_TITLE    = "Essential Matrix";
    const std::string FUNDAMENTAL_MATRIX_TITLE  = "Fundamental Matrix";
    const std::string RECT_TRANSFORM_1_TITLE    = "Rectification Transform 1";
    const std::string RECT_TRANSFORM_2_TITLE    = "Rectification Transform 2";
    const std::string PROJECTION_MATRIX_1_TITLE = "Projection Matrix 1";
    const std::string PROJECTION_MATRIX_2_TITLE = "Projection Matrix 2";
    const std::string D2D_MAPPING_MATRIX_TITLE  =
        "Disparity-to-depth Mapping Matrix";
};

#endif // STEREOCALIBRATIONDATA_H
