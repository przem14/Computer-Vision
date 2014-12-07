#include "StereoCalibrationData.h"

StereoCalibrationData::StereoCalibrationData(const int imagesAmount,
                                             const int boardWidth,
                                             const int boardHeight) noexcept
    : CalibrationData(imagesAmount, boardWidth, boardHeight)
{
}


void StereoCalibrationData::setRectTransforms(
                                    const cv::Mat& rectTransform1,
                                    const cv::Mat& rectTransform2) noexcept
{
    _rectTransform1 = rectTransform1;
    _rectTransform2 = rectTransform2;
}

void StereoCalibrationData::setProjectionMatrices(
                                    const cv::Mat& projectionMatrix1,
                                    const cv::Mat& projectionMatrix2) noexcept
{
    _projectionMatrix1 = projectionMatrix1;
    _projectionMatrix2 = projectionMatrix2;
}

void StereoCalibrationData::setHomographyMatrices(
                                    const cv::Mat& homographyMatrix1,
                                    const cv::Mat& homographyMatrix2) noexcept
{
    _homographyMatrix1 = homographyMatrix1;
    _homographyMatrix2 = homographyMatrix2;
}

void StereoCalibrationData::saveStereoRotationWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << STEREO_ROTATION_TITLE  << _stereoRotation;
    fileStorage.release();
}

void StereoCalibrationData::saveStereoTranslationWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << STEREO_TRANSLATION_TITLE  << _stereoTranslation;
    fileStorage.release();
}

void StereoCalibrationData::saveEssentialMatrixWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << ESSENTIAL_MATRIX_TITLE  << _essentialMatrix;
    fileStorage.release();
}

void StereoCalibrationData::saveFundamentalMatrixWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << FUNDAMENTAL_MATRIX_TITLE  << _fundamentalMatrix;
    fileStorage.release();
}

void StereoCalibrationData::saveRectTransformsWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << RECT_TRANSFORM_1_TITLE << _rectTransform1;
    fileStorage << RECT_TRANSFORM_2_TITLE << _rectTransform2;
    fileStorage.release();
}

void StereoCalibrationData::saveProjectionMatricesWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << PROJECTION_MATRIX_1_TITLE << _projectionMatrix1;
    fileStorage << PROJECTION_MATRIX_2_TITLE << _projectionMatrix2;
    fileStorage.release();
}

void StereoCalibrationData::saveD2DMappingMatrixWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << D2D_MAPPING_MATRIX_TITLE  << _d2DMappingMatrix;
    fileStorage.release();
}
