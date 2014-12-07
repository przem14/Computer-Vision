#include "CalibrationData.h"

CalibrationData::CalibrationData(const int imagesAmount,
                                 const int boardWidth,
                                 const int boardHeight) noexcept
    :_imagesAmount(imagesAmount),
     _boardWidth(boardWidth),
     _boardHeight(boardHeight),
     _pointsOnBoardAmount(_boardWidth * _boardHeight),
     _boardSize(cv::Size(_boardWidth, _boardHeight))
{}

void CalibrationData::saveIntrinsicMatrixWithYmlExtension(
        const std::string &path, int index) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << INTRINSIC_MATRIX_TITLE  << _intrinsics[index];
    fileStorage.release();
}

void CalibrationData::saveDistortionCoeffsWithYmlExtension(
        const std::string &path, int index) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << DISTORTION_COEFFS_TITLE << _distortions[index];
    fileStorage.release();
}

void CalibrationData::loadIntrinsicMatrix(const std::string &path, int index)
        noexcept
{
    cv::FileStorage fileStorage;
    fileStorage.open(path, cv::FileStorage::READ);
    fileStorage[INTRINSIC_MATRIX_TITLE] >> _intrinsics[index];
    fileStorage.release();
}

void CalibrationData::loadDistortionCoeffs(const std::string &path, int index)
        noexcept
{
    cv::FileStorage fileStorage;
    fileStorage.open(path, cv::FileStorage::READ);
    fileStorage[DISTORTION_COEFFS_TITLE] >> _distortions[index];
    fileStorage.release();
}
