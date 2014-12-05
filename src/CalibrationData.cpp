#include "CalibrationData.h"

CalibrationData::CalibrationData(const int imagesAmount,
                                 const int boardWidth,
                                 const int boardHeight) noexcept
    :_imagesAmount(imagesAmount),
     _boardWidth(boardWidth),
     _boardHeight(boardHeight),
     _pointsOnBoardAmount(_boardWidth * _boardHeight),
     _boardSize(cv::Size(_boardWidth, _boardHeight)),
     _captureSource("")
{}

void CalibrationData::saveIntrinsicMatrixWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << INTRINSIC_MATRIX_TITLE  << _intrinsic;
    fileStorage.release();
}

void CalibrationData::saveDistortionCoeffsWithYmlExtension(
        const std::string &path) const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << DISTORTION_COEFFS_TITLE << _distortion;
    fileStorage.release();
}
