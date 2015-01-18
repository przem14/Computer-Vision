#include "PointCloudGenerator.h"

PointCloudGenerator::PointCloudGenerator(
        const std::string& pathToDisparityMap,
        const std::string& pathToD2DMappingMatrix) noexcept
{
    loadDisparityMap(pathToDisparityMap);
    loadD2DMappingMatrix(pathToD2DMappingMatrix);
}

void PointCloudGenerator::loadDisparityMap(const std::string &pathToDisparityMap)
        noexcept
{
    cv::FileStorage fileStorage(pathToDisparityMap, cv::FileStorage::READ);
    fileStorage[DISPARITY_MAP_TITLE] >> _disparityMap;
    fileStorage.release();
}

void PointCloudGenerator::loadD2DMappingMatrix(
        const std::string &pathToD2DMappingMatrix) noexcept
{
    cv::FileStorage fileStorage(pathToD2DMappingMatrix, cv::FileStorage::READ);
    fileStorage[D2D_MAPPING_MATRIX_TITLE] >> _d2DMappingMatrix;
    fileStorage.release();
}
