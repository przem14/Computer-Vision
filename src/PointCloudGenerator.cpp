#include "PointCloudGenerator.h"

PointCloudGenerator::PointCloudGenerator(
        const std::string& pathToDisparityMap,
        const std::string& pathToD2DMappingMatrix) noexcept
{
    loadDisparityMap(pathToDisparityMap);
    loadD2DMappingMatrix(pathToD2DMappingMatrix);
}

void PointCloudGenerator::generate() noexcept
{
    cv::reprojectImageTo3D(_disparityMap, _depthMap, _d2DMappingMatrix);
    savePointsWithPlyExtension();
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

void PointCloudGenerator::savePointsWithPlyExtension() noexcept
{
    int pointsAmount = computeOutput();
    _outputFile.open(OUTPUT_FILENAME, std::ofstream::out);
    addPlyHeader(pointsAmount);
    _outputFile << _output.str();
    _outputFile.close();
    std::cout << "Point cloud saved to " << OUTPUT_FILENAME << std::endl;
}

void PointCloudGenerator::addPlyHeader(int pointsAmount) noexcept
{
    _outputFile << "ply" << std::endl;
    _outputFile << "format ascii 1.0" << std::endl;
    _outputFile << "element vertex " << pointsAmount << std::endl;
    _outputFile << "property float32 x" << std::endl;
    _outputFile << "property float32 y" << std::endl;
    _outputFile << "property float32 z" << std::endl;
    _outputFile << "end_header" << std::endl;
}

int PointCloudGenerator::computeOutput() noexcept
{
    int pointsAmount = 0;
    for (int x = 0; x < _depthMap.rows; x++) {
        for (int y = 0; y < _depthMap.cols; y++) {
            cv::Point3f point = _depthMap.at<cv::Point3f>(x, y);
            if(point.x > INFINITY_VALUE || point.x < -INFINITY_VALUE) continue;
            if(point.y > INFINITY_VALUE || point.y < -INFINITY_VALUE) continue;
            if(point.z > INFINITY_VALUE || point.z < -INFINITY_VALUE) continue;
            _output << point.x << " " << point.y << " " << point.z << std::endl;
            pointsAmount++;
        }
    }
    return pointsAmount;
}
