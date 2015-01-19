#ifndef POINTCLOUDGENERATOR_H
#define POINTCLOUDGENERATOR_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <string>
#include <iostream>
#include <fstream>

class PointCloudGenerator
{
public:
    PointCloudGenerator(const std::string& pathToDisparityMap,
                        const std::string& pathToD2DMappingMatrix) noexcept;

    void generate() noexcept;

    void loadDisparityMap(const std::string& pathToDisparityMap) noexcept;
    void loadD2DMappingMatrix(const std::string& pathToD2DMappingMatrix) noexcept;

private:
    void savePointsWithPlyExtension() noexcept;

    void addPlyHeader(int pointsAmount) noexcept;
    int computeOutput() noexcept;



    cv::Mat _disparityMap;
    cv::Mat _d2DMappingMatrix;
    cv::Mat _depthMap;

    std::stringstream _output;
    std::ofstream _outputFile;

    const int INFINITY_VALUE = 500;

    const std::string OUTPUT_FILENAME = "points.ply";

    const std::string DISPARITY_MAP_TITLE = "Disparity Map";
    const std::string D2D_MAPPING_MATRIX_TITLE =
        "Disparity-to-depth Mapping Matrix";
};

#endif // POINTCLOUDGENERATOR_H
