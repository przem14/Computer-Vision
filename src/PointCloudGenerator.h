#ifndef POINTCLOUDGENERATOR_H
#define POINTCLOUDGENERATOR_H

#include <opencv2/calib3d/calib3d.hpp>

#include <string>

class PointCloudGenerator
{
public:
    PointCloudGenerator(const std::string& pathToDisparityMap,
                        const std::string& pathToD2DMappingMatrix) noexcept;

    void loadDisparityMap(const std::string& pathToDisparityMap) noexcept;
    void loadD2DMappingMatrix(const std::string& pathToD2DMappingMatrix) noexcept;

private:
    cv::Mat _disparityMap;
    cv::Mat _d2DMappingMatrix;

    const std::string DISPARITY_MAP_TITLE = "Disparity Map";
    const std::string D2D_MAPPING_MATRIX_TITLE =
        "Disparity-to-depth Mapping Matrix";
};

#endif // POINTCLOUDGENERATOR_H
