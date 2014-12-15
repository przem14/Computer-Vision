#ifndef DISPARITYPROVIDER_H
#define DISPARITYPROVIDER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

class DisparityProvider
{
public:
    DisparityProvider(std::string& pathToRectifyMaps) noexcept;

    void loadRectifyMaps(std::string& pathToRectifyMaps) noexcept;

private:
    void prepareImages(std::string& leftImage, std::string& rightImage) noexcept;

    void loadGrayImages(std::string& leftImage, std::string& rightImage)
        noexcept;

    void remapImages() noexcept;
    cv::Mat remapImage(cv::Mat& image, cv::Mat& rectifyMapX, cv::Mat& rectifyMapY)
        const noexcept;



    cv::Mat _leftImage;
    cv::Mat _rightImage;

    cv::Mat _rectifyMapXLeft;
    cv::Mat _rectifyMapYLeft;
    cv::Mat _rectifyMapXRight;
    cv::Mat _rectifyMapYRight;

    const std::string RECTIFY_MAP_X1_TITLE = "Rectify Map X1";
    const std::string RECTIFY_MAP_Y1_TITLE = "Rectify Map Y1";
    const std::string RECTIFY_MAP_X2_TITLE = "Rectify Map X2";
    const std::string RECTIFY_MAP_Y2_TITLE = "Rectify Map Y2";
};

#endif // DISPARITYPROVIDER_H
