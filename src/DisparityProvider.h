#ifndef DISPARITYPROVIDER_H
#define DISPARITYPROVIDER_H

#include "DisplayManager.h"
#include "CommonExceptions.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <string>

class DisparityProvider
{
public:
    DisparityProvider(std::string& pathToRectifyMaps) noexcept;

    void loadRectifyMaps(std::string& pathToRectifyMaps) noexcept;

    void computeAndDisplayDisparityMap(std::string& leftImage,
                                       std::string& rightImage) noexcept;

private:
    void prepareImages(std::string& leftImage, std::string& rightImage) noexcept;

    void loadGrayImages(std::string& leftImage, std::string& rightImage)
        noexcept;

    void remapImages() noexcept;
    cv::Mat remapImage(cv::Mat& image, cv::Mat& rectifyMapX, cv::Mat& rectifyMapY)
        const noexcept;

    void computeDisparityMap() noexcept;

    void disparityConfigurator() noexcept;

    void handleSliders() noexcept;
    void handleMinDisparitySlider() noexcept;
    void handleSADWindowsSizeSlider() noexcept;

    void handleESCInterruption() const throw (InterruptedByUser);

    void static callbackMinDisparitySlider(int newValue, void * object);
    void static callbackSADWindowsSizeSlider(int newValue, void * object);
    void static callbackGenerateSlider(int, void * object);
    void addSliders() noexcept;

    void updateMapWindow() noexcept;

    cv::StereoSGBM _stereoBMState;

    cv::Mat _disparity;
    cv::Mat _disparityBlackWhite;

    cv::Mat _leftImage;
    cv::Mat _rightImage;

    cv::Mat _rectifyMapXLeft;
    cv::Mat _rectifyMapYLeft;
    cv::Mat _rectifyMapXRight;
    cv::Mat _rectifyMapYRight;

    int _generateSlider      = 0;
    int _minDisparitySlider  = 50;
    int _maxDisparity        = 100;
    int _SADWindowSizeSlider = 2;
    int _maxSADWindowSize    = 125;

    const std::string RECTIFY_MAP_X1_TITLE = "Rectify Map X1";
    const std::string RECTIFY_MAP_Y1_TITLE = "Rectify Map Y1";
    const std::string RECTIFY_MAP_X2_TITLE = "Rectify Map X2";
    const std::string RECTIFY_MAP_Y2_TITLE = "Rectify Map Y2";

    const std::string DISPARITY_WINDOW_TITLE = "Disparity";

    const std::string MIN_DISPARITY_TRACKBAR_TITLE = "Minimum Disparity";
    const std::string SAD_WINDOWS_SIZE_TRACKBAR_TITLE = "SAD Windows Size";
    const std::string GENERATE_SLIDER_TITLE = "Generate";
};

#endif // DISPARITYPROVIDER_H
