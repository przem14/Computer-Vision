#include "DisparityProvider.h"

DisparityProvider::DisparityProvider(std::string& pathToRectifyMaps) noexcept
    : _stereoBMState(cv::StereoBM::BASIC_PRESET, 256, 15)
{
    loadRectifyMaps(pathToRectifyMaps);
}

void DisparityProvider::loadRectifyMaps(std::string& pathRoRectifyMaps) noexcept
{
    cv::FileStorage fileStorage(pathRoRectifyMaps, cv::FileStorage::READ);
    fileStorage[RECTIFY_MAP_X1_TITLE] >> _rectifyMapXLeft;
    fileStorage[RECTIFY_MAP_Y1_TITLE] >> _rectifyMapYLeft;
    fileStorage[RECTIFY_MAP_X2_TITLE] >> _rectifyMapXRight;
    fileStorage[RECTIFY_MAP_Y2_TITLE] >> _rectifyMapYRight;
    fileStorage.release();
}

void DisparityProvider::computeAndDisplayDisparityMap(
        std::string& leftImage, std::string& rightImage) noexcept
{
    prepareImages(leftImage, rightImage);
    computeDisparityMap();

    DisplayManager::createWindows({DISPARITY_WINDOW_TITLE});
    DisplayManager::showImages(
        {std::make_tuple(DISPARITY_WINDOW_TITLE,
                         std::make_shared<cv::Mat>(_disparityBlackWhite),
                         1)});

    disparityConfigurator();
}

void DisparityProvider::computeDisparityMap() noexcept
{
    _stereoBMState(_leftImage, _rightImage, _disparity, CV_16S);
    cv::normalize(_disparity, _disparityBlackWhite, 0, 255, CV_MINMAX, CV_8U);
}

void DisparityProvider::prepareImages(std::string& leftImage,
                                      std::string& rightImage) noexcept
{
    loadGrayImages(leftImage, rightImage);
    remapImages();
}

void DisparityProvider::loadGrayImages(std::string& leftImage,
                                       std::string& rightImage) noexcept
{
    cv::Mat image;
    image  = cv::imread(leftImage);
    cv::cvtColor(image, _leftImage, CV_BGR2GRAY);
    image = cv::imread(rightImage);
    cv::cvtColor(image, _rightImage, CV_BGR2GRAY);
}

void DisparityProvider::remapImages() noexcept
{
    _leftImage  = remapImage(_leftImage,  _rectifyMapXLeft,  _rectifyMapYLeft);
    _rightImage = remapImage(_rightImage, _rectifyMapXRight, _rectifyMapYRight);
}

cv::Mat DisparityProvider::remapImage(cv::Mat& image,
                                      cv::Mat& rectifyMapX,
                                      cv::Mat& rectifyMapY) const noexcept
{
    cv::Mat remappedImage;

    cv::remap(image, remappedImage, rectifyMapX, rectifyMapY, cv::INTER_LINEAR);

    return remappedImage;
}

void DisparityProvider::disparityConfigurator() noexcept
{
    cv::createTrackbar(MIN_DISPARITY_TRACKBAR_TITLE,
                       DISPARITY_WINDOW_TITLE,
                       &_minDisparitySlider, _maxDisparity);
    cv::createTrackbar(SAD_WINDOWS_SIZE_TRACKBAR_TITLE,
                       DISPARITY_WINDOW_TITLE,
                       &_SADWindowSizeSlider, _maxSADWindowSize);
    handleSliders();
}

void DisparityProvider::handleSliders() noexcept
{
    while(true)
    {
        handleMinDisparitySlider();
        handleSADWindowsSizeSlider();

        computeDisparityMap();
        DisplayManager::showImages(
            {std::make_tuple(DISPARITY_WINDOW_TITLE,
                             std::make_shared<cv::Mat>(_disparityBlackWhite),
                             1)});

        handleESCInterruption();
    }
}

void DisparityProvider::handleMinDisparitySlider() noexcept
{
    _stereoBMState.state -> minDisparity = _minDisparitySlider - 50;
}

void DisparityProvider::handleSADWindowsSizeSlider() noexcept
{
    _stereoBMState.state -> SADWindowSize = 2 * _SADWindowSizeSlider + 5;
}

void DisparityProvider::handleESCInterruption() const throw (InterruptedByUser)
{
    char pressedKey = cv::waitKey(50);
    if (pressedKey == 27) throw InterruptedByUser();
}
