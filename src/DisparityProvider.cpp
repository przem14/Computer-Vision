#include "DisparityProvider.h"

DisparityProvider::DisparityProvider(std::string& pathToRectifyMaps) noexcept
    : _stereoBMState(1,768,11,200,255,1,0,0,0,0,false)
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

    addSliders();
    cv::waitKey(0);
}

void DisparityProvider::computeDisparityMap() noexcept
{
    _stereoBMState(_leftImage, _rightImage, _disparity);
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

void DisparityProvider::callbackMinDisparitySlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoBMState.minDisparity = newValue - 50;
    dispProvider->updateMapWindow();
}

void DisparityProvider::callbackSADWindowsSizeSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoBMState.SADWindowSize = 2 * newValue + 5;
    dispProvider->updateMapWindow();
}

void DisparityProvider::callbackGenerateSlider(int, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    std::cout << "\nGenerating disparity map... ";
    dispProvider->computeDisparityMap();
    std::cout << "done\n";
    dispProvider->updateMapWindow();
}

void DisparityProvider::addSliders() noexcept
{
    cv::createTrackbar(GENERATE_SLIDER_TITLE,
                       DISPARITY_WINDOW_TITLE,
                       &_generateSlider,
                       1,
                       callbackGenerateSlider, this);
    cv::createTrackbar(MIN_DISPARITY_TRACKBAR_TITLE,
                       DISPARITY_WINDOW_TITLE,
                       &_minDisparitySlider,
                       _maxDisparity,
                       callbackMinDisparitySlider, this);
    cv::createTrackbar(SAD_WINDOWS_SIZE_TRACKBAR_TITLE,
                       DISPARITY_WINDOW_TITLE,
                       &_SADWindowSizeSlider,
                       _maxSADWindowSize,
                       callbackSADWindowsSizeSlider, this);
}

void DisparityProvider::updateMapWindow() noexcept
{
    DisplayManager::showImages(
            {std::make_tuple(DISPARITY_WINDOW_TITLE,
             std::make_shared<cv::Mat>(_disparityBlackWhite),
             1)});
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
                             100)});

        handleESCInterruption();
    }
}

void DisparityProvider::handleMinDisparitySlider() noexcept
{
    _stereoBMState.minDisparity = _minDisparitySlider - 50;
}

void DisparityProvider::handleSADWindowsSizeSlider() noexcept
{
    _stereoBMState.SADWindowSize = 2 * _SADWindowSizeSlider + 5;
}

void DisparityProvider::handleESCInterruption() const throw (InterruptedByUser)
{
    char pressedKey = cv::waitKey(50);
    if (pressedKey == 27) throw InterruptedByUser();
}
