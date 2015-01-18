#include "DisparityProvider.h"

DisparityProvider::DisparityProvider(std::string& pathToRectifyMaps) noexcept
    : _stereoSGBMState(0,768,9,200,255,1)
{
    loadRectifyMaps(pathToRectifyMaps);
}

void DisparityProvider::loadRectifyMaps(std::string& pathToRectifyMaps) noexcept
{
    cv::FileStorage fileStorage(pathToRectifyMaps, cv::FileStorage::READ);
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

    updateMapWindow();
    showOptionsWindow();

    addSliders();
    handleKeyInterruptions();
}

void DisparityProvider::computeDisparityMap() noexcept
{
    _stereoSGBMState(_leftImage, _rightImage, _disparity);
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
    dispProvider->_stereoSGBMState.minDisparity = newValue - 50;
}

void DisparityProvider::callbackSADWindowsSizeSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.SADWindowSize = 2 * newValue + 5;
}

void DisparityProvider::callbackDisp12MaxDiffSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.disp12MaxDiff = newValue;
}

void DisparityProvider::callbackPreFilterCapSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.preFilterCap = newValue;
}

void DisparityProvider::callbackUniquenessRatioSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.uniquenessRatio = newValue;
}

void DisparityProvider::callbackSpecleWindowSizeSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.speckleWindowSize = newValue;
}

void DisparityProvider::callbackSpecleRangeSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.speckleRange = newValue;
}

void DisparityProvider::callbackSmoothnessPar1Slider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.P1 = newValue;
}

void DisparityProvider::callbackSmoothnessPar2Slider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.P2 = newValue;
}

void DisparityProvider::callbackNumDisparitiesSlider(int newValue, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    dispProvider->_stereoSGBMState.numberOfDisparities = 16 * newValue;
}

void DisparityProvider::callbackGenerateSlider(int, void* object)
{
    DisparityProvider* dispProvider = (DisparityProvider*) object;
    std::cout << "Generating disparity map... ";
    dispProvider->computeDisparityMap();
    std::cout << "done\n";
    dispProvider->updateMapWindow();
}

void DisparityProvider::addSliders() noexcept
{
    cv::createTrackbar(GENERATE_SLIDER_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_generateSlider,
                       1,
                       callbackGenerateSlider, this);
    cv::createTrackbar(MIN_DISPARITY_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_minDisparitySlider,
                       _maxMinDisparity,
                       callbackMinDisparitySlider, this);
    cv::createTrackbar(NUM_DISPARITIES_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_numDisparitiesSlider,
                       _maxNumDisparities,
                       callbackNumDisparitiesSlider, this);
    cv::createTrackbar(SAD_WINDOWS_SIZE_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_SADWindowSizeSlider,
                       _maxSADWindowSize,
                       callbackSADWindowsSizeSlider, this);
    cv::createTrackbar(DISP12_MAX_DIFF_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_disp12MaxDiffSlider,
                       _maxDisp12MaxDiff,
                       callbackDisp12MaxDiffSlider, this);
    cv::createTrackbar(PREFILTER_CAP_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_preFilterCapSlider,
                       _maxPreFilterCap,
                       callbackPreFilterCapSlider, this);
    cv::createTrackbar(UNIQUENESS_RATIO_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_uniquenessRatioSlider,
                       _maxUniquenessRatio,
                       callbackUniquenessRatioSlider, this);
    cv::createTrackbar(SPECKLE_WINDOW_SIZE_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_speckleWindowSizeSlider,
                       _maxSpeckleWindowSize,
                       callbackSpecleWindowSizeSlider, this);
    cv::createTrackbar(SPECKLE_RANGE_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_speckleRangeSlider,
                       _maxSpeckleRange,
                       callbackSpecleRangeSlider, this);
    cv::createTrackbar(SMOOTHNESS_PAR1_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_smoothnessPar1Slider,
                       _maxSmoothnessPar1,
                       callbackSmoothnessPar1Slider, this);
    cv::createTrackbar(SMOOTHNESS_PAR2_TRACKBAR_TITLE,
                       OPTIONS_WINDOW_TITLE,
                       &_smoothnessPar2Slider,
                       _maxSmoothnessPar2,
                       callbackSmoothnessPar2Slider, this);
}

void DisparityProvider::updateMapWindow() noexcept
{
    DisplayManager::showImages(
            {std::make_tuple(DISPARITY_WINDOW_TITLE,
             std::make_shared<cv::Mat>(_disparityBlackWhite),
             1)});
}

void DisparityProvider::showOptionsWindow() noexcept
{
    MatSharedPtr emptyImage =
        MatSharedPtr(new cv::Mat(50, _leftImage.size().width, CV_32F));
    DisplayManager::showImages(
        {std::make_tuple(OPTIONS_WINDOW_TITLE,
                         emptyImage,
                         1)});
}

void DisparityProvider::handleKeyInterruptions() const throw(InterruptedByUser)
{
    while(char pressedKey = cv::waitKey(0))
    {
        if(pressedKey == SAVE_KEY)
        {
            saveDisparityMap();
            break;
        }
        else if(pressedKey == ESCAPE_KEY)
            throw InterruptedByUser();
    }
}

void DisparityProvider::saveDisparityMap() const noexcept
{
    cv::FileStorage fileStorage(DISPARITY_MAP_OUTPUT_FILE,
                                cv::FileStorage::WRITE);
    fileStorage << DISPARITY_MAP_TITLE << _disparity;
    fileStorage.release();
}
