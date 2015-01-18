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

    void static callbackMinDisparitySlider(int newValue, void * object);
    void static callbackNumDisparitiesSlider(int newValue, void * object);
    void static callbackSADWindowsSizeSlider(int newValue, void * object);
    void static callbackDisp12MaxDiffSlider(int newValue, void * object);
    void static callbackPreFilterCapSlider(int newValue, void * object);
    void static callbackUniquenessRatioSlider(int newValue, void * object);
    void static callbackSpecleWindowSizeSlider(int newValue, void * object);
    void static callbackSpecleRangeSlider(int newValue, void * object);
    void static callbackSmoothnessPar1Slider(int newValue, void * object);
    void static callbackSmoothnessPar2Slider(int newValue, void * object);
    void static callbackBackgroundRemovalSlider(int, void *);
    void static callbackGenerateSlider(int, void * object);
    void addSliders() noexcept;

    void updateMapWindow() noexcept;
    void showOptionsWindow() noexcept;

    void handleKeyInterruptions() const throw (InterruptedByUser);

    void saveDisparityMap() const noexcept;



    cv::StereoSGBM _stereoSGBMState;

    cv::Mat _disparity;
    cv::Mat _disparityBlackWhite;

    cv::Mat _leftImage;
    cv::Mat _rightImage;

    cv::Mat _rectifyMapXLeft;
    cv::Mat _rectifyMapYLeft;
    cv::Mat _rectifyMapXRight;
    cv::Mat _rectifyMapYRight;

    int _generateSlider               = 0;
    int _minDisparitySlider           = 50;
    int _numDisparitiesSlider         = 48;
    int _SADWindowSizeSlider          = 2;
    int _disp12MaxDiffSlider          = 1;
    int _preFilterCapSlider           = 0;
    int _uniquenessRatioSlider        = 0;
    int _speckleWindowSizeSlider      = 0;
    int _speckleRangeSlider           = 0;
    int _smoothnessPar1Slider         = 200;
    int _smoothnessPar2Slider         = 255;
    int _backgroundRemovalSlider      = 0;
    const int _maxMinDisparity        = 100;
    const int _maxNumDisparities      = 64;
    const int _maxSADWindowSize       = 125;
    const int _maxDisp12MaxDiff       = 100;
    const int _maxPreFilterCap        = 100;
    const int _maxUniquenessRatio     = 100;
    const int _maxSpeckleWindowSize   = 250;
    const int _maxSpeckleRange        = 10;
    const int _maxSmoothnessPar1      = 1250;
    const int _maxSmoothnessPar2      = 5000;
    const int _maxBackgroundRemoval   = 255;

    const std::string DISPARITY_MAP_OUTPUT_FILE = "disparity_map.yml";

    const std::string DISPARITY_MAP_TITLE   = "Disparity Map";
    const std::string RECTIFY_MAP_X1_TITLE  = "Rectify Map X1";
    const std::string RECTIFY_MAP_Y1_TITLE  = "Rectify Map Y1";
    const std::string RECTIFY_MAP_X2_TITLE  = "Rectify Map X2";
    const std::string RECTIFY_MAP_Y2_TITLE  = "Rectify Map Y2";

    const std::string DISPARITY_WINDOW_TITLE = "Disparity";
    const std::string OPTIONS_WINDOW_TITLE = "Options";

    const std::string MIN_DISPARITY_TRACKBAR_TITLE = "Minimum Disparity";
    const std::string NUM_DISPARITIES_TRACKBAR_TITLE = "Num Disparities";
    const std::string SAD_WINDOWS_SIZE_TRACKBAR_TITLE = "SAD Windows Size";
    const std::string DISP12_MAX_DIFF_TRACKBAR_TITLE = "Disp12 Max Diff";
    const std::string PREFILTER_CAP_TRACKBAR_TITLE = "PreFilter Cap";
    const std::string UNIQUENESS_RATIO_TRACKBAR_TITLE = "Uniqueness Ratio";
    const std::string SPECKLE_WINDOW_SIZE_TRACKBAR_TITLE = "Speckle Window Size";
    const std::string SPECKLE_RANGE_TRACKBAR_TITLE = "Speckle Range";
    const std::string SMOOTHNESS_PAR1_TRACKBAR_TITLE = "Smoothness Par1";
    const std::string SMOOTHNESS_PAR2_TRACKBAR_TITLE = "Smoothness Par2";
    const std::string BACKGROUND_REMOVAL_TRACKBAR_TITLE = "Background Removal";
    const std::string GENERATE_SLIDER_TITLE = "Generate";

    const char SAVE_KEY   = 's';
    const char ESCAPE_KEY = 27;
};

#endif // DISPARITYPROVIDER_H
