#include "StereoCalibrator.h"

StereoCalibrator::StereoCalibrator(const std::string imagesLeft,
                                   const std::string imagesRight,
                                   int boardWidth,
                                   int boardHeight)
    throw (FramesAmountMatchError)
    : _imagesLeft(imagesLeft),
      _imagesRight(imagesRight),
      _captureLeft(imagesLeft),
      _captureRight(imagesRight),
      _calibrationData(_captureLeft.get(CV_CAP_PROP_FRAME_COUNT),
                       boardWidth,
                       boardHeight)
{
    int leftFramesAmount  = _captureLeft.get(CV_CAP_PROP_FRAME_COUNT);
    int rightFramesAmount = _captureRight.get(CV_CAP_PROP_FRAME_COUNT);

    if(leftFramesAmount != rightFramesAmount)
        throw FramesAmountMatchError();
    initIntrinsicsAndDistortions();
}

void StereoCalibrator::execute() noexcept
{
    loadSingleCalibrationResults("intrinsic_matrixL.yml",
                                 "distortion_coeffsL.yml",
                                 "intrinsic_matrixR.yml",
                                 "distortion_coeffsR.yml");

    setDisplayCorners(false);
    findAllCorners();

    calibrateCameras();
    showAverageCalibrationError();

    initOutputMapsAndImages();
    computeAndDisplayRectification();
    computeAndDisplayDisparityMap();

    saveCalibrationResults();
}

void StereoCalibrator::computeAndDisplayRectification() noexcept
{
    computeRectification();
    rectifyAndDisplayAllImages();
}

void StereoCalibrator::rectifyAndDisplayAllImages() noexcept
{
    _captureLeft.open(_imagesLeft);
    _captureRight.open(_imagesRight);

    for(int i = 0; i < _calibrationData.imagesAmount(); i++)
    {
        auto firstGrayImage  = captureLeftGrayImage();
        auto secondGrayImage = captureRightGrayImage();

        if(!firstGrayImage.empty() && !secondGrayImage.empty())
            prepareAndDisplayPairImage(firstGrayImage, secondGrayImage);
    }
}

cv::Mat StereoCalibrator::captureLeftGrayImage() noexcept
{
    cv::Mat image, grayImage(_image->size(), CV_8UC3);

    _captureLeft.read(image);
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    
    return grayImage;
}

cv::Mat StereoCalibrator::captureRightGrayImage() noexcept
{
    cv::Mat image, grayImage(_image->size(), CV_8UC3);

    _captureRight.read(image);
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    
    return grayImage;
}

void StereoCalibrator::prepareAndDisplayPairImage(
        const cv::Mat& firstGrayImage,
        const cv::Mat& secondGrayImage) const noexcept
{
    auto resizedPairImageHeight = _image->size().height * RESIZE_FACTOR;
    auto resizedPairImageWidth  = _image->size().width * 2 * RESIZE_FACTOR;
    auto pairImage
        = createPairImage(resizeImage(remapImage(firstGrayImage,
                                                 *_rectifyMapX1,
                                                 *_rectifyMapY1)),
                          resizeImage(remapImage(secondGrayImage,
                                                 *_rectifyMapX2,
                                                 *_rectifyMapY2)),
                          cv::Size(resizedPairImageWidth, 
                                   resizedPairImageHeight));

    pairImage = createImageWithHorizontalLines(pairImage); 

    DisplayManager::showImages(
        {std::make_tuple("rectified", 
                         std::make_shared<cv::Mat>(pairImage), 
                         5000)});
}

cv::Mat StereoCalibrator::createImageWithHorizontalLines(const cv::Mat& image)
    const noexcept
{
    auto resultImage = image.clone();
    auto resizedImageWidth = _image->size().width * 2 * RESIZE_FACTOR;
    auto resizedImageHeight = image.size().height;
    auto jump = 16 * RESIZE_FACTOR;

    for(size_t j = 0; j < resizedImageHeight; j += jump)
        cv::line(resultImage,
                 cv::Point(0, j),
                 cv::Point(resizedImageWidth, j),
                 CV_RGB(0, 255, 0));

    return resultImage;
}
cv::Mat StereoCalibrator::createPairImage(const cv::Mat& leftImage,
                                          const cv::Mat& rightImage,
                                          const cv::Size& size) const noexcept
{
    cv::Mat pairImage(size, CV_8UC3);
    auto convertedLeftImage  = convertToBGRImage(leftImage, leftImage.size());
    auto convertedRightImage = convertToBGRImage(rightImage, rightImage.size());
    auto halfOfWidth = size.width / 2;

    convertedLeftImage.copyTo(columnRange(pairImage, 0, halfOfWidth));
    convertedRightImage.copyTo(columnRange(pairImage, halfOfWidth, size.width));

    return pairImage;
}

cv::Mat StereoCalibrator::convertToBGRImage(const cv::Mat& grayImage, 
                                            const cv::Size& newImageSize)
    const noexcept
{
    cv::Mat bgrImage(newImageSize, CV_8UC3);

    cv::cvtColor(grayImage, bgrImage, CV_GRAY2BGR);

    return bgrImage;
} 

cv::Mat StereoCalibrator::columnRange(const cv::Mat& image, 
                                      const int startColumn,
                                      const int endColumn) const noexcept
{
    return image.colRange(startColumn, endColumn);
}

cv::Mat StereoCalibrator::remapImage(const cv::Mat& image,
                                     const cv::Mat& rectifyMapX1,
                                     const cv::Mat& rectifyMapY1)
    const noexcept
{
    cv::Mat remappedImage;

    cv::remap(image, 
              remappedImage, 
              rectifyMapX1,
              rectifyMapY1,
              cv::INTER_LINEAR);

    return remappedImage;
}

cv::Mat StereoCalibrator::resizeImage(const cv::Mat& image)
    const noexcept
{
    cv::Mat resizedImage;

    cv::resize(image,
               resizedImage,
               cv::Size(_image->size().width * RESIZE_FACTOR, 
                        _image -> size().height * RESIZE_FACTOR),
               cv::INTER_AREA);
    
    return resizedImage;
}

void StereoCalibrator::computeAndDisplayDisparityMap() noexcept
{
    MatSharedPtr _disparity(new cv::Mat(_image->size(), CV_16SC1));
    MatSharedPtr _disparityBlackWhite(new cv::Mat());

    cv::StereoBM _stereoBMState(cv::StereoBM::BASIC_PRESET, 256, 15);
    _stereoBMState(*(_remappedImage1.get()),
                   *(_remappedImage2.get()),
                   *(_disparity.get()),
                   CV_16S);

    double _min, _max;
    cv::minMaxLoc(*(_disparity.get()), &_min, &_max);
    _disparity.get()->convertTo(*(_disparityBlackWhite.get()),
                                CV_8UC1,
                                255 / (_max - _min));
    DisplayManager::showImages(
        {std::make_tuple("disparity", _disparityBlackWhite, 100000)});
}

void StereoCalibrator::useBouguetsMethod() noexcept
{
    _isBouguetsMethodChoosen = true;
}

void StereoCalibrator::useHartleyMethod() noexcept
{
    _isBouguetsMethodChoosen = false;
}

void StereoCalibrator::computeRectification() noexcept
{
    if(_isBouguetsMethodChoosen)
        bouguetsMethod();
    else
        hartleysMethod();
}

void StereoCalibrator::precomputeMapForRemap(
                                    const cv::Mat& cameraMatrix1,
                                    const cv::Mat& cameraMatrix2) noexcept
{
    cv::initUndistortRectifyMap(
        _calibrationData.intrinsic(LEFT),
        _calibrationData.distortion(LEFT),
        _calibrationData.rectTransform1(),
        cameraMatrix1,
        _image -> size(), CV_32F, *_rectifyMapX1, *_rectifyMapY1);
    cv::initUndistortRectifyMap(
        _calibrationData.intrinsic(RIGHT),
        _calibrationData.distortion(RIGHT),
        _calibrationData.rectTransform2(),
        cameraMatrix2,
        _image -> size(), CV_32F, *_rectifyMapX2, *_rectifyMapY2);
}

void StereoCalibrator::bouguetsMethod()
{
    cv::Mat rectTransform1(3, 3, CV_32FC1), rectTransform2(3, 3, CV_32FC1);
    cv::Mat projectionMatrix1(3, 4, CV_32FC1), projectionMatrix2(3, 4, CV_32FC1);
    cv::Mat d2DMappingMatrix(3, 4, CV_32FC1);

    cv::stereoRectify(
        _calibrationData.intrinsic(LEFT), _calibrationData.distortion(LEFT),
        _calibrationData.intrinsic(RIGHT), _calibrationData.distortion(RIGHT),
        _image -> size(),
        _calibrationData.stereoRotation(), _calibrationData.stereoTranslation(),
        rectTransform1, rectTransform2,
        projectionMatrix1, projectionMatrix2,
        d2DMappingMatrix, cv::CALIB_ZERO_DISPARITY);

    _calibrationData.setRectTransforms(rectTransform1, rectTransform2);
    _calibrationData.setProjectionMatrices(projectionMatrix1, projectionMatrix2);

    precomputeMapForRemap(_calibrationData.projectionMatrix1(),
                          _calibrationData.projectionMatrix2());
}


void StereoCalibrator::hartleysMethod()
{
    cv::Mat homographyMatrix1(3, 3, CV_32FC1);
    cv::Mat homographyMatrix2(3, 3, CV_32FC1);

    vector<cv::Point2f> allImagesPoints[2];
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < _calibrationData.imagesAmount(); j++)
            std::copy(_points[i][j].begin(),
                      _points[i][j].end(),
                      back_inserter(allImagesPoints[i]));
    }
    _calibrationData.setFundamentalMatrix(
        cv::findFundamentalMat(allImagesPoints[LEFT],
                               allImagesPoints[RIGHT]));

    cv::stereoRectifyUncalibrated(
        allImagesPoints[LEFT],
        allImagesPoints[RIGHT],
        _calibrationData.fundamentalMatrix(),
        _image -> size(),
        homographyMatrix1, homographyMatrix2, 3);

    _calibrationData.setHomographyMatrices(homographyMatrix1,
                                           homographyMatrix2);

    _calibrationData.setRectTransforms(
        _calibrationData.intrinsic(LEFT).inv() *
            _calibrationData.homographyMatrix1() *
            _calibrationData.intrinsic(LEFT),
        _calibrationData.intrinsic(RIGHT).inv() *
            _calibrationData.homographyMatrix2() *
            _calibrationData.intrinsic(RIGHT));

    precomputeMapForRemap(_calibrationData.intrinsic(LEFT),
                          _calibrationData.intrinsic(RIGHT));
}

void StereoCalibrator::initIntrinsicsAndDistortions() noexcept
{
    for(int i = 0; i < 2; i++)
    {
        _calibrationData.addIntrinsic(cv::Mat(3, 3, CV_32FC1));
        cv::setIdentity(_calibrationData.intrinsic(i));
        cv::Mat zeroMatrix = cv::Mat(5, 1, CV_32FC1).setTo(cv::Scalar::all(0));
        _calibrationData.addDistortion(zeroMatrix);
    }
}

void StereoCalibrator::loadSingleCalibrationResults(
                                      const std::string intrinsicL,
                                      const std::string distortionL,
                                      const std::string intrinsicR,
                                      const std::string distortionR) noexcept
{
    _calibrationData.loadIntrinsicMatrix(intrinsicL, LEFT);
    _calibrationData.loadDistortionCoeffs(distortionL, LEFT);
    _calibrationData.loadIntrinsicMatrix(intrinsicR, RIGHT);
    _calibrationData.loadDistortionCoeffs(distortionR, RIGHT);
}

void StereoCalibrator::saveCalibrationResults() const noexcept
{
    _calibrationData.saveStereoRotationWithYmlExtension(
                STEREO_ROTATION_OUTPUT_FILE);
    _calibrationData.saveStereoTranslationWithYmlExtension(
                STEREO_TRANSLATION_OUTPUT_FILE);
    _calibrationData.saveEssentialMatrixWithYmlExtension(
                ESSENTIAL_MATRIX_OUTPUT_FILE);
    _calibrationData.saveFundamentalMatrixWithYmlExtension(
                FUNDAMENTAL_MATRIX_OUTPUT_FILE);
    _calibrationData.saveRectTransformsWithYmlExtension(
                RECT_TRANSFORMS_OUTPUT_FILE);
    _calibrationData.saveProjectionMatricesWithYmlExtension(
                PROJECTION_MATRICES_OUTPUT_FILE);
    _calibrationData.saveD2DMappingMatrixWithYmlExtension(
                D2D_MAPPING_MATRIX_OUTPUT_FILE);
}

void StereoCalibrator::chooseNextImage(const int leftOrRight) noexcept
{
    if (leftOrRight == LEFT)
        _image = nextImage(_captureLeft);
    else
        _image = nextImage(_captureRight);
}

void StereoCalibrator::findAllCorners() noexcept
{
    if(_displayCorners) DisplayManager::createWindows({CORNERS_WINDOW_TITLE});

    int leftOrRight;

    for (int i = 0; i < _calibrationData.imagesAmount() * 2; i++)
    {
        leftOrRight = i % 2;
        chooseNextImage(leftOrRight);
        _grayImage = createGrayImage();
        findCornersOnImage(_calibrationData,
                           _points[leftOrRight]);
    }
}

void StereoCalibrator::calibrateCameras() noexcept
{
    std::cout << RUNNING_CALIBRATION << std::flush;

    cv::Mat stereoRotation(3, 3, CV_32FC1), stereoTranslation(3, 1, CV_32FC1);
    cv::Mat essentialMatrix(3, 3, CV_32FC1), fundamentalMatrix(3, 3, CV_32FC1);

    double error = cv::stereoCalibrate(
        _objectPoints, _points[LEFT], _points[RIGHT],
        _calibrationData.intrinsic(LEFT), _calibrationData.distortion(LEFT),
        _calibrationData.intrinsic(RIGHT), _calibrationData.distortion(RIGHT),
        _image -> size(), stereoRotation, stereoTranslation,
        essentialMatrix, fundamentalMatrix,
        cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_INTRINSIC);

    _calibrationData.setStereoRotation(stereoRotation);
    _calibrationData.setStereoTranslation(stereoTranslation);
    _calibrationData.setEssentialMatrix(essentialMatrix);
    _calibrationData.setFundamentalMatrix(fundamentalMatrix);

    std::cout << CALIBRATION_DONE << std::endl;
    showCalibrationError(error);
}

void StereoCalibrator::initOutputMapsAndImages() noexcept
{
    _rectifyMapX1 = MatSharedPtr(
            new cv::Mat(_image -> size().height, _image -> size().width, CV_32F));
    _rectifyMapY1 = MatSharedPtr(
            new cv::Mat(_image -> size().height, _image -> size().width, CV_32F));
    _rectifyMapX2 = MatSharedPtr(
            new cv::Mat(_image -> size().height, _image -> size().width, CV_32F));
    _rectifyMapY2 = MatSharedPtr(
            new cv::Mat(_image -> size().height, _image -> size().width, CV_32F));
    _remappedImage1 = MatSharedPtr(
            new cv::Mat(_image -> size().height, _image -> size().width, CV_8U));
    _remappedImage2 = MatSharedPtr(
            new cv::Mat(_image -> size().height, _image -> size().width, CV_8U));

}

double StereoCalibrator::computeAverageCalibrationError() noexcept
{
    vector<cv::Point3f> lines[2];

    double avgErr = undistortAndComputeEpilines(lines);
    int totalPointsAmount = _calibrationData.imagesAmount() *
                            _calibrationData.pointsOnBoardAmount();
    return avgErr / totalPointsAmount;
}

double StereoCalibrator::undistortAndComputeEpilines(vector<cv::Point3f> lines[])
        noexcept
{
    double avgErr = 0;
    for (int i = 0; i < _calibrationData.imagesAmount(); i++ )
    {
        cv::undistortPoints(_points[LEFT][i],
                            _points[LEFT][i],
                            _calibrationData.intrinsic(LEFT),
                            _calibrationData.distortion(LEFT),
                            cv::noArray(),
                            _calibrationData.intrinsic(LEFT));
        cv::undistortPoints(_points[RIGHT][i],
                            _points[RIGHT][i],
                            _calibrationData.intrinsic(RIGHT),
                            _calibrationData.distortion(RIGHT),
                            cv::noArray(),
                            _calibrationData.intrinsic(RIGHT));
        cv::computeCorrespondEpilines(_points[LEFT][i], 1,
                                      _calibrationData.fundamentalMatrix(),
                                      lines[LEFT]);
        cv::computeCorrespondEpilines(_points[RIGHT][i], 2,
                                      _calibrationData.fundamentalMatrix(),
                                      lines[RIGHT]);
        avgErr += computeErrorForImagePair(lines, i);
    }
    return avgErr;
}

double StereoCalibrator::computeErrorForImagePair(vector<cv::Point3f> lines[],
                                                  int index) noexcept
{
    double err = 0;
    for (unsigned j = 0; j < _calibrationData.pointsOnBoardAmount(); j++)
    {
        err += std::abs(_points[LEFT][index][j].x*lines[RIGHT][j].x +
               _points[LEFT][index][j].y*lines[RIGHT][j].y + lines[RIGHT][j].z)
               + std::abs(_points[RIGHT][index][j].x*lines[LEFT][j].x +
               _points[RIGHT][index][j].y*lines[LEFT][j].y + lines[LEFT][j].z);
    }
    return err;
}

void StereoCalibrator::showAverageCalibrationError() noexcept
{
    double err = computeAverageCalibrationError();
    std::cout << "Average calibration error: " << err << "\n";
}
