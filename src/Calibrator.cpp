#include "Calibrator.h"
#include "DisplayManager.h"

Calibrator::Calibrator(int imagesAmount, int boardWidth, int boardHeight)
    noexcept
    : _calibrationData(imagesAmount, boardWidth, boardHeight)
{
}

void Calibrator::execute() noexcept
{
    _image = nextImage(_capture);

    DisplayManager::createWindows(
        {CALIBRATION_WINDOW_NAME, UNDISTORTED_WINDOW_NAME});
    findAllCorners();

    calibrateCamera();

    _calibrationData.saveIntrinsicMatrixWithYmlExtension(
            INTRINSIC_MATRIX_OUTPUT_FILE, 0);
    _calibrationData.saveDistortionCoeffsWithYmlExtension(
            DISTORTION_COEFFS_OUTPUT_FILE, 0);

    reinitCaptureIfNecessary();
    if(_showUndistorted) presentImagesWithTheirsUndistortedCopy();
    cv::destroyAllWindows();
}

void Calibrator::calibrateCamera() noexcept
{
    vector<cv::Mat> rotation, translation;
    cv::Mat intrinsic(3, 3, CV_32FC1), distortion(5, 1, CV_32FC1);

    intrinsic.at<float>(0,0) = 1.0f;
    intrinsic.at<float>(1,1) = 1.0f;

    double error = cv::calibrateCamera(_objectPoints,
                                       _imagePoints,
                                       _image -> size(),
                                       intrinsic,
                                       distortion,
                                       rotation,
                                       translation);
    showCalibrationError(error);

    _calibrationData.addIntrinsic(intrinsic);
    _calibrationData.addDistortion(distortion);
    _calibrationData.setRotation(rotation);
    _calibrationData.setTranslation(translation);
}

char Calibrator::handlePause() const noexcept
{
    char pressedKey = cv::waitKey(WAITING_TIME);

    if(pressedKey == PAUSE_KEY)
    {
        pressedKey = 0;
        while(pressedKey != PAUSE_KEY && pressedKey != ESCAPE_KEY)
            pressedKey = cv::waitKey(PAUSE_TIME);
    }
    return pressedKey;
}

void Calibrator::handleEscInterruption(char pressedKey)
    const throw (InterruptedByUser)
{
    if(pressedKey == ESCAPE_KEY) throw InterruptedByUser();
}

void Calibrator::reinitCaptureIfNecessary() noexcept
{
    if(_needReinitCapture)
        _capture = cv::VideoCapture(_captureSource);
}

void Calibrator::presentImagesWithTheirsUndistortedCopy()
{
    char pressedKey = 0;

    while(pressedKey != ESCAPE_KEY)
    {
        try{ _image = nextImage(_capture); }
        catch(ImageReadError) { break; }
        showImageAndItsUndistortedCopy();
        pressedKey = handlePause();
    }
}

void Calibrator::showImageAndItsUndistortedCopy()
    const noexcept
{
    MatSharedPtr undistortedImage = createUndistortedImage();
    DisplayManager::showImages(
               {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, 1),
                std::make_tuple(UNDISTORTED_WINDOW_NAME, undistortedImage, 1)});
}

MatSharedPtr Calibrator::createUndistortedImage() const noexcept
{
    MatSharedPtr undistortedImage =
        MatSharedPtr(new cv::Mat(_image -> clone()));

    cv::undistort(*_image,
                  *undistortedImage,
                  _calibrationData.intrinsic(0),
                  _calibrationData.distortion(0));
    return undistortedImage;
}

MatSharedPtr Calibrator::nextImage(cv::VideoCapture& capture)
    const throw (ImageReadError)
{
    MatSharedPtr image = MatSharedPtr(new cv::Mat());

    if(!capture.read(*image))
        throw ImageReadError();
    return image;
}

void Calibrator::reinitCaptureFieldWithImagesPath(const std::string &path)
    noexcept
{
    _capture = cv::VideoCapture(path);
    _captureSource = path;
    _framesSkip = 1;
    _needReinitCapture = true;
}

void Calibrator::showChessboardPointsWhenFound(
                                    const CalibrationData& calibrationData)
{
    const bool isPatternFound = true;

    drawChessboardCorners(*_image,
                          calibrationData.boardSize(),
                          _corners,
                          isPatternFound);
    DisplayManager::showImages(
            {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, 1)});
}

void Calibrator::showChessboardPointsWhenNotFound(
                                    const CalibrationData& calibrationData)
{
    const bool isPatternFound = false;

    drawChessboardCorners(*_image,
                          calibrationData.boardSize(),
                          _corners,
                          isPatternFound);
    DisplayManager::showImages(
            {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, 1)});
}


bool Calibrator::findCornersOnChessboard(const CalibrationData& calibrationData)
    noexcept
{
    return findChessboardCorners(*_image,
                                 calibrationData.boardSize(),
                                 _corners,
                                 cv::CALIB_CB_ADAPTIVE_THRESH |
                                 cv::CALIB_CB_FILTER_QUADS);
}

void Calibrator::getSubpixelAccuracy() noexcept
{
    cv::TermCriteria termCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);

    cv::cvtColor(*_image, *_grayImage, CV_BGR2GRAY);
    cv::cornerSubPix(*_grayImage,
                     _corners,
                     cv::Size(11,11),
                     cv::Size(-1,-1),
                     termCriteria);
}

void Calibrator::saveImagePoints(
        const CalibrationData& calibrationData,
        vector<vector<cv::Point2f>>& imagePoints) noexcept
{
    imagePoints.push_back(_corners);
    if(_objectPoints.size() < calibrationData.imagesAmount())
    {
        vector<cv::Point3f> boardPoints(calibrationData.pointsOnBoardAmount());
        for(size_t i = 0; i < calibrationData.pointsOnBoardAmount(); i++)
        {
            boardPoints[i] = cv::Point3f(
                        (i / calibrationData.boardWidth()) * _squareSize,
                        (i % calibrationData.boardWidth()) * _squareSize, 0);
        }
        _objectPoints.push_back(boardPoints);
    }
}

void Calibrator::findCornersOnImage(
        const CalibrationData& calibrationData,
        vector<vector<cv::Point2f>>& imagePoints) noexcept
{
    if(findCornersOnChessboard(calibrationData))
    {
        getSubpixelAccuracy();
        if(_displayCorners) showChessboardPointsWhenFound(calibrationData);
    }
    else if(_displayCorners) showChessboardPointsWhenNotFound(calibrationData);

    if(_corners.size() == calibrationData.pointsOnBoardAmount())
    {
        saveImagePoints(calibrationData, imagePoints);
        _successes++;
    }
}

MatSharedPtr Calibrator::createGrayImage() noexcept
{
    return MatSharedPtr(new cv::Mat(_image -> size(), CV_8UC1));
}

void Calibrator::displayNumberOfSuccesses() noexcept
{
    std::cout << "Successes: " << _successes << std::endl;
}

void Calibrator::findAllCorners() noexcept
{
    int frame = 0;

    _grayImage = createGrayImage();
    while(_successes < _calibrationData.imagesAmount())
    {
        DisplayManager::showImages(
            {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, SHOWING_TIME)});
        if(frame++ % _framesSkip == 0)
        {
            findCornersOnImage(_calibrationData,
                               _imagePoints);
            displayNumberOfSuccesses();
        }

        char pressedKey = handlePause();
        handleEscInterruption(pressedKey);
        if(_successes < _calibrationData.imagesAmount())
            _image = nextImage(_capture);
    }
}

void Calibrator::setDisplayCorners(bool displayCorners) noexcept
{
    _displayCorners = displayCorners;
}

void Calibrator::setShowUndistorted(bool showUndistorted) noexcept
{
    _showUndistorted = showUndistorted;
}

void Calibrator::setSquareSize(double squareSize) noexcept
{
    _squareSize = squareSize;
}

void Calibrator::showCalibrationError(double error) const noexcept
{
    std::cout << std::endl << "Err<" << error << ">" << std::endl;
}
