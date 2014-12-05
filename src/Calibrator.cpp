#include "Calibrator.h"
#include "DisplayManager.h"

Calibrator::Calibrator(int imagesAmount, int boardWidth, int boardHeight)
    noexcept
    : _calibrationData(imagesAmount, boardWidth, boardHeight),
      _imagePoints(
              _calibrationData.imagesAmount(),
              vector<cv::Point2f>(_calibrationData.pointsOnBoardAmount())),
      _objectPoints(
              _calibrationData.imagesAmount(),
              vector<cv::Point3f>(_calibrationData.pointsOnBoardAmount()))
{
}

void Calibrator::execute() noexcept
{
    _image = getNextImage();

    DisplayManager::createWindows(
        {CALIBRATION_WINDOW_NAME, UNDISTORTED_WINDOW_NAME});
    findAllCorners();

    calibrateCamera();

    _calibrationData.saveIntrinsicMatrixWithYmlExtension(
            INTRINSIC_MATRIX_OUTPUT_FILE);
    _calibrationData.saveDistortionCoeffsWithYmlExtension(
            DISTORTION_COEFFS_OUTPUT_FILE);

    reinitCaptureIfNecessary();
    presentImagesWithTheirsUndistortedCopy();
    cv::destroyAllWindows();
}

void Calibrator::calibrateCamera() noexcept
{
    vector<cv::Mat> rotation, translation;
    cv::Mat intrinsic(3, 3, CV_32FC1), distortion(5, 1, CV_32FC1);

    intrinsic.at<float>(0,0) = 1.0f;
    intrinsic.at<float>(1,1) = 1.0f;

    cv::calibrateCamera(_objectPoints,
                        _imagePoints,
                        _image -> size(),
                        intrinsic,
                        distortion,
                        rotation,
                        translation);

    _calibrationData.setIntrinsic(intrinsic);
    _calibrationData.setDistortion(distortion);
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
        _capture = cv::VideoCapture(_calibrationData.captureSource());
}

void Calibrator::presentImagesWithTheirsUndistortedCopy()
{
    char pressedKey = 0;

    while(pressedKey != ESCAPE_KEY)
    {
        try{ _image = getNextImage(); }
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
                  _calibrationData.intrinsic(), 
                  _calibrationData.distortion());
    return undistortedImage;
}

MatSharedPtr Calibrator::getNextImage() throw (ImageReadError)
{
    MatSharedPtr image = MatSharedPtr(new cv::Mat());

    if(!_capture.read(*image))
        throw ImageReadError();
    return image;
}

void Calibrator::reinitCaptureFieldWithImagesPath(const std::string &path)
    noexcept
{
    _capture = cv::VideoCapture(path);
    _calibrationData.setCaptureSource(path);
    _framesSkip = 1;
    _needReinitCapture = true;
}

void Calibrator::showChessboardPointsWhenFounded()
{
    const bool isPatternFounded = true;

    drawChessboardCorners(*_image, 
                          _calibrationData.boardSize(), 
                          _corners, 
                          isPatternFounded);
    DisplayManager::showImages(
            {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, 1)});
}

void Calibrator::showChessboardPointsWhenNotFounded()
{
    const bool isPatternFounded = false;

    drawChessboardCorners(*_image, 
                          _calibrationData.boardSize(), 
                          _corners, 
                          isPatternFounded);
    DisplayManager::showImages(
            {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, 1)});
}


bool Calibrator::findCornersOnChessboard() noexcept
{
    return findChessboardCorners(*_image,
                                 _calibrationData.boardSize(),
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

void Calibrator::saveImagePoints() noexcept
{
    for(int j=0; j<_calibrationData.pointsOnBoardAmount(); j++)
    {
        _imagePoints[_successes][j]    = _corners[j];
        _objectPoints[_successes][j].x = j / _calibrationData.boardWidth();
        _objectPoints[_successes][j].y = j % _calibrationData.boardWidth();
        _objectPoints[_successes][j].z = 0.0f;
    }
}

void Calibrator::findCornersOnImage() noexcept
{
    if(findCornersOnChessboard())
    {
        getSubpixelAccuracy();
        showChessboardPointsWhenFounded();
    }
    else showChessboardPointsWhenNotFounded();

    if(_corners.size() == _calibrationData.pointsOnBoardAmount())
    {
        saveImagePoints();
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
            findCornersOnImage();
            displayNumberOfSuccesses();
        }

        char pressedKey = handlePause();
        handleEscInterruption(pressedKey);
        if(_successes < _calibrationData.imagesAmount())
            _image = getNextImage();
    }
}
