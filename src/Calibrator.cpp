#include "Calibrator.h"
#include "DisplayManager.h"

Calibrator::Calibrator(int imagesAmount, int boardWidth, int boardHeight)
    noexcept
    : _imagesAmount(imagesAmount),
      _boardWidth(boardWidth),
      _boardHeight(boardHeight)
{
}

void Calibrator::execute() noexcept
{
    int pointsOnBoardAmount = _boardWidth * _boardHeight;
    cv::Size boardSize = cv::Size(_boardWidth, _boardHeight);

    vector<vector<cv::Point2f>>
        imagePoints(_imagesAmount, vector<cv::Point2f>(pointsOnBoardAmount));
    vector<vector<cv::Point3f>>
        objectPoints(_imagesAmount, vector<cv::Point3f>(pointsOnBoardAmount));

    _image = getNextImage();

    DisplayManager::createWindows(
        {CALIBRATION_WINDOW_NAME, UNDISTORTED_WINDOW_NAME});
    findAllCorners(pointsOnBoardAmount,
                   boardSize,
                   imagePoints,
                   objectPoints);

    _intrinsic.at<float>(0,0) = 1.0f;
    _intrinsic.at<float>(1,1) = 1.0f;

    vector<cv::Mat> rot;
    vector<cv::Mat> trans;

    cv::calibrateCamera(objectPoints,
                        imagePoints,
                        _image -> size(),
                        _intrinsic,
                        _distortion,
                        rot,
                        trans);

    saveIntrinsicMatrixWithYmlExtension(INTRINSIC_MATRIX_OUTPUT_FILE);
    saveDistortionCoeffsWithYmlExtension(DISTORTION_COEFFS_OUTPUT_FILE);

    presentImagesWithTheirsUndistortedCopy();
    cv::destroyAllWindows();
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

void Calibrator::presentImagesWithTheirsUndistortedCopy()
{
    char pressedKey = 0;

    _image = getNextImage();
    while(!_image -> empty() && pressedKey != ESCAPE_KEY)
    {
        showImageAndItsUndistortedCopy();
        pressedKey = handlePause();
        _image = getNextImage();
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
    MatSharedPtr undistortedImage = MatSharedPtr(new cv::Mat(_image -> clone()));

    cv::undistort(*_image, *undistortedImage, _intrinsic, _distortion);
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
}

void Calibrator::saveIntrinsicMatrixWithYmlExtension(const std::string &path)
    const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << INTRINSIC_MATRIX_TITLE << _intrinsic;
    fileStorage.release();
}

void Calibrator::saveDistortionCoeffsWithYmlExtension(const std::string &path)
    const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << DISTORTION_COEFFS_TITLE << _distortion;
    fileStorage.release();
}

void Calibrator::showChessboardPoints(const cv::Size &boardSize,
                                      const vector<cv::Point2f> &corners,
                                      const bool &found)
{
    drawChessboardCorners(*_image, boardSize, corners, found);
    DisplayManager::showImages(
                        {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, 1)});
}

bool Calibrator::findCornersOnChessboard(const cv::Size &boardSize,
                                         vector<cv::Point2f> &corners)
{
    return findChessboardCorners(*_image,
                                 boardSize,
                                 corners,
                                 cv::CALIB_CB_ADAPTIVE_THRESH |
                                 cv::CALIB_CB_FILTER_QUADS);
}

void Calibrator::getSubpixelAccuracy(MatSharedPtr grayImage,
                                     vector<cv::Point2f> &corners)
{
    cv::cvtColor(*_image, *grayImage, CV_BGR2GRAY);
    cv::cornerSubPix(*grayImage,
                     corners,
                     cv::Size(11,11),
                     cv::Size(-1,-1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
}

void Calibrator::saveImagePoints(const int &successes,
                                 const int &pointsOnBoardAmount,
                                 const vector<cv::Point2f> &corners,
                                 vector<vector<cv::Point2f>> &imagePoints,
                                 vector<vector<cv::Point3f>> &objectPoints)
{
    for(int j=0; j<pointsOnBoardAmount; j++)
    {
        imagePoints[successes][j]    = corners[j];
        objectPoints[successes][j].x = j / _boardWidth;
        objectPoints[successes][j].y = j % _boardWidth;
        objectPoints[successes][j].z = 0.0f;
    }
}

void Calibrator::findCornersOnImage(MatSharedPtr grayImage,
                                    const cv::Size &boardSize,
                                    vector<cv::Point2f> &corners,
                                    int &successes,
                                    const int &pointsOnBoardAmount,
                                    vector<vector<cv::Point2f> > &imagePoints,
                                    vector<vector<cv::Point3f> > &objectPoints)
{
    bool found = this->findCornersOnChessboard(boardSize, corners);

    if(found) this->getSubpixelAccuracy(grayImage, corners);
    this->showChessboardPoints(boardSize, corners, found);
    if(corners.size() == pointsOnBoardAmount)
    {
        this->saveImagePoints(successes,
                              pointsOnBoardAmount,
                              corners,
                              imagePoints,
                              objectPoints);
        successes++;
    }
}

MatSharedPtr Calibrator::createGrayImage()
{
    return MatSharedPtr(new cv::Mat(_image -> size(), CV_8UC1));
}

void Calibrator::displayNumberOfSuccesses(const int &successes)
{
    std::cout << "Successes: " << successes << std::endl;
}

int Calibrator::findAllCorners(const int &pointsOnBoardAmount,
                               const cv::Size &boardSize,
                               vector<vector<cv::Point2f> > &imagePoints,
                               vector<vector<cv::Point3f> > &objectPoints)
{
    vector<cv::Point2f> corners;
    MatSharedPtr grayImage = this->createGrayImage();
    int successes = 0, frame = 0;

    while(successes < _imagesAmount)
    {
        DisplayManager::showImages(
            {std::make_tuple(CALIBRATION_WINDOW_NAME, _image, SHOWING_TIME)});
        if(frame++ % BOARD_DT == 0)
        {
            this->findCornersOnImage(grayImage,
                                     boardSize,
                                     corners,
                                     successes,
                                     pointsOnBoardAmount,
                                     imagePoints,
                                     objectPoints);
            this->displayNumberOfSuccesses(successes);
        }

        char pressedKey = handlePause();
        handleEscInterruption(pressedKey);
        _image = getNextImage();
    }
    return successes;
}
