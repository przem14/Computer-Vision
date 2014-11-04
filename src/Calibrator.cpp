#include "Calibrator.h"

Calibrator::Calibrator(int imagesAmount, int boardWidth, int boardHeight) 
    noexcept
    : _imagesAmount(imagesAmount),
      _boardWidth(boardWidth),
      _boardHeight(boardHeight)
{
    _capture = cv::VideoCapture(0);
}

void Calibrator::execute() noexcept
{
    int pointsOnBoardAmount = _boardWidth * _boardHeight;
    cv::Size boardSize = cv::Size(_boardWidth, _boardHeight);

    vector<vector<cv::Point2f>> 
        imagePoints(_imagesAmount, vector<cv::Point2f>(pointsOnBoardAmount));
    vector<vector<cv::Point3f>>
        objectPoints(_imagesAmount, vector<cv::Point3f>(pointsOnBoardAmount));
    cv::Mat _intrinsic(3, 3, CV_32FC1);
    cv::Mat _distortion(5, 1, CV_32FC1);

    cv::Mat image;
    image = getNextImage();

    findAllCorners(image,
                   pointsOnBoardAmount,
                   boardSize,
                   imagePoints,
                   objectPoints);

    _intrinsic.at<float>(0,0) = 1.0f;
    _intrinsic.at<float>(1,1) = 1.0f;

    vector<cv::Mat> rot;
    vector<cv::Mat> trans;

    cv::calibrateCamera(objectPoints,
                        imagePoints,
                        image.size(),
                        _intrinsic,
                        _distortion,
                        rot,
                        trans);

    saveIntrinsicMatrixWithYmlExtension("intrinsic_matrix.yml");
    saveDistortionCoeffsWithYmlExtension("distortion_coeffs.yml");

    presentImagesWithTheirsUndistortedCopy();
}

void Calibrator::showImages(const std::initializer_list
                                <std::pair<const std::string&, const cv::Mat&>> 
                                &imagesWithWindowsNames) const noexcept
{
    for(auto image : imagesWithWindowsNames)
         cv::imshow(image.first.c_str(), image.second);
}

int Calibrator::handlePause() const noexcept
{
    int pressedKey = 0;

    if(cv::waitKey(WAITING_TIME) == PAUSE_KEY)
        while(pressedKey != PAUSE_KEY && pressedKey != ESCAPE_KEY)
            pressedKey = cv::waitKey(WAITING_TIME);
    return pressedKey;
}

void Calibrator::presentImagesWithTheirsUndistortedCopy()
{
    int pressedKey = 0;

    createWindows({CALIBRATION_WINDOW_NAME, UNDISTORTED_WINDOW_NAME});
    _image = getNextImage();
    while(!_image.empty() && pressedKey != ESCAPE_KEY)
    {
        showImageAndItsUndistortedCopy();
        pressedKey = this->handlePause();
        _image = getNextImage();
    }
}

void Calibrator::showImageAndItsUndistortedCopy() 
    const noexcept
{
    cv::Mat undistortedImage = createUndistortedImage(_image);

    showImages({{CALIBRATION_WINDOW_NAME, _image}, 
                {UNDISTORTED_WINDOW_NAME, undistortedImage}});
}

cv::Mat Calibrator::createUndistortedImage(const cv::Mat &image) 
    const noexcept
{
    cv::Mat undistortedImage = image.clone();

    cv::undistort(image, undistortedImage, _intrinsic, _distortion);
    return undistortedImage;
}

void Calibrator::createWindows(const std::initializer_list
                                  <const std::string> &names) 
    const noexcept
{
    for(auto name : names)
        cv::namedWindow(name.c_str());
}

cv::Mat& Calibrator::getNextImage() throw (ImageReadError)
{
    cv::Mat* image = new cv::Mat();

    if(!_capture.read(*image))
        throw ImageReadError();
    return *image;
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
    fileStorage << "Intrinsic Matrix" << _intrinsic;
    fileStorage.release();
}

void Calibrator::saveDistortionCoeffsWithYmlExtension(const std::string &path) 
    const noexcept
{
    cv::FileStorage fileStorage(path, cv::FileStorage::WRITE);
    fileStorage << "Distortion Coefficients" << _distortion;
    fileStorage.release();
}

void Calibrator::showChessboardPoints(const cv::Mat &image,
                                      const cv::Size &boardSize,
                                      const vector<cv::Point2f> &corners,
                                      const bool &found)
{
    drawChessboardCorners(image, boardSize, corners, found);
    cv::imshow("Calibration", image);
    cv::waitKey(33);
}

bool Calibrator::findCornersOnChessboard(const cv::Mat &image, 
                                         const cv::Size &boardSize, 
                                         vector<cv::Point2f> &corners)
{
    return findChessboardCorners(image, 
                                 boardSize, 
                                 corners,
                                 cv::CALIB_CB_ADAPTIVE_THRESH | 
                                 cv::CALIB_CB_FILTER_QUADS);
}

void Calibrator::getSubpixelAccuracy(const cv::Mat &image,
                                     cv::Mat &grayImage,
                                     vector<cv::Point2f> &corners)
{
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    cv::cornerSubPix(grayImage,
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

void Calibrator::findCornersOnImage(const cv::Mat &image,
                                    cv::Mat &grayImage,
                                    const cv::Size &boardSize,
                                    vector<cv::Point2f> &corners,
                                    int &successes,
                                    const int &pointsOnBoardAmount,
                                    vector<vector<cv::Point2f> > &imagePoints,
                                    vector<vector<cv::Point3f> > &objectPoints)
{
    bool found = this->findCornersOnChessboard(image, boardSize, corners);

    if(found) this->getSubpixelAccuracy(image, grayImage, corners);
    this->showChessboardPoints(image, boardSize, corners, found);
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

cv::Mat Calibrator::createGrayImage(const cv::Mat &image)
{
    return cv::Mat(image.size(), CV_8UC1);
}

void Calibrator::displayNumberOfSuccesses(const int &successes)
{
    std::cout << "Successes: " << successes << std::endl;
}

int Calibrator::findAllCorners(cv::Mat image,
                               const int &pointsOnBoardAmount,
                               const cv::Size &boardSize,
                               vector<vector<cv::Point2f> > &imagePoints,
                               vector<vector<cv::Point3f> > &objectPoints)
{
    vector<cv::Point2f> corners;
    cv::Mat grayImage = this->createGrayImage(image);
    int successes = 0, frame = 0;

    while(successes < _imagesAmount)
    {
        if(frame++ % board_dt == 0)
        {
            this->displayNumberOfSuccesses(successes);
            this->findCornersOnImage(image,
                                     grayImage,
                                     boardSize,
                                     corners,
                                     successes,
                                     pointsOnBoardAmount,
                                     imagePoints,
                                     objectPoints);
        }

        this->handlePause();
        image = getNextImage();
    }
    return successes;
}
