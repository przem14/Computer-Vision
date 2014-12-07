#include "StereoCalibrator.h"
#include "DisplayManager.h"
#include "CalibrationData.h"

StereoCalibrator::StereoCalibrator(const std::string imagesLeft,
                                   const std::string imagesRight,
                                   int boardWidth, int boardHeight)
                                            throw (FramesAmountMatchError)
    :_imagesLeft(imagesLeft),
     _imagesRight(imagesRight),
     _captureLeft(imagesLeft),
     _captureRight(imagesRight),
     _calibrationData(
              _captureLeft.get(CV_CAP_PROP_FRAME_COUNT),
              boardWidth, boardHeight),
     _objectPoints(
              _calibrationData.imagesAmount(),
              vector<cv::Point3f>(_calibrationData.pointsOnBoardAmount()))
{
    int leftFramesAmount = _captureLeft.get(CV_CAP_PROP_FRAME_COUNT);
    int rightFramesAmount = _captureRight.get(CV_CAP_PROP_FRAME_COUNT);
    if (leftFramesAmount == rightFramesAmount)
        initIntrinsicsAndDistortions();
    else
        throw FramesAmountMatchError();
}

void StereoCalibrator::bouguetsMethod(cv::Size imageSize, MatSharedPtr mx1, MatSharedPtr my1, MatSharedPtr mx2, MatSharedPtr my2,
                                      cv::Mat _R, cv::Mat _T, cv::Mat _R1, cv::Mat _R2)
{
    cv::Mat _P1(3, 4, CV_32FC1);
    cv::Mat _P2(3, 4, CV_32FC1);
    cv::Mat _Q(3, 4, CV_32FC1);
    cv::stereoRectify(
        _calibrationData.intrinsic(LEFT), _calibrationData.distortion(LEFT),
        _calibrationData.intrinsic(RIGHT), _calibrationData.distortion(RIGHT),
        imageSize, _R, _T, _R1, _R2, _P1, _P2, _Q, cv::CALIB_ZERO_DISPARITY);

    cv::initUndistortRectifyMap(
        _calibrationData.intrinsic(LEFT), _calibrationData.distortion(LEFT),
        _R1, _P1, imageSize, CV_32F, *mx1, *my1);
    cv::initUndistortRectifyMap(
        _calibrationData.intrinsic(RIGHT), _calibrationData.distortion(RIGHT),
        _R2, _P2, imageSize, CV_32F, *mx2, *my2);
}

void StereoCalibrator::execute() noexcept
{
    loadSingleCalibrationResults("intrinsic_matrixL.yml",
                                 "distortion_coeffsL.yml",
                                 "intrinsic_matrixR.yml",
                                 "distortion_coeffsR.yml");

    int useUncalibrated = 0;//defines which method of calibration use
    float resizeFactor = 0.625;
    int displayCorners = 0;
    int showUndistorted = 1;
    const int maxScale = 1;
    const float squareSize = 1.f; //Set this to your actual square size
    int lr;
    vector<uchar> active[2];
    vector<cv::Point2f> temp(_calibrationData.pointsOnBoardAmount());
    cv::Size imageSize(0, 0);
    // ARRAY AND VECTOR STORAGE:
    cv::Mat _R(3, 3, CV_32FC1);
    cv::Mat _T(3, 1, CV_32FC1);
    cv::Mat _E(3, 3, CV_32FC1);
    cv::Mat _F(3, 3, CV_32FC1);
    if(displayCorners)
        DisplayManager::createWindows({"Corners"});
    // READ IN THE LIST OF CHESSBOARDS:
    for (int i = 0;; i++)
    {
        int result = 0;
        lr = i % 2;
        MatSharedPtr img = MatSharedPtr(new cv::Mat());
        if (lr == 0)
            _captureLeft.read(*img);
        else
            _captureRight.read(*img);
        if (img -> empty())
            break;
        imageSize = img -> size();
        //FIND CHESSBOARDS AND CORNERS THEREIN:
        for (int s = 1; (s <= maxScale) && !result; s++)
        {
            MatSharedPtr timg = img;
            if (s > 1)
                cv::resize(*img,
                           *timg,
                           cv::Size(imageSize.width * s, imageSize.height * s),
                           cv::INTER_CUBIC);
            result = findChessboardCorners(
                        *timg,
                        _calibrationData.boardSize(),
                        temp,
                        cv::CALIB_CB_ADAPTIVE_THRESH |
                        cv::CALIB_CB_NORMALIZE_IMAGE);
            if (result || s == maxScale)
                for (unsigned j = 0; j < temp.size(); j++)
                {
                    temp[j].x /= s;
                    temp[j].y /= s;
                }
        }
        if (displayCorners)
        {
            MatSharedPtr cimg(new cv::Mat(imageSize, 8, 3));
            cv::cvtColor(*img, *cimg, CV_GRAY2BGR);
            cv::drawChessboardCorners(
                    *cimg,
                    _calibrationData.boardSize(),
                    temp,
                    result);
            DisplayManager::showImages(
                {std::make_tuple("corners", cimg, 5000)});
        }
        else
            putchar('.');
        active[lr].push_back((uchar)result);
    //assert( result != 0 );
        if (result)
        {
            MatSharedPtr grayImage(new cv::Mat(imageSize, CV_8UC3));
            cv::cvtColor(*img, *grayImage, CV_BGR2GRAY);
            cv::cornerSubPix(
                    *grayImage,
                    temp,
                    cv::Size(11, 11),
                    cv::Size(-1,-1),
                    cv::TermCriteria(
                        CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
            _points[lr].push_back(temp);
        }
    }
    // HARVEST CHESSBOARD 3D OBJECT POINT LIST:

    for (int k = 0; k < _calibrationData.imagesAmount(); k++)
        for (int i = 0; i < _calibrationData.boardHeight(); i++)
            for (int j = 0; j < _calibrationData.boardWidth(); j++)
                _objectPoints[k][i*_calibrationData.boardWidth() + j] =
                    cv::Point3f(i*squareSize, j*squareSize, 0);

    // CALIBRATE THE STEREO CAMERAS
    std::cout << "\nRunning stereo calibration ...";
    std::fflush(stdout);
    double error = cv::stereoCalibrate(
        _objectPoints, _points[LEFT], _points[RIGHT],
        _calibrationData.intrinsic(LEFT), _calibrationData.distortion(LEFT),
        _calibrationData.intrinsic(RIGHT), _calibrationData.distortion(RIGHT),
        imageSize, _R, _T, _E, _F,
        cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
        CV_CALIB_FIX_INTRINSIC);
    std::cout << " done\n";
    std::cout << "\nErr<" << error << ">\n";

    showCalibrationError(_F);

    //COMPUTE AND DISPLAY RECTIFICATION
    if (showUndistorted)
    {
        MatSharedPtr mx1(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr my1(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr mx2(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr my2(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr img1r(new cv::Mat(imageSize.height, imageSize.width, CV_8U));
        MatSharedPtr img2r(new cv::Mat(imageSize.height, imageSize.width, CV_8U));
        MatSharedPtr pair;
        cv::Mat _R1(3, 3, CV_32FC1);
        cv::Mat _R2(3, 3, CV_32FC1);

        if (useUncalibrated == 0)
            bouguetsMethod(imageSize, mx1, my1, mx2, my2, _R, _T, _R1, _R2);
        //OR ELSE HARTLEY'S METHOD
        else if (useUncalibrated == 1 || useUncalibrated == 2)
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
        {
            cv::Mat _H1(3, 3, CV_32FC1);
            cv::Mat _H2(3, 3, CV_32FC1);
            cv::Mat _iM(3, 3, CV_32FC1);
            //Just to show you could have independently used F
            vector<cv::Point2f> allImagesPoints[2];
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < _calibrationData.imagesAmount(); j++)
                    std::copy(_points[i][j].begin(),
                              _points[i][j].end(),
                              back_inserter(allImagesPoints[i]));
            }
            if (useUncalibrated == 2)
                _F = cv::findFundamentalMat(allImagesPoints[0],
                                            allImagesPoints[1]);
            cv::stereoRectifyUncalibrated(
                    allImagesPoints[0],
                    allImagesPoints[1],
                    _F,
                    imageSize,
                    _H1, _H2, 3);
            _R1 = _calibrationData.intrinsic(LEFT).inv() *
                    _H1 * _calibrationData.intrinsic(LEFT);
            _R2 = _calibrationData.intrinsic(RIGHT).inv() *
                    _H2 * _calibrationData.intrinsic(RIGHT);
            //Precompute map for cvRemap()
            cv::initUndistortRectifyMap(
                _calibrationData.intrinsic(LEFT),
                _calibrationData.distortion(LEFT),
                _R1, _calibrationData.intrinsic(LEFT),
                imageSize, CV_32F, *mx1, *my1);
            cv::initUndistortRectifyMap(
                _calibrationData.intrinsic(RIGHT),
                _calibrationData.distortion(RIGHT),
                _R2, _calibrationData.intrinsic(RIGHT),
                imageSize, CV_32F, *mx2, *my2);
        }
        else
            assert(0);
        // RECTIFY THE IMAGES
        pair = MatSharedPtr(new cv::Mat(imageSize.height*resizeFactor, imageSize.width*2*resizeFactor,
                                        CV_8UC3));
        _captureLeft.open(_imagesLeft);
        _captureRight.open(_imagesRight);
        for (int i = 0; i < _calibrationData.imagesAmount(); i++)
        {
            MatSharedPtr img1 = MatSharedPtr(new cv::Mat());
            MatSharedPtr img2 = MatSharedPtr(new cv::Mat());

            _captureLeft.read(*img1);
            MatSharedPtr grayImage1(new cv::Mat(imageSize, CV_8UC3));
            cv::cvtColor(*img1, *grayImage1, CV_BGR2GRAY);

            _captureRight.read(*img2);
            MatSharedPtr grayImage2(new cv::Mat(imageSize, CV_8UC3));
            cv::cvtColor(*img2, *grayImage2, CV_BGR2GRAY);

            if (!img1 -> empty() && !img2 -> empty())
            {
                MatSharedPtr part, resized1 = img1r, resized2 = img2r;
                cv::remap(*grayImage1, *img1r, *mx1, *my1, cv::INTER_LINEAR);
                cv::remap(*grayImage2, *img2r, *mx2, *my2, cv::INTER_LINEAR);
                cv::resize(*img1r,
                           *resized1,
                           cv::Size(imageSize.width * resizeFactor, imageSize.height * resizeFactor),
                           cv::INTER_AREA);
                cv::resize(*img2r,
                           *resized2,
                           cv::Size(imageSize.width * resizeFactor, imageSize.height * resizeFactor),
                           cv::INTER_AREA);
                part = MatSharedPtr(new cv::Mat(
                                pair -> colRange(0, imageSize.width*resizeFactor)));
                cv::cvtColor(*resized1, *part, CV_GRAY2BGR);
                part = MatSharedPtr(new cv::Mat(
                        pair -> colRange(imageSize.width*resizeFactor, imageSize.width*2*resizeFactor)));
                cv::cvtColor(*resized2, *part, CV_GRAY2BGR);
                for (int j = 0; j < imageSize.height * resizeFactor; j += 16*resizeFactor)
                    cv::line(*pair, cv::Point(0, j),
                             cv::Point(imageSize.width*2*resizeFactor, j),
                             CV_RGB(0, 255, 0));
                DisplayManager::showImages(
                    {std::make_tuple("rectified", pair, 5000)});
            }
        }
    }
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

double StereoCalibrator::computeCalibrationError(cv::Mat _F) noexcept
{
    vector<cv::Point3f> lines[2];

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
        cv::computeCorrespondEpilines(_points[LEFT][i], 1, _F, lines[LEFT]);
        cv::computeCorrespondEpilines(_points[RIGHT][i], 2, _F, lines[RIGHT]);
        for (unsigned j = 0; j < _calibrationData.pointsOnBoardAmount(); j++)
        {
            double err = std::abs(_points[LEFT][i][j].x*lines[RIGHT][j].x +
                _points[LEFT][i][j].y*lines[RIGHT][j].y + lines[RIGHT][j].z)
                + std::abs(_points[RIGHT][i][j].x*lines[LEFT][j].x +
                _points[RIGHT][i][j].y*lines[LEFT][j].y + lines[LEFT][j].z);
            avgErr += err;
        }
    }
    int totalPointsAmount = _calibrationData.imagesAmount() *
                            _calibrationData.pointsOnBoardAmount();
    return avgErr / totalPointsAmount;
}

void StereoCalibrator::showCalibrationError(cv::Mat _F) noexcept
{
    double err = computeCalibrationError(_F);
    std::cout << "Average calibration error: " << err << "\n";
}
