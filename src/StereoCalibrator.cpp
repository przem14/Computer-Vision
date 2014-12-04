#include "StereoCalibrator.h"
#include "DisplayManager.h"

StereoCalibrator::StereoCalibrator(const std::string imagesLeft,
                                   const std::string imagesRight,
                                   int boardWidth,
                                   int boardHeight) noexcept
    :_imagesLeft(imagesLeft),
     _imagesRight(imagesRight),
     _captureLeft(imagesLeft),
     _captureRight(imagesRight),
     _boardWidth(boardWidth),
     _boardHeight(boardHeight)
{
}

void StereoCalibrator::execute() noexcept
{
    int useUncalibrated = 0;//defines which method of calibration use

    int displayCorners = 0;
    int showUndistorted = 1;
    const int maxScale = 1;
    const float squareSize = 1.f; //Set this to your actual square size
    int lr, nframes, n = _boardWidth*_boardHeight, N = 0;
    vector<std::string> imageNames[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<cv::Point2f> temp(n);
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
        for (int s = 1; s <= maxScale; s++)
        {
            MatSharedPtr timg = img;
            if (s > 1)
                cv::resize(*img,
                           *timg,
                           cv::Size(imageSize.width * s, imageSize.height * s),
                           cv::INTER_CUBIC);
            result = findChessboardCorners(
                        *timg,
                        cv::Size(_boardWidth, _boardHeight),
                        temp,
                        cv::CALIB_CB_ADAPTIVE_THRESH |
                        cv::CALIB_CB_NORMALIZE_IMAGE);
            if (result || s == maxScale)
                for (int j = 0; j < temp.size(); j++)
                {
                    temp[j].x /= s;
                    temp[j].y /= s;
                }
            if (result)
                break;
        }

        if (displayCorners)
        {
            MatSharedPtr cimg(new cv::Mat(imageSize, 8, 3));
            cv::cvtColor(*img, *cimg, CV_GRAY2BGR);
            cv::drawChessboardCorners(
                    *cimg,
                    cv::Size(_boardWidth, _boardHeight),
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
    nframes = active[0].size();//Number of good chessboads found
    vector<vector<cv::Point3f>> objectPoints(nframes, vector<cv::Point3f>(n));

    for (int k = 0; k < nframes; k++)
        for (int i = 0; i < _boardHeight; i++)
            for (int j = 0; j < _boardWidth; j++)
                objectPoints[k][i*_boardWidth + j] =
                    cv::Point3f(i*squareSize, j*squareSize, 0);
    npoints.resize(nframes,n);
    N = nframes*n;
    cv::setIdentity(_intrinsicLeft);
    cv::setIdentity(_intrinsicRight);
    _distortionLeft.setTo(cv::Scalar::all(0));
    _distortionRight.setTo(cv::Scalar::all(0));
    // CALIBRATE THE STEREO CAMERAS
    std::cout << "\nRunning stereo calibration ...";
    std::fflush(stdout);
    cv::stereoCalibrate(
            objectPoints,
            _points[0],
            _points[1],
            _intrinsicLeft, _distortionLeft, _intrinsicRight, _distortionRight,
            imageSize,
            _R, _T, _E, _F,
            cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
            cv::CALIB_FIX_ASPECT_RATIO +
            cv::CALIB_ZERO_TANGENT_DIST +
            cv::CALIB_SAME_FOCAL_LENGTH);
    std::cout << " done\n";

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
        // IF BY CALIBRATED (BOUGUET'S METHOD)
        if (useUncalibrated == 0)
        {
            cv::Mat _P1(3, 4, CV_32FC1);
            cv::Mat _P2(3, 4, CV_32FC1);
            cv::Mat _Q(3, 4, CV_32FC1);
            cv::stereoRectify(
                    _intrinsicLeft,
                    _distortionLeft,
                    _intrinsicRight,
                    _distortionRight,
                    imageSize,
                    _R, _T, _R1, _R2, _P1, _P2, _Q,
                    cv::CALIB_ZERO_DISPARITY );
            //Precompute maps for cvRemap()
            cv::initUndistortRectifyMap(
                    _intrinsicLeft,
                    _distortionLeft,
                    _R1, _P1,
                    imageSize,
                    CV_32F,
                    *mx1, *my1);
            cv::initUndistortRectifyMap(
                    _intrinsicRight,
                    _distortionRight,
                    _R2, _P2,
                    imageSize,
                    CV_32F,
                    *mx2, *my2);
        }
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
                for (int j = 0; j < nframes; j++)
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
            _R1 = _intrinsicLeft.inv() * _H1 * _intrinsicLeft;
            _R2 = _intrinsicRight.inv() * _H2 * _intrinsicRight;
            //Precompute map for cvRemap()
            cv::initUndistortRectifyMap(
                    _intrinsicLeft,
                    _distortionLeft,
                    _R1,
                    _intrinsicLeft,
                    imageSize,
                    CV_32F,
                    *mx1, *my1);
            cv::initUndistortRectifyMap(
                    _intrinsicRight,
                    _distortionLeft,
                    _R2,
                    _intrinsicRight,
                    imageSize,
                    CV_32F,
                    *mx2, *my2);
        }
        else
            assert(0);
        // RECTIFY THE IMAGES
        pair = MatSharedPtr(new cv::Mat(imageSize.height, imageSize.width*2,
                                        CV_8UC3));
        _captureLeft.open(_imagesLeft);
        _captureRight.open(_imagesRight);
        for (int i = 0; i < nframes; i++)
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
                MatSharedPtr part;
                cv::remap(*grayImage1, *img1r, *mx1, *my1, cv::INTER_LINEAR);
                cv::remap(*grayImage2, *img2r, *mx2, *my2, cv::INTER_LINEAR);
                part = MatSharedPtr(new cv::Mat(
                                pair -> colRange(0, imageSize.width)));
                cv::cvtColor(*img1r, *part, CV_GRAY2BGR);
                part = MatSharedPtr(new cv::Mat(
                        pair -> colRange(imageSize.width, imageSize.width*2)));
                cv::cvtColor(*img2r, *part, CV_GRAY2BGR);
                for (int j = 0; j < imageSize.height; j += 16)
                    cv::line(*pair, cv::Point(0, j),
                             cv::Point(imageSize.width*2, j),
                             CV_RGB(0, 255, 0));
                DisplayManager::showImages(
                    {std::make_tuple("rectified", pair, 5000)});
            }
        }
    }
};

double StereoCalibrator::computeCalibrationError(cv::Mat _F, int nframes) noexcept
{
    vector<cv::Point3f> lines[2];
    int n = _boardWidth * _boardHeight;

    double avgErr = 0;
    for (int i = 0; i < nframes; i++ )
    {
        cv::undistortPoints(_points[0][i],
                            _points[0][i],
                            _intrinsicLeft,
                            _distortionLeft,
                            cv::noArray(),
                            _intrinsicLeft);
        cv::undistortPoints(_points[1][i],
                            _points[1][i],
                            _intrinsicRight,
                            _distortionRight,
                            cv::noArray(),
                            _intrinsicRight);
        cv::computeCorrespondEpilines(_points[0][i], 1, _F, lines[0]);
        cv::computeCorrespondEpilines(_points[1][i], 2, _F, lines[1]);
        for (int j = 0; j < n; j++)
        {
            double err = std::abs(_points[0][i][j].x*lines[1][j].x +
                _points[0][i][j].y*lines[1][j].y + lines[1][j].z)
                + std::abs(_points[1][i][j].x*lines[0][j].x +
                _points[1][i][j].y*lines[0][j].y + lines[0][j].z);
            avgErr += err;
        }
    }
    return avgErr / (nframes*n);
}

void StereoCalibrator::showCalibrationError(cv::Mat _F, int nframes) noexcept
{
    double err = computeCalibrationError(_F, nframes);
    std::cout << "Average calibration error: " << err << "\n";
}
