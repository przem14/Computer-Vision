#include "StereoCalibrator.h"

StereoCalibrator::StereoCalibrator(const char* imageList,
                                   int boardWidth,
                                   int boardHeight) noexcept
    :_imageList(imageList),
     _boardWidth(boardWidth),
     _boardHeight(boardHeight)
{
}

void StereoCalibrator::execute() noexcept
{
    int useUncalibrated = 0;//defines which method of calibration use

    int displayCorners = 0;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
    //or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = 1.f; //Set this to your actual square size
    FILE* f = fopen(_imageList, "rt");
    int i, j, lr, nframes, n = _boardWidth*_boardHeight, N = 0;
    vector<std::string> imageNames[2];
    vector<vector<cv::Point2f>> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<cv::Point2f> temp(n);
    cv::Size imageSize(0, 0);
    // ARRAY AND VECTOR STORAGE:
    cv::Mat _M1(3, 3, CV_64F);
    cv::Mat _M2(3, 3, CV_64F);
    cv::Mat _D1(1, 5, CV_64F);
    cv::Mat _D2(1, 5, CV_64F);
    cv::Mat _R(3, 3, CV_64F);
    cv::Mat _T(3, 1, CV_64F);
    cv::Mat _E(3, 3, CV_64F);
    cv::Mat _F(3, 3, CV_64F);
    if( displayCorners )
        cv::namedWindow( "corners", 1 );
    // READ IN THE LIST OF CHESSBOARDS:
    if( !f )
    {
        std::cerr << "can not open file " << _imageList << std::endl;
        return;
    }
    for(i=0;;i++)
    {
        char buf[1024];
        int result=0;
        lr = i % 2;
        //vector<cv::Point2f>& pts = points[lr];
        if( !fgets( buf, sizeof(buf)-3, f ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        MatSharedPtr img(new cv::Mat(cv::imread( buf, 0 )));
        if( img -> empty() )
            break;
        imageSize = img -> size();
        imageNames[lr].push_back(buf);
        //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            MatSharedPtr timg = img;
            if( s > 1 )
            {
                cv::resize(*img,
                           *timg,
                           cv::Size(imageSize.width * s, imageSize.height * s),
                           cv::INTER_CUBIC);
            }
            result = findChessboardCorners( *timg,
                cv::Size(_boardWidth, _boardHeight),
                temp,
                cv::CALIB_CB_ADAPTIVE_THRESH |
                cv::CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                timg -> release();
            if( result || s == maxScale )
                for( j = 0; j < temp.size(); j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result )
                break;
        }
        if( displayCorners )
        {
            std::cout << buf << std::endl;
            MatSharedPtr cimg(new cv::Mat( imageSize, 8, 3 ));
            cv::cvtColor( *img, *cimg, CV_GRAY2BGR );
            cv::drawChessboardCorners( *cimg, cv::Size(_boardWidth, _boardHeight),
                temp, result );
            cv::imshow( "corners", *cimg );
            cimg -> release();
            if( cv::waitKey(0) == 27 ) //Allow ESC to quit
                exit(-1);
        }
        else
            putchar('.');
        active[lr].push_back((uchar)result);
    //assert( result != 0 );
        if( result )
        {
            //Calibration will suffer without subpixel interpolation
            cv::cornerSubPix( *img, temp,
                cv::Size(11, 11), cv::Size(-1,-1),
                cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                30, 0.01) );
            points[lr].push_back(temp);
        }
        img -> release();
    }
    fclose(f);
    // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    nframes = active[0].size();//Number of good chessboads found
    vector<vector<cv::Point3f>> objectPoints(nframes, vector<cv::Point3f>(n));

    for( int k = 0; k < nframes; k++ )
        for( i = 0; i < _boardHeight; i++ )
            for( j = 0; j < _boardWidth; j++ )
                objectPoints[k][i*_boardWidth + j] = cv::Point3f(i*squareSize, j*squareSize, 0);
    npoints.resize(nframes,n);
    N = nframes*n;
    cv::setIdentity(_M1);
    cv::setIdentity(_M2);
    _D1.setTo(cv::Scalar::all(0));
    _D2.setTo(cv::Scalar::all(0));
    // CALIBRATE THE STEREO CAMERAS
    std::cout << "Running stereo calibration ...";
    std::fflush(stdout);
    cv::stereoCalibrate(objectPoints, points[0],
        points[1],
        _M1, _D1, _M2, _D2,
        imageSize, _R, _T, _E, _F,
        cv::TermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 100, 1e-5),
        cv::CALIB_FIX_ASPECT_RATIO +
        cv::CALIB_ZERO_TANGENT_DIST +
        cv::CALIB_SAME_FOCAL_LENGTH );
    std::cout << " done" << std::endl;
    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    vector<cv::Point3f> lines[2];

    double avgErr = 0;
    for( i = 0; i < nframes; i++ )
    {
        cv::undistortPoints( points[0][i], points[0][i],
            _M1, _D1, cv::noArray(), _M1 );
        cv::undistortPoints( points[1][i], points[1][i],
            _M2, _D2, cv::noArray(), _M2 );
        cv::computeCorrespondEpilines( points[0][i], 1, _F, lines[0] );
        cv::computeCorrespondEpilines( points[1][i], 2, _F, lines[1] );
        for(j = 0; j < n; j++)
        {
            double err = std::abs(points[0][i][j].x*lines[1][j].x +
                points[0][i][j].y*lines[1][j].y + lines[1][j].z)
                + std::abs(points[1][i][j].x*lines[0][j].x +
                points[1][i][j].y*lines[0][j].y + lines[0][j].z);
            avgErr += err;
        }
    }
    std::cout << "avg err = " << avgErr/(nframes*n) << std::endl;
    //COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
        MatSharedPtr mx1(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr my1(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr mx2(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr my2(new cv::Mat(imageSize.height, imageSize.width, CV_32F));
        MatSharedPtr img1r(new cv::Mat(imageSize.height, imageSize.width, CV_8U));
        MatSharedPtr img2r(new cv::Mat(imageSize.height, imageSize.width, CV_8U));
        MatSharedPtr disp(new cv::Mat(imageSize.height, imageSize.width, CV_16S));
        MatSharedPtr pair;
        cv::Mat _R1(3, 3, CV_64F);
        cv::Mat _R2(3, 3, CV_64F);
        // IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            cv::Mat _P1(3, 4, CV_64F);
            cv::Mat _P2(3, 4, CV_64F);
            cv::Mat _Q(3, 4, CV_64F);
            cv::stereoRectify( _M1, _D1, _M2, _D2, imageSize,
                _R, _T,
                _R1, _R2, _P1, _P2, _Q,
                cv::CALIB_ZERO_DISPARITY );
            isVerticalStereo =
                    std::abs(_P2.at<double>(1,3)) > std::abs(_P2.at<double>(0,3));
            //Precompute maps for cvRemap()
            cv::initUndistortRectifyMap(_M1, _D1, _R1, _P1,
                                        imageSize, CV_32F, *mx1, *my1);
            cv::initUndistortRectifyMap(_M2, _D2, _R2, _P2,
                                        imageSize, CV_32F, *mx2, *my2);
        }
        //OR ELSE HARTLEY'S METHOD
        else if( useUncalibrated == 1 || useUncalibrated == 2 )
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
        {
            cv::Mat _H1(3, 3, CV_64F);
            cv::Mat _H2(3, 3, CV_64F);
            cv::Mat _iM(3, 3, CV_64F);
            //Just to show you could have independently used F
            vector<cv::Point2f> allImagesPoints[2];
            for( i = 0; i < 2; i++ )
            {
                for( j = 0; j < nframes; j++ )
                    std::copy(points[i][j].begin(),
                              points[i][j].end(),
                              back_inserter(allImagesPoints[i]));
            }
            if( useUncalibrated == 2 )
                _F = cv::findFundamentalMat( allImagesPoints[0],
                allImagesPoints[1]);
            cv::stereoRectifyUncalibrated( allImagesPoints[0],
                allImagesPoints[1], _F,
                imageSize,
                _H1, _H2, 3);
            _R1 = _M1.inv()*_H1*_M1;
            _R2 = _M2.inv()*_H2*_M2;
            //Precompute map for cvRemap()
            cv::initUndistortRectifyMap(_M1, _D1, _R1, _M1,
                                        imageSize, CV_32F, *mx1, *my1);
            cv::initUndistortRectifyMap(_M2, _D1, _R2, _M2,
                                        imageSize, CV_32F, *mx2, *my2);
        }
        else
            assert(0);
        cv::namedWindow( "rectified", 1 );
        // RECTIFY THE IMAGES
        if( !isVerticalStereo )
            pair = MatSharedPtr(new cv::Mat( imageSize.height, imageSize.width*2,
            CV_8UC3 ));
        else
            pair = MatSharedPtr(new cv::Mat( imageSize.height*2, imageSize.width,
            CV_8UC3 ));
        for( i = 0; i < nframes; i++ )
        {
            MatSharedPtr img1(new cv::Mat(
                                    cv::imread(imageNames[0][i].c_str(), 0)));
            MatSharedPtr img2(new cv::Mat(
                                    cv::imread(imageNames[1][i].c_str(), 0)));
            if( !img1 -> empty() && !img2 -> empty() )
            {
                MatSharedPtr part;
                cv::remap( *img1, *img1r, *mx1, *my1, cv::INTER_LINEAR );
                cv::remap( *img2, *img2r, *mx2, *my2, cv::INTER_LINEAR );
                if( !isVerticalStereo )
                {
                    part = MatSharedPtr(new cv::Mat(
                                    pair -> colRange(0, imageSize.width)));
                    cv::cvtColor( *img1r, *part, CV_GRAY2BGR );
                    part = MatSharedPtr(new cv::Mat(
                                    pair -> colRange(imageSize.width,
                                                     imageSize.width*2)));
                    cv::cvtColor( *img2r, *part, CV_GRAY2BGR );
                    for( j = 0; j < imageSize.height; j += 16 )
                        cv::line( *pair, cv::Point(0,j),
                        cv::Point(imageSize.width*2,j),
                        CV_RGB(0,255,0));
                }
                else
                {
                    part = MatSharedPtr(new cv::Mat(
                                    pair -> rowRange(0, imageSize.height)));
                    cv::cvtColor( *img1r, *part, CV_GRAY2BGR );
                    part = MatSharedPtr(new cv::Mat(
                                    pair -> rowRange(imageSize.height,
                                                     imageSize.height*2)));
                    cv::cvtColor( *img2r, *part, CV_GRAY2BGR );
                    for( j = 0; j < imageSize.width; j += 16 )
                        cv::line( *pair, cv::Point(j,0),
                        cv::Point(j,imageSize.height*2),
                        CV_RGB(0,255,0));
                }
                cv::imshow( "rectified", *pair );
                if( cv::waitKey() == 27 )
                    break;
            }
            img1 -> release();
            img2 -> release();
        }
        mx1 -> release();
        my1 -> release();
        mx2 -> release();
        my2 -> release();
        img1r -> release();
        img2r -> release();
        disp -> release();
    }
}
