#include "Calibrator.h"

Calibrator::Calibrator(int imagesAmount, int boardWidth, int boardHeight) noexcept
	: _imagesAmount(imagesAmount),
	  _boardWidth(boardWidth),
	  _boardHeight(boardHeight)
{
	_capture = VideoCapture(0);
}

void Calibrator::execute() noexcept
{
	int pointsOnBoardAmount = _boardWidth * _boardHeight;
	Size boardSize = Size(_boardWidth, _boardHeight);

	vector<vector<Point2f> >  image_points(_imagesAmount, vector<Point2f>(pointsOnBoardAmount));
	vector<vector<Point3f> > object_points(_imagesAmount, vector<Point3f>(pointsOnBoardAmount));
	Mat _intrinsic(3, 3, CV_32FC1);
	Mat _distortion(5, 1, CV_32FC1);

	Mat image;
	image = getNextImage();

	int successes = findAllCorners(
						image,
						pointsOnBoardAmount,
						boardSize,
						_capture,
						image_points,
						object_points);

	_intrinsic.at<float>(0,0) = 1.0f;
	_intrinsic.at<float>(1,1) = 1.0f;

	vector<Mat> rot;
	vector<Mat> trans;

	calibrateCamera(
			object_points,
			image_points,
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
                                <std::pair<const std::string&, const Mat&>> 
                                &imagesWithWindowsNames) const noexcept
{
    for(auto image : imagesWithWindowsNames)
	    imshow(image.first.c_str(), image.second);
}

int Calibrator::handlePause() const noexcept
{
	int pressedKey = 0;

    if(waitKey(WAITING_TIME) == PAUSE_KEY)
	    while(pressedKey != PAUSE_KEY && pressedKey != ESCAPE_KEY)
		    pressedKey = waitKey(WAITING_TIME);
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
    Mat undistortedImage = createUndistortedImage(_image);

    showImages({{CALIBRATION_WINDOW_NAME, _image}, 
                {UNDISTORTED_WINDOW_NAME, undistortedImage}});
}

Mat Calibrator::createUndistortedImage(const Mat &image) const noexcept
{
    Mat undistortedImage = image.clone();

    undistort(image, undistortedImage, _intrinsic, _distortion);
    return undistortedImage;
}

void Calibrator::createWindows(const std::initializer_list
                                  <const std::string> &names) 
    const noexcept
{
    for(auto name : names)
        namedWindow(name.c_str());
}

Mat& Calibrator::getNextImage() throw (ImageReadError)
{
	Mat* image = new Mat();

	if(!_capture.read(*image))
		throw ImageReadError();
	return *image;
}

void Calibrator::reinitCaptureFieldWithImagesPath(const string path) noexcept
{
	_capture = VideoCapture(path);
}

void Calibrator::saveIntrinsicMatrixWithYmlExtension(const string path) const noexcept
{
	FileStorage fileStorage(path, FileStorage::WRITE);
	fileStorage << "Intrinsic Matrix" << _intrinsic;
	fileStorage.release();
}

void Calibrator::saveDistortionCoeffsWithYmlExtension(const string path) const noexcept
{
	FileStorage fileStorage(path, FileStorage::WRITE);
	fileStorage << "Distortion Coefficients" << _distortion;
	fileStorage.release();
}

void Calibrator::findCornersOnBoard(
        Mat image,
        Mat gray_image,
        Size board_sz,
        vector<Point2f> corners,
        int &successes,
        int board_n,
        vector<vector<Point2f> > &image_points,
        vector<vector<Point3f> > &object_points)
{
    bool found = findChessboardCorners(
                    image,
                    board_sz,
                    corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

    if (found)
    {
        cvtColor(image, gray_image, CV_BGR2GRAY);
        cornerSubPix(
            gray_image,
            corners,
            Size(11,11),
            Size(-1,-1),
            TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

        drawChessboardCorners(
            image,
            board_sz,
            corners,
            found);
    }
    imshow("Calibration", image);
    waitKey(33);

    if (corners.size() == board_n)
    {
        for(int j = 0; j < board_n; ++j)
        {
            image_points[successes][j]    = corners[j];
            object_points[successes][j].x = j / _boardWidth;
            object_points[successes][j].y = j % _boardWidth;
            object_points[successes][j].z = 0.0f;
        }
        successes++;
    }
}

int Calibrator::findAllCorners(
        Mat image,
        int board_n,
        Size board_sz,
        VideoCapture capture,
        vector<vector<Point2f> > &image_points,
        vector<vector<Point3f> > &object_points)
{
    vector<Point2f> corners;
    Mat gray_image(image.size(), CV_8UC1);
    int successes = 0, frame = 0;

    while(successes < _imagesAmount)
    {
        if(frame++ % board_dt == 0)
        {
            cout << "Successes: " << successes << "\n";
            findCornersOnBoard(
                    image,
                    gray_image,
                    board_sz,
                    corners,
                    successes,
                    board_n,
                    image_points,
                    object_points);
        }

        int c = -1;
        if(c == 'p')
        {
            c = 0;
            while(c != 'p' && c != 27)
                c = waitKey(250);
        }
        capture >> image;
    }
    return successes;
}
