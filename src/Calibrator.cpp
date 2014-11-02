#include "Calibrator.h"

#define PAUSE_KEY 'p'
#define ESCAPE_KEY 27

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
	Mat intrinsicMatrix(3, 3, CV_32FC1);
	Mat distortionCoeffs(5, 1, CV_32FC1);

	Mat image;
	image = getNextImage();

	int successes = findAllCorners(
						image,
						pointsOnBoardAmount,
						boardSize,
						_capture,
						image_points,
						object_points);

	intrinsicMatrix.at<float>(0,0) = 1.0f;
	intrinsicMatrix.at<float>(1,1) = 1.0f;

	vector<Mat> rot;
	vector<Mat> trans;

	calibrateCamera(
			object_points,
			image_points,
			image.size(),
			intrinsicMatrix,
			distortionCoeffs,
			rot,
			trans);

	saveIntrinsicMatrixWithYmlExtension("intrinsic_matrix.yml", intrinsicMatrix);
	saveDistortionCoeffsWithYmlExtension("distortion_coeffs.yml", distortionCoeffs);

	showImages(intrinsicMatrix, distortionCoeffs);
}

void Calibrator::showSingleImage(const Mat &image, const Mat &intrinsic, const Mat &distortion)
{
	Mat t = image.clone();
	imshow("Calibration", image);
	undistort(image, t, intrinsic, distortion);
	imshow("Undistort", t);
}

int Calibrator::handlePause()
{
	int c = waitKey(15);
	if (c == PAUSE_KEY)
	{
		c = 0;
		while (c != PAUSE_KEY && c != ESCAPE_KEY)
			c = waitKey(250);
	}
	return c;
}

void Calibrator::showImages(const Mat intrinsic, const Mat distortion)
{
	Mat image;
	image = getNextImage();
	namedWindow("Calibration");
	namedWindow("Undistort");
	int c = 0;
	while(!image.empty() && c != ESCAPE_KEY)
	{
		this->showSingleImage(image, intrinsic, distortion);
		c = this->handlePause();
		image = getNextImage();
	}
}

Mat& Calibrator::getNextImage() throw (ImageReadError)
{
	Mat* image = new Mat();
	if ( !_capture.read(*image) )
	{
		throw ImageReadError();
	}
	return *image;
}

void Calibrator::reinitCaptureFieldWithImagesPath(const string path) noexcept
{
	_capture = VideoCapture(path);
}

void Calibrator::saveIntrinsicMatrixWithYmlExtension(
		const string path,
		const Mat& intrinsicMatrix) const noexcept
{
	FileStorage fileStorage(path, FileStorage::WRITE);
	fileStorage << "Intrinsic Matrix" << intrinsicMatrix;
	fileStorage.release();
}

void Calibrator::saveDistortionCoeffsWithYmlExtension(
		const string path,
		const Mat& distortionCoeffs) const noexcept
{
	FileStorage fileStorage(path, FileStorage::WRITE);
	fileStorage << "Distortion Coefficients" << distortionCoeffs;
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
