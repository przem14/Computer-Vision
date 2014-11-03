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

	vector<vector<Point2f> >  imagePoints(_imagesAmount, vector<Point2f>(pointsOnBoardAmount));
	vector<vector<Point3f> > objectPoints(_imagesAmount, vector<Point3f>(pointsOnBoardAmount));
	Mat intrinsicMatrix(3, 3, CV_32FC1);
	Mat distortionCoeffs(5, 1, CV_32FC1);

	Mat image;
	image = getNextImage();

	int successes = findAllCorners(
						image,
						pointsOnBoardAmount,
						boardSize,
						imagePoints,
						objectPoints);

	intrinsicMatrix.at<float>(0,0) = 1.0f;
	intrinsicMatrix.at<float>(1,1) = 1.0f;

	vector<Mat> rot;
	vector<Mat> trans;

	calibrateCamera(
			objectPoints,
			imagePoints,
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

void Calibrator::showChessboardPoints(const Mat &image, const Size &boardSize, 
				const vector<Point2f> &corners, const bool &found)
{
	drawChessboardCorners(image, boardSize, corners, found);
	imshow("Calibration", image);
	waitKey(33);
}

bool Calibrator::findCornersOnChessboard(const Mat &image, const Size &boardSize, vector<Point2f> &corners)
{
	return findChessboardCorners(image, boardSize, corners,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
}

void Calibrator::getSubpixelAccuracy(const Mat &image, Mat &grayImage, vector<Point2f> &corners)
{
	cvtColor(image, grayImage, CV_BGR2GRAY);
	cornerSubPix(
			grayImage,
			corners,
			Size(11,11),
			Size(-1,-1),
			TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
}

void Calibrator::SaveImagePoints(const int &successes, const int &pointsOnBoardAmount, const vector<Point2f> &corners,
		vector<vector<Point2f> > &imagePoints, vector<vector<Point3f> > &objectPoints)
{
	for(int j=0; j<pointsOnBoardAmount; j++)
	{
		imagePoints[successes][j]	= corners[j];
		objectPoints[successes][j].x = j / _boardWidth;
		objectPoints[successes][j].y = j % _boardWidth;
		objectPoints[successes][j].z = 0.0f;
	}
}

void Calibrator::findCornersOnImage(
		const Mat &image,
		Mat &grayImage,
		const Size &boardSize,
		vector<Point2f> &corners,
		int &successes,
		const int &pointsOnBoardAmount,
		vector<vector<Point2f> > &imagePoints,
		vector<vector<Point3f> > &objectPoints)
{
	bool found = this->findCornersOnChessboard(image, boardSize, corners);
	if (found)
		this->getSubpixelAccuracy(image, grayImage, corners);
	this->showChessboardPoints(image, boardSize, corners, found);
	if (corners.size() == pointsOnBoardAmount)
	{
		this->SaveImagePoints(successes, pointsOnBoardAmount, corners, imagePoints, objectPoints);
		successes++;
	}
}

Mat Calibrator::CreateGrayImage(const Mat &image)
{
	return Mat(image.size(), CV_8UC1);
}

void Calibrator::DisplayNumberOfSuccesses(const int &successes)
{
	cout << "Successes: " << successes << "\n";
}

int Calibrator::findAllCorners(
		Mat image,
		const int &pointsOnBoardAmount,
		const Size &boardSize,
		vector<vector<Point2f> > &imagePoints,
		vector<vector<Point3f> > &objectPoints)
{
	vector<Point2f> corners;
	Mat grayImage = this->CreateGrayImage(image);
	int successes = 0, frame = 0;
	while(successes < _imagesAmount)
	{
		if(frame++ % board_dt == 0)
		{
			this->DisplayNumberOfSuccesses(successes);
			this->findCornersOnImage(
					image,
					grayImage,
					boardSize,
					corners,
					successes,
					pointsOnBoardAmount,
					imagePoints,
					objectPoints);
		}

		int c = this->handlePause();
		image = getNextImage();
	}
	return successes;
}
