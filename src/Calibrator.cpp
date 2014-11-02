#include "Calibrator.h"
#define PAUSE_KEY 'p'
#define ESCAPE_KEY 27

Calibrator::Calibrator(int imagesNumber, size_t imageWidth, size_t imageHeight) noexcept
{
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
	_capture = VideoCapture(0);
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
	if (!_capture.read(*image))
	{
		throw ImageReadError();
	}
	return *image;
}

void Calibrator::saveIntrinsicMatrix(const string path, const Mat& intrinsicMatrix)
	const noexcept
{
	FileStorage fileStorage(path, FileStorage::WRITE);
	fileStorage << "Intrinsic Matrix" << intrinsicMatrix;
	fileStorage.release();
}

void Calibrator::saveDistortionCoeffs(const string path, const Mat& distortionCoeffs)
	const noexcept
{
	FileStorage fileStorage(path, FileStorage::WRITE);
	fileStorage << "Distortion Coefficients" << distortionCoeffs;
	fileStorage.release();
}
