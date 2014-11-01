/*
 * Calibrator.cpp
 *
 *  Created on: 31-10-2014
 *      Author: Bartosz Lagwa
 */

#include "Calibrator.h"

Calibrator::Calibrator(int imagesNumber, size_t imageWidth, size_t imageHeight) noexcept
{
}

void Calibrator::showSingleImage(Mat image, Mat intrinsic, Mat distortion)
{
		Mat t = image.clone();
		imshow("Calibration", image);

		// to powinno byc robione przez initUndistortRectifyMap + remap,
		// zamiast undistort, ale nie mialem czasu przetestowac tego	 <- Maciek
		undistort(t, image, intrinsic, distortion);

		imshow("Undistort", image);
}

void Calibrator::handlePause(int &c)
{
	c = waitKey(15);
	if (c == 'p')
	{
		c = 0;
		while (c != 'p' && c != 27)
			c = waitKey(250);
	}
}

void Calibrator::showImages(Mat intrinsic, Mat distortion)
{
	Mat image;
	VideoCapture capture(0);
	capture >> image;
	/*
	Mat mapx(image.size(), CV_32FC1);
	Mat mapy(image.size(), CV_32FC1);
	Mat R;

	Mat newCamMat = getOptimalNewCameraMatrix(
								intrinsic,
								distortion,
								image.size(),
								-1);

	initUndistortRectifyMap(
			intrinsic,
			distortion,
			R,
			newCamMat,
			image.size(),
			CV_32FC1,
			mapx,
			mapy);
	*/
	namedWindow("Calibration");
	namedWindow("Undistort");
	int c = 0;
	while(!image.empty() && c != 27)
	{
		this->showSingleImage(image, intrinsic, distortion);
		this->handlePause(c);
		capture >> image;
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
