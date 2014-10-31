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
