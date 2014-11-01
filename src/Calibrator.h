/*
 * Calibrator.h
 *
 *  Created on: 31-10-2014
 *      Author: Bartosz Lagwa
 */

#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

#include "CalibrationExceptions.h"

using namespace std;
using namespace cv;

class Calibrator
{
public:
	Calibrator(int imagesAmount, size_t imageWidth, size_t imageHeight) noexcept;
	void showImages(Mat intrinsic, Mat distortion);
private:
	Mat& getNextImage() throw (ImageReadError);

	/*Extension of output files should be .yml*/
	void saveIntrinsicMatrix(const string path, const Mat& intrinsicMatrix)
		const noexcept;
	void saveDistortionCoeffs(const string path, const Mat& distortionCoeffs)
		const noexcept;
	void showSingleImage(Mat image, Mat intrinsic, Mat distortion);
	void handlePause(int &c);
	/*
	 * Initialization of _capture should look like:
	 * _capture = VideoCapture(/path/to/images/folder/image_%03d)
	 *
	 * Each use of getNextImage() will then return images
	 * 'image_001', 'image_002', etc.
	 */
	VideoCapture _capture;
};


#endif /* CALIBRATOR_H_ */
