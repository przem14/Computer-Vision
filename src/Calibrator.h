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
	void saveIntrinsicMatrix(const string path, const Mat& intrinsicMatrix)
		const noexcept;
	void saveDistortionCoeffs(const string path, const Mat& distortionCoeffs)
		const noexcept;
	void showSingleImage(const Mat &image, const Mat &intrinsic, const Mat &distortion);
	int handlePause();

	VideoCapture _capture;
};


#endif /* CALIBRATOR_H_ */
