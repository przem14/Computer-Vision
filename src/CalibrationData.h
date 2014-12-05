#ifndef CALIBRATIONDATA_H_
#define CALIBRATIONDATA_H_

#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

using std::vector;

class CalibrationData
{
public:
    CalibrationData(const int imagesAmount,
                    const int boardWidth,
                    const int boardHeight) noexcept;

    int imagesAmount() const noexcept { return _imagesAmount; }
    int boardWidth()   const noexcept { return _boardWidth; }
    int boardHeight()  const noexcept { return _boardHeight; }

    int pointsOnBoardAmount() const noexcept { return _pointsOnBoardAmount; }
    const cv::Size& boardSize() const noexcept { return _boardSize; }

    std::string captureSource() const noexcept { return _captureSource ; }

    void setCaptureSource(const std::string& captureSource) noexcept
    { _captureSource = captureSource; }

    const cv::Mat& intrinsic()  const noexcept { return _intrinsic;  } 
    const cv::Mat& distortion() const noexcept { return _distortion; }

    void setIntrinsic(const cv::Mat& intrinsic) noexcept 
    { _intrinsic = intrinsic; }
    void setDistortion(const cv::Mat& distortion) noexcept
    { _distortion = distortion; }

    const vector<cv::Mat>& rotation() const noexcept { return _rotation; }
    const vector<cv::Mat>& translation() const noexcept { return _translation; }

    void setRotation(const vector<cv::Mat> &rotation) noexcept 
    { _rotation = rotation; }
    void setTranslation(const vector<cv::Mat> &translation) noexcept
    { _translation = translation; }

    void saveIntrinsicMatrixWithYmlExtension(const std::string &path)
        const noexcept;
    void saveDistortionCoeffsWithYmlExtension(const std::string &path)
        const noexcept;

private:
    int _imagesAmount;
    int _boardWidth;
    int _boardHeight;

    int _pointsOnBoardAmount;
    cv::Size _boardSize;

    std::string _captureSource;

    cv::Mat _intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat _distortion = cv::Mat(5, 1, CV_32FC1);

    vector<cv::Mat> _rotation;
    vector<cv::Mat> _translation;

    const std::string INTRINSIC_MATRIX_TITLE = "Intrinsic Matrix";
    const std::string DISTORTION_COEFFS_TITLE = "Distortion Coefficients";
};

#endif //CALIBRATIONDATA_H_
