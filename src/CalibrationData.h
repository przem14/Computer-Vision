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

    unsigned int pointsOnBoardAmount() const noexcept
    { return _pointsOnBoardAmount; }
    const cv::Size& boardSize() const noexcept { return _boardSize; }

    const cv::Mat& intrinsic(int index)  const noexcept
    { return _intrinsics[index]; }
    const cv::Mat& distortion(int index) const noexcept
    { return _distortions[index]; }

    void addIntrinsic(const cv::Mat& intrinsic) noexcept
    { _intrinsics.push_back(intrinsic); }
    void addDistortion(const cv::Mat& distortion) noexcept
    { _distortions.push_back(distortion); }
    void setIntrinsic(const cv::Mat& intrinsic, int index) noexcept
    { _intrinsics[index] = intrinsic; }
    void setDistortion(const cv::Mat& distortion, int index) noexcept
    { _distortions[index] = distortion; }

    const vector<cv::Mat>& rotation() const noexcept { return _rotation; }
    const vector<cv::Mat>& translation() const noexcept { return _translation; }

    void setRotation(const vector<cv::Mat> &rotation) noexcept
    { _rotation = rotation; }
    void setTranslation(const vector<cv::Mat> &translation) noexcept
    { _translation = translation; }

    void saveIntrinsicMatrixWithYmlExtension(const std::string &path, int index)
        const noexcept;
    void saveDistortionCoeffsWithYmlExtension(const std::string &path, int index)
        const noexcept;
    void loadIntrinsicMatrix(const std::string &path, int index) noexcept;
    void loadDistortionCoeffs(const std::string &path, int index) noexcept;

private:
    int _imagesAmount;
    int _boardWidth;
    int _boardHeight;

    int _pointsOnBoardAmount;
    cv::Size _boardSize;

    vector<cv::Mat> _intrinsics;
    vector<cv::Mat> _distortions;

    vector<cv::Mat> _rotation;
    vector<cv::Mat> _translation;
    //cv::Mat _r = cv::Mat(1, 1, CV_32FC1);
    //cv::Mat _t = cv::Mat(1, 1, CV_32FC1);
    //cv::Mat _n = cv::Mat(1, 1, CV_32FC1);

    const std::string INTRINSIC_MATRIX_TITLE = "Intrinsic Matrix";
    const std::string DISTORTION_COEFFS_TITLE = "Distortion Coefficients";
};

#endif //CALIBRATIONDATA_H_
