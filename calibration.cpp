#include "src/Calibrator.h"
#include "src/StereoCalibrator.h"

int main()
{
    std::string pathL = "imagesA\\images\\left\\image%02d.jpg";
    std::string pathR = "imagesA\\images\\right\\image%02d.jpg";


    Calibrator calibratorL(20, 9, 6);
    calibratorL.reinitCaptureFieldWithImagesPath(pathL);
    calibratorL.execute();
    rename("intrinsic_matrix.yml", "intrinsic_matrixL.yml");
    rename("distortion_coeffs.yml", "distortion_coeffsL.yml");
    Calibrator calibratorR(20, 9, 6);
    calibratorR.reinitCaptureFieldWithImagesPath(pathR);
    calibratorR.execute();
    rename("intrinsic_matrix.yml", "intrinsic_matrixR.yml");
    rename("distortion_coeffs.yml", "distortion_coeffsR.yml");


    StereoCalibrator scalibrator(pathL, pathR, 9, 6);
    scalibrator.execute();

    return 0;
}
