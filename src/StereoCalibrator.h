#ifndef STEREOCALIBRATOR_H
#define STEREOCALIBRATOR_H

#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>

using namespace std;

class StereoCalibrator
{
public:
    StereoCalibrator(const char* imageList, int boardWidth, int boardHeight)
        noexcept;

    void execute() noexcept;

private:
    const char* _imageList;
    int _boardWidth;
    int _boardHeight;
};

#endif // STEREOCALIBRATOR_H
