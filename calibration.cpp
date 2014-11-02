#include "src/Calibrator.h"

#include <iostream>

using namespace cv;
using namespace std;

int imagesAmount;
int boardHeight;
int boardWidth;

int main(int argc, char* argv[])
{
    if(argc != 4)
    {
        cerr << "ERROR: Wrong number of input parameters\n";
        return -1;
    }

    int boardWidth = atoi(argv[1]);
    int boardHeight = atoi(argv[2]);
    int imagesAmount = atoi(argv[3]);

    Calibrator calibrator(imagesAmount, boardWidth, boardHeight);

    calibrator.execute();

    return 0;
}
