#include "src/Calibrator.h"

#include <iostream>

bool isNumberOfInuptParametersCorrect(const int numberOfParameters);

int main(int argc, char* argv[])
{
    if(isNumberOfInuptParametersCorrect(argc))
    {
        std::cerr << "ERROR: Wrong number of input parameters" << std::endl;
        return -1;
    }

    int boardWidth   = atoi(argv[1]);
    int boardHeight  = atoi(argv[2]);
    int imagesAmount = atoi(argv[3]);

    Calibrator calibrator(imagesAmount, boardWidth, boardHeight);

    calibrator.execute();

    return 0;
}

bool isNumberOfInuptParametersCorrect(const int numberOfParameters)
{
    const int CORRECT_NUMBER_OF_PARAMETERS = 4;

    return numberOfParameters == CORRECT_NUMBER_OF_PARAMETERS;
}
