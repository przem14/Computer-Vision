#ifndef PHOTOTAKER_H
#define PHOTOTAKER_H

#include "CommonExceptions.h"

#include <opencv2/highgui/highgui.hpp>

#include <initializer_list>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <iomanip>

using MatSharedPtr = std::shared_ptr<cv::Mat>;
using CaptureAndSource = std::pair<cv::VideoCapture, std::string>;
using ListOfStringsPairs = std::initializer_list<
                                    std::pair<std::string, std::string>>;

class PhotoTaker
{
public:
    PhotoTaker(const ListOfStringsPairs &devicesAndPaths) noexcept;

    void takePhotos(const int numberOfPhotos) noexcept;
    void setDevicesAndPaths(const ListOfStringsPairs &devicesAndPaths) noexcept;

private:
    void createWindowsForAllDevices() const noexcept;
    void showImagesForAllDevices() const noexcept;
    void closeWindowsForAllDevices() const noexcept;

    MatSharedPtr nextImage(CaptureAndSource &captureAndSource)
        const throw (ImageReadError);
    void nextImagesFromAllDevices() noexcept;

    char waitForKeyInterruption() const noexcept;
    void handleKeyInterruption(char pressedKey, int &photosTaken)
        const throw (InterruptedByUser);

    void saveCurrentImages(int photoNumber) const noexcept;

    std::string concatPath(std::string directoryPath, int photoNumber)
        const noexcept;



    std::vector<std::tuple<std::string, CaptureAndSource, MatSharedPtr>>
        _pathsCapturesAndImages;


    const char TAKE_KEY   = 't';
    const char ESCAPE_KEY = 27;

    const char DIRECTORY_SEPERATOR      = '/';
    const std::string IMAGE_NAME_PREFIX = "image";
    const std::string IMAGE_EXTENSION   = ".jpg";
    const int DIGITS_IN_IMAGE_NAME      = 2;

    const std::string SUCCESSFULLY_TAKEN = "Successfully taken: ";

    const int SHOWING_TIME = 1;
    const int WAITING_TIME = 15;
    const int SAVED_IMAGE_SHOWING_TIME = 1000;
};

#endif // PHOTOTAKER_H
