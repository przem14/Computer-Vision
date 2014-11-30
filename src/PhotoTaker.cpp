#include "PhotoTaker.h"
#include "DisplayManager.h"

PhotoTaker::PhotoTaker(const ListOfStringsPairs &devicesAndPaths) noexcept
{
    setDevicesAndPaths(devicesAndPaths);
}

void PhotoTaker::takePhotos(const int numberOfPhotos) noexcept
{
    createWindowsForAllDevices();
    int photosTaken = 0;

    while(photosTaken < numberOfPhotos)
    {
        getNextImagesFromAllDevices();
        char pressedKey = waitForKeyInterruption();
        handleKeyInterruption(pressedKey, photosTaken);
        showImagesForAllDevices();
    }
    closeWindowsForAllDevices();
}

void PhotoTaker::setDevicesAndPaths(const ListOfStringsPairs &devicesAndPaths)
    noexcept
{
    _pathsCapturesAndImages.clear();
    for(auto &pair : devicesAndPaths)
    {
        _pathsCapturesAndImages.push_back(
            std::make_tuple(
                pair.second,
                std::make_pair(cv::VideoCapture(pair.first), pair.first),
                nullptr)
        );
    }
}

void PhotoTaker::createWindowsForAllDevices() const noexcept
{
    for(auto &tuple : _pathsCapturesAndImages)
        DisplayManager::createWindows({std::get<0>(tuple)});
}

void PhotoTaker::showImagesForAllDevices() const noexcept
{
    for(auto &tuple : _pathsCapturesAndImages)
        DisplayManager::showImages({std::make_tuple(
                                        std::get<0>(tuple),
                                        std::get<2>(tuple),
                                        SHOWING_TIME)});
}

void PhotoTaker::closeWindowsForAllDevices() const noexcept
{
    for(auto &tuple : _pathsCapturesAndImages)
        DisplayManager::destroyWindows({std::get<0>(tuple)});
}

MatSharedPtr PhotoTaker::getNextImage(CaptureAndSource &captureAndSource)
    const throw (ImageReadError)
{
    MatSharedPtr image = MatSharedPtr(new cv::Mat());

    if(!captureAndSource.first.read(*image))
    {
        captureAndSource.first.open(captureAndSource.second);
        if (!captureAndSource.first.read(*image))
            throw ImageReadError();
    }
    return image;
}

void PhotoTaker::getNextImagesFromAllDevices() noexcept
{
    for(auto &tuple : _pathsCapturesAndImages)
    {
        std::get<2>(tuple) = getNextImage(std::get<1>(tuple));
    }
}

char PhotoTaker::waitForKeyInterruption() const noexcept
{
    return cv::waitKey(WAITING_TIME);
}

void PhotoTaker::handleKeyInterruption(char pressedKey, int &photosTaken)
    const throw (InterruptedByUser)
{
    if(pressedKey == ESCAPE_KEY) throw InterruptedByUser();
    if(pressedKey == TAKE_KEY)
    {
        saveCurrentImages(photosTaken);
        photosTaken++;
        std::cout << SUCCESSFULLY_TAKEN << photosTaken << std::endl;
        cv::waitKey(SAVED_IMAGE_SHOWING_TIME);
    }
}

void PhotoTaker::saveCurrentImages(int photoNumber) const noexcept
{
    for(auto &tuple : _pathsCapturesAndImages)
    {
        std::string filePath = concatPath(std::get<0>(tuple), photoNumber);
        cv::imwrite(filePath, *std::get<2>(tuple));
    }
}

std::string PhotoTaker::concatPath(std::string directoryPath, int photoNumber)
    const noexcept
{
    std::stringstream stringStream;
    stringStream << directoryPath;
    if (directoryPath.back() != DIRECTORY_SEPERATOR)
        stringStream << DIRECTORY_SEPERATOR;
    stringStream << IMAGE_NAME_PREFIX;
    stringStream << std::setw(DIGITS_IN_IMAGE_NAME) << std::setfill('0');
    stringStream << photoNumber << IMAGE_EXTENSION;
    return stringStream.str();
}
