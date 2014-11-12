#include "DisplayManager.h"

vector<std::string> DisplayManager::_openedWindows = vector<std::string>();

DisplayManager::DisplayManager() noexcept
{
}

void DisplayManager::showImages(
        const std::initializer_list
        <std::pair<const std::string&, const MatSharedPtr>>
        &imagesWithWindowsNames) noexcept
{
    for(auto image : imagesWithWindowsNames)
         cv::imshow(image.first.c_str(), *image.second);
}

void DisplayManager::createWindows(const std::initializer_list
                                  <const std::string> &names) noexcept
{
    for(auto name : names)
        if (isOpened(name))
        {
            cv::namedWindow(name.c_str());
            _openedWindows.push_back(name);
        }
}

vector<std::string> DisplayManager::listOfOpenedWindows() noexcept
{
    return _openedWindows;
}

bool DisplayManager::isOpened(const std::string name) noexcept
{
    return std::find(_openedWindows.begin(), _openedWindows.end(), name)
            != _openedWindows.end();
}

void DisplayManager::destroyWindows(const std::initializer_list
                                  <const std::string> &names) noexcept
{
    for(auto name : names)
        if (isOpened(name))
        {
            cv::destroyWindow(name.c_str());
            _openedWindows.erase(std::remove(
                _openedWindows.begin(), _openedWindows.end(), name),
                _openedWindows.end());
        }
}
