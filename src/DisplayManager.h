#ifndef DISPLAYMANAGER_H_
#define DISPLAYMANAGER_H_

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <utility>
#include <initializer_list>
#include <memory>

using std::vector;

using MatSharedPtr = std::shared_ptr<cv::Mat>;

class DisplayManager
{
public:
    DisplayManager() noexcept;

    static void showImages(const std::initializer_list <std::tuple
                           <const std::string, const MatSharedPtr, const int>>
                           &imagesWithWindowsNamesAndTimesToShow) noexcept;

    static void createWindows(const std::initializer_list
                              <const std::string> &names) noexcept;

    static vector<std::string> listOfOpenedWindows() noexcept;

    static void destroyWindows(const std::initializer_list
                               <const std::string> &names) noexcept;

    static bool isOpened(const std::string name) noexcept;

private:
    static vector<std::string> _openedWindows;
};



#endif /* DISPLAYMANAGER_H_ */

