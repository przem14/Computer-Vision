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

    void showImages(const std::initializer_list
                        <std::pair<const std::string&, const MatSharedPtr>>
                        &imagesWithWindowsNames) const noexcept;

    void createWindows(const std::initializer_list<const std::string> &names)
        noexcept;

    vector<std::string> listOfOpenedWindows() const noexcept;

    void destroyWindows(const std::initializer_list<const std::string> &names)
        noexcept;

    bool isOpened(const std::string name) const noexcept;

private:
    vector<std::string> _openedWindows;
};


#endif /* DISPLAYMANAGER_H_ */

