#include "DisparityProvider.h"

DisparityProvider::DisparityProvider(std::string& pathToRectifyMaps) noexcept
{
    loadRectifyMaps(pathToRectifyMaps);
}

void DisparityProvider::loadRectifyMaps(std::string& pathRoRectifyMaps) noexcept
{
    cv::FileStorage fileStorage(pathRoRectifyMaps, cv::FileStorage::READ);
    fileStorage[RECTIFY_MAP_X1_TITLE] >> _rectifyMapXLeft;
    fileStorage[RECTIFY_MAP_Y1_TITLE] >> _rectifyMapYLeft;
    fileStorage[RECTIFY_MAP_X2_TITLE] >> _rectifyMapXRight;
    fileStorage[RECTIFY_MAP_Y2_TITLE] >> _rectifyMapYRight;
    fileStorage.release();
}
