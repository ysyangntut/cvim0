#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>

#include <cvimUtil.h>

char buf[1000];
//const double pi = 3.14159265358979323846;
//const float pif = 3.14159265358979323846f;

using namespace std;

// namespace cvim {

/*!
  \fn bool fileExist(std::string fileName)

  Returns true if the file path \a fileName exists and is a file.
  Returns false if \a fileName is a directory or it does not even exist.

  \param fileName the path (suggested to be full path) of the file name

  \sa dirExist()
*/
bool fileExist(const std::string & fileName) {
    struct stat info;
    if (stat(fileName.c_str(), &info) != 0)
        return false; // does not exist
    else if (info.st_mode & S_IFDIR)
        return false; // is a directory
    return true; // is a file
}

/*!
  \fn bool dirExist(std::string dirName)

  Returns true if the directory path \a dirName exists and is a directory.
  Returns false if \a dirName is a file or it does not even exist.

  \param dirName the path (suggested to be full path) of the directory name

  \sa fileExist()
*/
bool dirExist (const std::string & dirName) {
    struct stat info;
    if (stat(dirName.c_str(), &info) != 0)
        return false; // does not exist
    else if (info.st_mode & S_IFDIR)
        return true; // is a directory
    return false; // is a file
}

// #include <sys/types.h>
// #include <sys/stat.h>
//
// struct stat info;
//
// if( stat( pathname, &info ) != 0 )
//     printf( "cannot access %s\n", pathname );
// else if( info.st_mode & S_IFDIR )  // S_ISDIR() doesn't exist on my windows
//     printf( "%s is a directory\n", pathname );
// else
//     printf( "%s is no directory\n", pathname );


cv::Point2f selectPoint_zoom(cv::Mat img, int nLevel, float zoomFactor)
{
    cv::Point2f pick = cv::Point2f(img.cols * 0.5f - .5f, img.rows * 0.5f - 0.5f);
    cv::Mat imgSelected = img;
    cv::Rect roi;
    float fac = zoomFactor;
    float fac_x = 1.0f, fac_y = 1.0f;
    float ori_x = 0.0f, ori_y = 0.0f;
    for (int iLevel = 0; (iLevel < nLevel || nLevel <= 0); iLevel++)
    {
        roi = cv::selectROI(imgSelected, true, false);
        if (roi.x < 0 || roi.y < 0 || roi.width < 1 || roi.height < 1) {
            break;
        }
        pick.x = ori_x + (roi.x + roi.width * 0.5f) / fac_x - .5f;
        pick.y = ori_y + (roi.y + roi.height * 0.5f) / fac_y - .5f;
        imgSelected(roi).copyTo(imgSelected);
        cv::resize(imgSelected, imgSelected, cv::Size(0, 0), fac, fac, cv::INTER_NEAREST);
        ori_x += roi.x / fac_x;
        ori_y += roi.y / fac_y;
        fac_x *= fac;
        fac_y *= fac;
    }

    // destroy window
    cv::destroyWindow("ROI selector");

    return pick;
}

cv::Rect selectRoi_zoom(cv::Mat img, int nLevel, float zoomFactor)
{
    cv::Point2f pick = cv::Point2f(img.cols * 0.5f - .5f, img.rows * 0.5f - 0.5f);
    cv::Mat imgSelected = img;
    cv::Rect roi(0, 0, img.cols, img.rows);  /// the roi returned from OpenCV (could be scaled)
    cv::Rect roi_ori = roi; /// the roi that in original scale
    float fac = zoomFactor;
    float fac_x = 1.0f, fac_y = 1.0f;
    float ori_x = 0.0f, ori_y = 0.0f;
    for (int iLevel = 0; (iLevel < nLevel || nLevel <= 0); iLevel++)
    {
        cv::Rect retRoi;
        retRoi = cv::selectROI(imgSelected, true, false);
        if (retRoi.x < 0 || retRoi.y < 0 || retRoi.width < 1 || retRoi.height < 1) {
            fac_x /= fac;
            fac_y /= fac;
            break;
        }
        roi = retRoi;
        pick.x = ori_x + (roi.x + roi.width * 0.5f) / fac_x - .5f;
        pick.y = ori_y + (roi.y + roi.height * 0.5f) / fac_y - .5f;
        imgSelected(roi).copyTo(imgSelected);
        cv::resize(imgSelected, imgSelected, cv::Size(0, 0), fac, fac, cv::INTER_NEAREST);
        ori_x += roi.x / fac_x;
        ori_y += roi.y / fac_y;
        if (iLevel == nLevel - 1) break;
        fac_x *= fac;
        fac_y *= fac;
    }

    // destroy window
    cv::destroyWindow("ROI selector");

    // calculate the roi_ori in original scale
    roi_ori.x = (int) (ori_x + 0.5f);
    roi_ori.y = (int) (ori_y + 0.5f);
    roi_ori.width = (int) (roi.width / fac_x + .5f);
    roi_ori.height = (int) (roi.height / fac_y + .5f);
    return roi_ori;
}




// } // end of namespace cvim
