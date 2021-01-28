#ifndef CALIBRATIONFUNCTIONS_H
#define CALIBRATIONFUNCTIONS_H

#include <iostream>

#include <vector>
#include <opencv2/opencv.hpp>

int printIntrinsic(
        const cv::Mat & cmat,
        const cv::Mat & dvec,
        const cv::Mat & stdevIntrinsic,
        const cv::Mat & perViewErrors = cv::Mat(0, 0, CV_64F),
        std::ostream & sout = std::cout);

int calibrateCameraFromImageFileNames(
        std::vector<std::string> fullFileNames, //!< vector of full-path file names
        cv::Size patternSize, //!< pattern size (numbers of corners along x and y) of chessboard
        cv::Vec2f calibBlockSize, //!< width and height of a block on chessboard
        cv::Mat & cmat,
        cv::Mat & dvec,
        cv::Mat & rvecs,
        cv::Mat & tvecs,
        cv::Mat & stdevIntrinsic,
        cv::Mat & stdevExtrinsic,
        cv::Mat & perViewErrors,
        bool showCorners = false,
        bool showUndistorted = false,
        bool printInfo = false,
        int flags = 0,
        cv::TermCriteria criteria =
        cv::TermCriteria(cv::TermCriteria::COUNT +
                         cv::TermCriteria::EPS,
                         30, DBL_EPSILON)
        );


#endif // CALIBRATIONFUNCTIONS_H
