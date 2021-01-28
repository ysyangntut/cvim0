#pragma once

#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>

#include <opencv2/opencv.hpp>

// time measurement
double getCpusTime();
double getWallTime();

// files
bool fileExist(const std::string & fname);
bool dirExist (const std::string & dname);

// point/region selection
cv::Point2f selectPoint_zoom(cv::Mat img, int nLevel, float zoomFactor = 4.0f);
cv::Rect    selectRoi_zoom  (cv::Mat img, int nLevel, float zoomFactor = 4.0f);



const double pi = 3.141592653589793238462643383279502884;

