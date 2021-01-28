#pragma once

#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>

#include <opencv2/opencv.hpp>

// octave(matlab)-like function
std::vector<double> linspace(double a, double b, int n);
std::vector<float>  linspace(float a, float b, int n);
cv::Mat linspaceMat(double a, double b, int n);
cv::Mat linspaceMat(float a, float b, int n);
void meshgrid(float xa, float xb, int nx, float ya, float yb, int ny, cv::Mat & X, cv::Mat & Y);
void meshgrid(double xa, double xb, int nx, double ya, double yb, int ny, cv::Mat & X, cv::Mat & Y);
