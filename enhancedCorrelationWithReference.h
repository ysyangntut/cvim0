#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

int enhancedCorrelationWithReference
(InputArray image, InputArray templ,
	double ref_x, double ref_y,
	double init_x, double init_y, double init_rot,
	vector<double> &  dispAndRot,
	int motionType = cv::MOTION_EUCLIDEAN,
	cv::TermCriteria criteria =
	cv::TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 50, 0.001)
); 


//! This function finds movement and rotation by ECC method
//! If ECC returns an exception, this function tries a different
//! initial guess and run ECC again until it succeeds. The std_x,
//! std_y, and std_rot are the standard deviations of random trials
//! that are assumed to be normally distributed.
int enhancedCorrelationWithReferenceAndRepeatTrial
(InputArray image, InputArray templ,
    double ref_x, double ref_y,
    double init_x, double init_y, double init_rot,
    double std_x,  double std_y,  double std_rot,
    vector<double> &  dispAndRot,
    int motionType = cv::MOTION_EUCLIDEAN,
    cv::TermCriteria criteria =
    cv::TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 50, 0.001)
);

