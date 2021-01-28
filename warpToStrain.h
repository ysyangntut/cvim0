#pragma once

#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>

#include <opencv2/opencv.hpp>

// namespace cvim {

//! A function that returns matrix to converts strains to displacements
///
///
cv::Mat deformToDxyMat(float w, float h);


//! A function that converts a warp matrix to strains
//! with respect to a given reference point. All deformations
//! (including rotation) refer to the given point.
/*!
 * This function converts a 2x3 or 3x3 warp matrix to
 * rigid body motions (dx, dy, rotation), strains (exx,
 * eyy, and exy), and two hourglassing deformations
 * (hgx and hgy).
 * This function is designed for engineers as the warp
 * matrix in computer vision has less meaningful information
 * for engineers.
 * Rigid body motion effect is well considered. If you input
 * a pure-rotation rigid-body motion, you will get
 * zero strains with a rotation, even if the rotation is large.
 * (Note: simple linear finite element formulation would give
 * you a strain field). For example, if you input
 * [cos(pi/6) -sin(pi/6) 0; sin(pi/6) cos(pi/6) 0; 0 0 1],
 * which is a pure rigid body motion, you will get {0, 0, 30,
 * 0, 0, 0, 0, 0}.
 * Here is an example. If you input an OpenCV matrix of
 * [1  0.1  3; 0 1 4; 0 0 1], you will get
 * {3, 4, -2.9, 0.0038, -0.0012, 0.09988, 0, 0}, indicating
 * translation of (3, 4), rotation -2.9 degrees of rotation,
 * exx of 0.0038, eyy of -0.0012, and shear strain of 0.09988,
 * without hourglassing deformations.
 */
/*!
  \param W the warp matrix, which can be calculated by
           cv::findTransformECC(), cv::getAffineTransform(), or
           cv::getPerspectiveTransform(). It can be 2x3 or 3x3,
           CV_32F or CV_64F. The calculation is 32F based. If W is
           CV_64F, a CV_32F version is cloned for calculation.
  \param ref the reference point in type of cv::Point2f
  \return the strain vector, in type of std::vector<float>.
          [0]: rigid-body translation x
          [1]: rigid-body translation y
          [2]: rigid-body rotation (in the unit of degrees,
               right-hand rule)
          [3]: strain x (exx)
          [4]: strain y (eyy)
          [5]: shear strain (gamma, du/dy + dv/dx)
          [6]: horizontal hourglassing deformation. If it is +1,
               the lower/upper boundary strains are -1/+1.
          [7]: vertical hourglassing deformation. If it is +1,
               the left/right boundary strains are -1/+1.
  */
std::vector<float> deformFromWarp(const cv::Mat & W,
                                  cv::Point2f ref = cv::Point2f(0.f, 0.f));



// } // end of namespace cvim
