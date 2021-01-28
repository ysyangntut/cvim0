#ifndef CONSTRAINED3DPOSITIONING_H
#define CONSTRAINED3DPOSITIONING_H

#include "opencv2/core/core.hpp"

// {xp} = {xc} + a1 * {xcVec1} + a2 * {xcVec2} 

int plane_constrained3dPositioning(
  const cv::Mat & camMat,      // 3x3 camera matrix
  const cv::Mat & disVec,      // 4x1 or 5x1 (or Nx1) distortion coefficients
  const cv::Mat & rmat,        // 3x3 Rotation matrix of this camera (wrt global coord.).
  const cv::Mat & tvec,        // 3x1 or 1x3 Translation vector of the camera.
  const cv::Mat & imgPoints,   // Nx1 image points on photo. N is number of points to track.
  const cv::Mat & xc,          // Nx1 3-channel constraining points, which define surfaces
  const cv::Mat & xcVec1,      // Nx1 3-channel vector 1 of each tracking point, which define surfaces
  const cv::Mat & xcVec2,      // Nx1 3-channel vector 2 of each tracking point, which define surfaces
		cv::Mat & xp           // Nx1 3-channel positioned points, will be on the surface
	);

int plane_constrained3dPositioning_newton(
	const cv::Mat & camMat,      // 3x3 camera matrix
	const cv::Mat & disVec,      // 4x1 or 5x1 (or Nx1) distortion coefficients
	const cv::Mat & rmat,        // 3x3 Rotation matrix of this camera (wrt global coord.).
	const cv::Mat & tvec,        // 3x1 or 1x3 Translation vector of the camera.
	const cv::Mat & imgPoints,   // Nx1 image points on photo. N is number of points to track.
	const cv::Mat & xc,          // Nx1 3-channel constraining points, which define surfaces
	const cv::Mat & xcVec1,      // Nx1 3-channel vector 1 of each tracking point, which define surfaces
	const cv::Mat & xcVec2,      // Nx1 3-channel vector 2 of each tracking point, which define surfaces
	cv::Mat & xp           // Nx1 3-channel positioned points, will be on the surface
);

#endif // CONSTRAINED3DPOSITIONING_H

