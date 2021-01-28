#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <Target.h>

const cv::Point2f Target::centerOfRectTarget = cv::Point2f(-1.e6, -1e6);
const cv::Rect Target::rectOfFullImage = cv::Rect(0, 0, 0, 0);

#include <cvim.h>

// char buf[1000];

//namespace cvim {



Target::Target()
{
    // Initialize member data
    this->img = cv::Mat(0, 0, CV_8UC3);
    this->filename = std::string("");
    this->rectTarget = cv::Rect(0, 0, 0, 0);
    this->refPoint = cv::Point2f(-1e6f, -1e6f);
    this->mask = cv::Mat(0, 0, CV_8UC3);
    this->debug = false;
    // static const member initialization
    if (this->debug == true) {
        printf("Target::Target() tested.\n"); fflush(stdout);
    }
}


//! Defines the target by giving the image
/*!
  This function defines a target by giving an image. It assumes
  the reference point is at the center of the image. This function
  also clears the file name, sets the template range (rectTarget)
  to the entire img, i.e., Rect(0, 0, w, h), and clear the mask.
  \param img       [in] entire background that contains the target. If
                        either height or width is <= 0, this image will
                        be read from the filename when needed.
  \param filename  [in] full path of the file that contains the target.
                        If img is given, or filename.length() is <= 0,
                        this argument will be ignored.
  \param rectTarget [in] the Rect of the template of the target. If either
                        width or height <= zero, the img will be a clone
                        (not refering to other image), and the rectTarget
                        will be set to the entire image, that is,
                        cv::Rect(0, 0, w, h) where w and h are the size
                        of img.
  \param ref       [in] the reference point of the target. It is related
                        to the upper-left point of the template, not the
                        entire image. If either the .x or .y is <= -1.e6,
                        it will be set to the center of the template
                        ((w-1)/2., (h-1)/2.) where w and h are the size
                        of the template.
  \param mask      [in] the mask of the image. If either width or height
                        is <= 0, mask is not used.
*/
void Target::setTarget(cv::Mat img,
               std::string filename,
               cv::Rect rectTarget, // = cv::Rect(0, 0, 0, 0),
               cv::Point2f ref, // = cv::Point2f(-9.e6, -9.e6),
               cv::Mat mask) // = cv::Mat(0, 0, CV_8U))
{
    // Set image
    this->img = img;
    // Set file name
    this->filename = filename;
    // Set rectTarget (roi)
    this->rectTarget = rectTarget;
    if (this->rectTarget.area() <= 0) {
        // rectTarget being zero indicates the full range of the image
        this->rectTarget.x = this->rectTarget.y = 0;
        this->rectTarget.width = img.cols;
        this->rectTarget.height = img.rows;
    }
    if (this->rectTarget.area() <= 0 && img.cols * img.rows <= 0) {
        // rectTarget being zero and image being empty indicate the full range of the image file
        cv::Mat tmpImg = cv::imread(filename);
        this->rectTarget.x = this->rectTarget.y = 0;
        this->rectTarget.width = tmpImg.cols;
        this->rectTarget.height = tmpImg.rows;
    }
    if (this->rectTarget.area() <= 0) {
        std::cerr << "Warning: Target::setTarget(): It gets an invalid target which has no image source.\n";
        std::cerr << "Warning: Target::setTarget(): The image is empty. The file name is not an image: " << filename << std::endl;
        this->refPoint = Target::centerOfRectTarget; // cv::Point2f(-1e9, -1e9);
        this->mask = cv::Mat(0, 0, CV_8U);
        return;
    }
    // Set refPoint
    this->refPoint = ref;
    if (ref == Target::centerOfRectTarget) {
        ref.x = rectTarget.width / 2.0f - 0.5f;
        ref.y = rectTarget.height / 2.0f - 0.5f;
    }
    // Set mask
    this->mask = mask;

}

//! Returns a circle mask image.
/*!
  This function returns a circle mask image. This function is static.
  \param w      [in] the width of the image
  \param h      [in] the height of the iamge
  \param type   [in] data type of the image, should be CV_8UCx or CV_32FCx.
                     For CV_8UCx, pixel inside the circle is 255, otherwize
                     is 0. For CV_32FCx, pixel inside the circle is 1.f,
                     outside is 0.0f.
  \param center [in] the center of the circle in data type of cv::Point2f.
                     If either coordinate is less than -1e6, it will be set
                     to ((w-1)/2., (h-1)/2.).
  \param radius [in] the radius of the circle. If it is <= 0.0f, it will be
                     set to min.((w-1)/2., (h-1)/2.)
 */
cv::Mat Target::circleMask(int w, int h, int type,
                           cv::Point center, //  = Target::centerOfRectTarget
                           int radius) // = 0);
{
    cv::Mat theMask = cv::Mat::zeros(h, w, type);
    cv::Scalar color;
    if (type == CV_8U)          color = cv::Scalar(255);
    else if (type == CV_8UC3)   color = cv::Scalar(255, 255, 255);
    else if (type == CV_8UC4)   color = cv::Scalar(255, 255, 255, 0);
    else if (type == CV_32FC1)  color = cv::Scalar(1.f);
    else if (type == CV_32FC3)  color = cv::Scalar(1.f, 1.f, 1.f);
    else if (type == CV_32FC4)  color = cv::Scalar(1.f, 1.f, 1.f, 0.f);
    else {
        std::cerr << "Warning: Target::circleMask() only supports type [8U/32F][C1/C3/C4].\n";
        return theMask;
    }
    if (center.x <= Target::centerOfRectTarget.x || center.y <= Target::centerOfRectTarget.y) {
        center = cv::Point(w / 2, h / 2); // we do not need to -0.5 as it will be rounded down.
    }
    if (radius <= 0) {
        radius = std::min(w / 2 + 1, h / 2 + 1);
    }
    cv::circle(theMask, center, radius, color, cv::FILLED, 8, 0);
    return theMask;
}




void Target::write(cv::FileStorage& fs) const {
    fs << "{";
    fs << "Target_img" << this->img;
    fs << "Target_filename" << this->filename;
    fs << "Target_rectTarget" << this->rectTarget;
    fs << "Target_refPoint" << this->refPoint;
    fs << "Target_mask" << this->mask;
    fs << "}";
}
void Target::read(const cv::FileNode& node) {
    node["Target_img"] >> this->img;
    node["Target_filename"] >> this->filename;
    node["Target_rectTarget"] >> this->rectTarget;
    node["Target_refPoint"] >> this->refPoint;
    node["Target_mask"] >> this->mask;
}



// } // end of namespace cvim
