#pragma once

#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <impro_util.h>
// for drawPointOnImage()

#include <cvim.h>

// namespace cvim {

/*!
    \class Target
    \brief The Target class defines a target point by giving an
    image, a rectangular ROI of the image (in type of cv::Rect), a
    reference point of the ROI (in type of cv::Point2f), and an optional
    mask of the ROI.

    \reentrant
    The Target class defines a target point by giving an
    image, a rectangular template of the image (in type of cv::Rect), a
    reference point of the template (in type of cv::Point2f), and an optional
    mask of the template (in type of cv::Mat(hTmplt(), wTmplt(), maskType());
    the hTmplt() and the wTmplt() are the height and the width of the template,
    respectively.
    As to the type of the mask, according to the OpenCV documentation
    of matchTemplate():
    "It must have the same size as templ. It must either have the same
    number of channels as template or only one channel, which is then
    used for all template and image channels. If the data type is CV_8U,
    the mask is interpreted as a binary mask, meaning only elements
    where mask is nonzero are used and are kept unchanged independent
    of the actual mask value (weight equals 1). For data tpye CV_32F,
    the mask values are used as weights. The exact formulas are
    documented in TemplateMatchModes. ..."

    \sa
*/
class Target {
public:
    Target();

    //! Target::setTarget() defines the target by giving the image
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
    void setTarget(cv::Mat img,
                   std::string filename = std::string(""),
                   cv::Rect rectTarget = Target::rectOfFullImage,
                   cv::Point2f ref = Target::centerOfRectTarget,
                   cv::Mat mask = cv::Mat(0, 0, CV_8U));

    //! Target::setMask() sets the mask of the target. It also
    //! checks the size and # of channels of the mask.
    void setMask(cv::Mat mask) {
        this->mask = mask;
        // make sure the rectTarget size is correct.
        this->setRectTargetToFullIfNeeded();
        // resize the mask if needed.
        if (mask.rows != this->rectTarget.height ||
            mask.cols != this->rectTarget.width) {
            cv::resize(mask, this->mask, cv::Size(this->rectTarget.width,
                       this->rectTarget.height), 0, 0, cv::INTER_CUBIC);
        }
        // check the number of channels are consistent with the image
        int nch;
        if (this->img.rows > 0 && this->img.cols > 0) {
            nch = img.channels();
        } else {
            cv::Mat tmpImg = cv::imread(this->filename);
            if (tmpImg.cols <= 0 || tmpImg.rows <= 0) {
                std::cerr << "Warning: Target::setMask(): Cannot read image. Having no idea number of channels of this image. Assuming it is equal to the inputted mask.\n";
                nch = mask.channels();
            } else {
                nch = tmpImg.channels();
            }
        }
        if (mask.channels() != nch) {
            std::cerr << "Warning: Target::setMask(): Mask has different number of channels with that of the target image.\n";
            std::cerr << "Warning: Target::setMask(): We will try to make them the same. But so far we do not handle float (CV_32FCx) mask correctly.\n";
            if (nch == 3 && mask.channels() == 1) {
                cv::cvtColor(mask, this->mask, cv::COLOR_GRAY2BGR);
            }
            if (nch == 4 && mask.channels() == 1) {
                cv::cvtColor(mask, this->mask, cv::COLOR_GRAY2BGRA);
            }
            if (nch == 1 && mask.channels() == 3) {
                cv::cvtColor(mask, this->mask, cv::COLOR_BGR2GRAY);
            }
            if (nch == 1 && mask.channels() == 4) {
                cv::cvtColor(mask, this->mask, cv::COLOR_BGR2GRAY);
            }
        }
    }

    //! Target::setRefPoint() sets the reference point of the target
    //! relative to the upper-left corner of the target image.
    //! If Target::centerOfRectTarget is given, it is set to
    //! the center of the rectTarget (i.e., (rectTarget.w /2.f - .5f,
    //! rectTArget.h / 2.f - .5)).
    void setRefPoint(cv::Point2f center = centerOfRectTarget)
    {
        if (center == Target::centerOfRectTarget) {
            // make sure the rectTarget size is correct.
            this->setRectTargetToFullIfNeeded();
            this->refPoint.x = this->rectTarget.width / 2.f - .5f;
            this->refPoint.y = this->rectTarget.height / 2.f - .5f;
        } else
            this->refPoint = center;
    }

    //! Target::setRectTarget() sets the rectTarget of the target.
    //! The image of the target could be large, and the target image
    //! is only a small part of it.
    void setRectTarget(cv::Rect rect = Target::rectOfFullImage) {
        this->rectTarget = rect;
        this->setRectTargetToFullIfNeeded();
    }

    //! \brief Target::getTargetImage() returns a clone of the
    //! rectangular image of the target. Mask is not applied on the image.
    //! \param mask         [in] mask image if it is true
    //! \param drawRefPoint [in] draw a cross at the reference point
    //! \return the target image
    cv::Mat getTargetImage(bool mask = true, bool drawRefPoint = true) {
        cv::Mat tmpImg;
        this->setRectTargetToFullIfNeeded();
        this->centerRefIfNeeded();
        // get full image, either from this->img or from file
        if (this->img.cols > 0 && this->img.rows > 0) {
            tmpImg = this->img;
        } else {
            tmpImg = cv::imread(this->filename);
            int w = tmpImg.cols, h = tmpImg.rows;
            if (w < this->rectTarget.x + this->rectTarget.width ||
                h < this->rectTarget.y + this->rectTarget.height) {
                std::cerr << "Warning: Target::getTargetImage(): Image from file is not large enough for rectTarget.\n";
                return cv::Mat(0, 0, tmpImg.type());
            }
        }
        // crop and clone the target from the image
        cv::Mat tmpTar = tmpImg(this->rectTarget).clone();
        // mask
        if (mask == true) {
            cv::Mat tmpMask = this->getMask();
            cv::Mat tmpMasked = tmpTar.clone();
            if (this->debug == true) {
                printf("Image: %d %d %d.\n", tmpImg.rows, tmpImg.cols, tmpImg.channels());
                printf("Mask: %d %d %d.\n", tmpMask.rows, tmpMask.cols, tmpMask.channels());
                cv::imshow("Image", tmpTar);
                cv::imshow("Mask", tmpMask);
            }

            if (tmpTar.cols == tmpMask.cols &&
                tmpTar.rows == tmpMask.rows &&
                tmpTar.channels() == tmpMask.channels())
            {
                cv::bitwise_and(tmpTar, tmpMask, tmpMasked);
            } else {
                std::cerr << "Warning: getTargetImage(): Image and mask are not the same size (or depth).\n";
                printf("Image: %d x %d x %d\n", tmpTar.rows, tmpTar.cols, tmpTar.channels());
                printf("Mask: %d x %d x %d\n", tmpMask.rows, tmpMask.cols, tmpMask.channels());
                printf("Masking is not done.\n");
            }
            tmpMasked.copyTo(tmpTar);
        } // end of if mask
        // draw a cross at the reference point
        if (drawRefPoint) {
            int crossSize = tmpTar.cols;
            int thickness = 4;
            cv::Scalar color = cv::Scalar(255, 255, 255);
            float alpha = 0.5f;
            drawPointOnImage(tmpTar, this->refPoint, "+",
                crossSize, thickness,
                color, alpha, -1 /* put text */, 2 /* shift */);
            thickness = 2;
            color = cv::Scalar(0, 0, 0);
            drawPointOnImage(tmpTar, this->refPoint, "+",
                crossSize, thickness,
                color, alpha, -1 /* put text */, 2 /* shift */);
        } // end of if drawRefPoint
        return tmpTar;
    }

    cv::Point2f getRefPoint() const
    {
        return this->refPoint;
    }

    cv::Rect getRect() const
    {
        return this->rectTarget;
    }

    //! \brief Target::getMask() returns a clone of the mask. If this
    //! target does not have a mask, it returns a zero sized Mat.
    cv::Mat getMask() {
        //
        return this->mask.clone();
    }

    // Check if rectTarget is equal to the predefined Target::rectOfFullImage.
    // If so, make it the full range of the image.
    void setRectTargetToFullIfNeeded() {
        if (this->rectTarget == Target::rectOfFullImage)
        {
            int w, h;
            if (this->img.cols >= 1 && this->img.rows >= 1) {
                w = this->img.cols;
                h = this->img.rows;
            } else if (filename.length() > 0) {
                cv::Mat theImg = cv::imread(filename);
                w = theImg.cols;
                h = theImg.rows;
                if (w <= 0 || h <= 0) {
                    std::cerr << "Target::centerRefIfNecesary(): No image can be found.\n";
                }
            }
            // set the full range
            this->rectTarget = cv::Rect(0, 0, w, h);
        }
    }

    // Check if refPoint is is equal to the predefined Target::centerOfFullImage.
    // If so, make it the center of the image.
    void centerRefIfNeeded() {
        if (this->refPoint == Target::centerOfRectTarget) {
            // Set the reference point (refPoint) to the center of rectTarget.
            // make sure rectTarget is correct
            this->setRectTargetToFullIfNeeded();
            this->refPoint.x = this->rectTarget.width / 2.f - .5f;
            this->refPoint.y = this->rectTarget.height / 2.f - .5f;
        }
        return;
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
      \param center [in] the center of the circle in data type of cv::Point.
                         If either coordinate is less than -1e6, it will be set
                         to (w / 2, h / 2).
      \param radius [in] the radius of the circle. If it is <= 0, it will be
                         set to min.(w / 2 + 1, h / 2 + 1)
     */
    static cv::Mat circleMask(int w, int h, int type,
                       cv::Point center = Target::centerOfRectTarget,
                       int radius = -1);

    //! \brief Target::chessboard(int w, int h, cv::Size patternSize, type) returns
    //! a chessboard pattern which image size is w by h with the given
    //! pattern size. This function only supports CV_8U/32F C1/C3/C4 type at present.
    //!
    static cv::Mat chessboard(int w, int h, cv::Size patternSize, int type) {
        int inRows = patternSize.width;
        int inCols = patternSize.height;
        int recWidth = w / inRows;
        int recHeight = h / inCols;
        cv::Mat imgCb = cv::Mat::zeros(h, w, type);
        cv::Scalar color;
        if (type == CV_8U) color = cv::Scalar(255);
        else if (type == CV_8UC3) color = cv::Scalar(255, 255, 255);
        else if (type == CV_8UC4) color = cv::Scalar(255, 255, 255, 255);
        else if (type == CV_32F) color = cv::Scalar(1.f);
        else if (type == CV_32FC3) color = cv::Scalar(1.f, 1.f, 1.f);
        else if (type == CV_32FC4) color = cv::Scalar(1.f, 1.f, 1.f, 1.f);
        else {
            std::cerr << "Warning: Target:chessboard() does not support type " << type << "\n";
            color = cv::Scalar(0);
        }
        for (int i = 0; i < inCols; i++) {
            for (int j = 0; j < inCols; j++) {
                if ((i + j) % 2 == 0) {
                    cv::rectangle(imgCb, cv::Point(j * recWidth, i * recHeight),
                                  cv::Point((j + 1) * recWidth - 1, (i + 1) * recHeight - 1),
                                  color,
                                  cv::FILLED, 8, 0);
                }
            }
        }
        return imgCb;
    }


    //! \brief Target::write() function allows cv::FileStorage to output a Target object.
    /*!
    //! For example:
    //! {
    //!     cv::FileStorage fs("aFile.xml", cv::FileStorage::WRITE);
    //!     Target t;
    //!     fs << "MyTarget" << t;
    //!     fs.release()
    //! }
    */
    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);

    friend std::ostream& operator<<(std::ostream &os, const Target &t);

    static const cv::Point2f centerOfRectTarget; // = cv::Point2f(-1.e6, -1e6);
    static const cv::Rect rectOfFullImage;       // = cv::Rect(0, 0, 0, 0)

    void setDebugMode(bool debug) {
        this->debug = debug;
    }

protected:
    cv::Mat     img;       //!< the image that contains the target
    std::string filename;  //!< the full-path file name of the image that contains the target
    cv::Rect    rectTarget; //!< the rectangle range of the template of the target in the image
    cv::Point2f refPoint;  //!< the point of the target
    cv::Mat     mask;      //!< mask of the target, same size with img
    bool        debug;     //!< debug mode is on or off

private:

};

static void write(cv::FileStorage& fs, const std::string&, const Target& target)
{
    target.write(fs);
}

static void read(const cv::FileNode& node, Target& target, const Target& default_value = Target())
{
  if(node.empty())
    target = default_value;
  else
    target.read(node);
}

static std::ostream& operator<<(std::ostream& out, const Target& t)
{
  out << "{ ";
  out << "Target: Image size in memory: (" << t.img.rows << ", "
      << t.img.cols << ") " << "Mat type: " << t.img.type() << "). ";
  out << t.img.channels() << " ch. Element size (byte): "
      << t.img.elemSize1() << "\n";
  out << "        filename: " << t.filename << "\n";
  out << "        rect: " << t.rectTarget << "\n";
  out << "        refPoint: " << t.refPoint << "\n";
  out << "        Mask size in memory: (" << t.mask.rows << ", "
      << t.mask.cols << ") " << "Mat type: " << t.mask.type() << "). ";
  out << "}\n";
  return out;
}







// } // end of namespace cvim
