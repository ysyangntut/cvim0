#ifndef CVIMPROJECT_H
#define CVIMPROJECT_H

#include <cmath>
#include <string>
#include <cstring>
#include <vector>
#include <list>
#include <map>
#include <random>

#include <opencv2/opencv.hpp>

#ifdef QT_GUI_LIB
#include <QFileDialog>
#endif

#include <Target.h>
#include <FileSeq.h>
#include <impro_util.h>
#include <pickAPoint.h>
#include <cvimCalibration.h>
#include <enhancedCorrelationWithReference.h>

class CvimProject
{
public:
    CvimProject();

    //! CvimProject::addFileSequence() adds a new file sequence by file
    //! names in c-style format returns number of files added. Returns
    //! a value <= 0 if there is something wrong.
    //! If returns -1, the sequence with this name has been existed, and
    //! is not added or replaced.
    int addFileSequence(
            std::string name,       //!< name of the new file sequence
            std::string fnames,     //!< file names of the files, requires a %d as it is c-style format
            int         startIndex, //!< the starting index of the %d in fnames
            int         nFiles     //!< number of files
            );

    //! CvimProject::addFileSeq() adds a new file sequence by file
    //! names in c-style format returns number of files added. Returns
    //! a value <= 0 if there is something wrong.
    //! If returns -1, the sequence with this name has been existed, and
    //! is not added or replaced.
    int addFileSeq(
            std::string name,       //!< name of the new file sequence
            std::string fnames,     //!< file names of the files, requires a %d as it is c-style format
            int         startIndex, //!< the starting index of the %d in fnames
            int         nFiles     //!< number of files
            )
    {
        FileSeq fsq;
        std::string theDir = directoryOfFullPathFile(fnames);
        std::string theFile = fileOfFullPathFile(fnames);
        fsq.setFilesByFormat(theDir, theFile, startIndex, nFiles);
        this->fileSeqs[name] = fsq;
        return 0;
    }

    //! CvimProject::fileSeq(std::string fsqName) returns the file sequence
    //! which is named fsqName in this->fileSequences
    FileSeq fileSeq(std::string fsqName) const
    {
        return this->fileSeqs.at(fsqName);
    }

    int addMat(std::string name,
            const cv::Mat & theMat)
    {
        this->mats[name] = theMat.clone();
        return 0;
    }

    int addMat(std::string name,
            const cv::Size & theSize)
    {
        cv::Mat theMat(1, 2, CV_32S);
        theMat.at<int>(0) = theSize.width;
        theMat.at<int>(1) = theSize.height;
        this->mats[name] = theMat.clone();
        return 0;
    }
    cv::Size matAsSize(std::string name) const
    {
        cv::Mat theMat = this->mats.at(name);
        cv::Size theSize;
        if (theMat.cols * theMat.rows >= 2 && theMat.type() == CV_32S)
        {
            theSize.width = theMat.at<int>(0);
            theSize.height = theMat.at<int>(1);
        } else {
            theSize = cv::Size(0, 0);
        }
        return theSize;
    }

    int addMat(std::string name,
               const cv::Point2f & point2f)
    {
        cv::Mat theMat(1, 2, CV_32F);
        theMat.at<float>(0) = point2f.x;
        theMat.at<float>(1) = point2f.y;
        this->mats[name] = theMat.clone();
        return 0;
    }
    cv::Vec2f matAsVec2f(std::string name) const
    {
        cv::Mat theMat = this->mats.at(name);
        cv::Vec2f theVec2f;
        if (theMat.cols * theMat.rows >= 2 && theMat.type() == CV_32F) {
            theVec2f[0] = theMat.at<float>(0);
            theVec2f[1] = theMat.at<float>(1);
        } else {
            theVec2f = cv::Vec2f(0.f, 0.f);
        }
        return theVec2f;
    }

    int addTarget(std::string name,
                  Target target)
    {
        this->targets[name] = target;
        return 0;
    }

    Target target(std::string name)
    {
        return this->targets.at(name);
    }

    int addTargetByGui(std::string name)
    {
        std::string filename;
        filename = uigetfile();
        cv::Mat img;
        img = cv::imread(filename);
        if (img.cols <= 0 || img.rows <= 0) {
            printf("# Error: addTargetByGui(): Selected file is not an image (%s)\n", filename.c_str());
            return -1;
        }

        // Select template
        cv::Rect roi;
        cv::Point2f ref;
        pickATemplate("Pick Target (point then ROI)", img, ref, roi);

        // build target
        Target target;
        target.setTarget(cv::Mat(0, 0, CV_8U), filename, roi, ref);

        // add target to project
        this->addTarget(name, target);

        cout << this->target(name) << endl;
        return 0;
    }



    cv::Mat mat(std::string name) const
    {
        return this->mats.at(name);
    }

    //! \brief CvimProject::printFileSequenceInfo() prints the information
    //! of all file sequences. Considering the numbers of files could be
    //! huge, only some of them (maybe the first and the last) are displayed.
    //! \return 0
    int printFileSequenceInfo() const;

    //! \brief CvimProject::printFileSequenceInfo(nameImgSeq) prints the
    //! informationof the specified file sequence. Considering the numbers
    //! of files could be huge, only some of them (maybe the first and the
    //! last) are displayed.
    //! \return 0
    int printFileSequenceInfo(std::string nameImgSeq) const;

    //! \brief CvimProject::getFileSequence() returns the file sequence
    //! in format of vector<string>.
    //! \return the file sequence
    const std::vector<std::string> & getFileSequence(std::string name) const;

    //! \brief CvimProject::getFileSequence() returns the file sequence
    //! in format of vector<string>.
    //! \return the file sequence
    std::vector<std::string> & getFileSequence(std::string name);

    //! \brief CvimProject::addImageTarget() adds an image target (i.e., a
    //! template, which is defined in an image). The image target (template)
    //! is conceptually a point which is searched or recognized by its
    //! surrounding region of interests (ROI). The ROI is a rectangular
    //! area and can (possibly in the future implementation) have a mask.
    //! A target is defined by an ROI of an image, a reference point
    //! (Point2f), and (optinally) a mask of the ROI.
    //! \details A target is defined as an element of matArrays. A target is
    //! a vector<Mat>, which contains the following elements:
    //! [0]: the template image (Mat(height, width, CV_8UC3 or CV_8U)
    //! [1]: the reference point (Mat(1, 1, CV_32FC2) wrt the template
    //! [2]: the mask which has the same size with the template image
    //! [3]: the reference point (Mat(1, 1, CV_32FC2) wrt a large image
    //! [4]: the file name of the large image (Mat(1, string length, CV_8U)
    //! \return Returns 0 if it is defined successfully.
    int addImageTarget();

    int calibrateCameraIntrinsic(
            const FileSeq & fileSeq,      //!< file sequence of calibation photos
            const cv::Size & patternSize, //!< numbers of corners
            const cv::Vec2f & blockSize,  //!< chessboard block size
            cv::Mat & cmat,               //!< camera matrix in type of (3, 3, CV_32F)
            cv::Mat & dvec,               //!< distortion coefficients in type of (1, n, CV_32F), n can be 4, 5, 8, or more
            cv::Mat & rvecs,              //!< rvec of each photo
            cv::Mat & tvecs,              //!< tvec of each photo
            cv::Mat & stdevIntrinsic,     //!< standard deviation of intrinsic parm.
            cv::Mat & stdevExtrinsic,     //!< standard deviation of extrinsic parm.
            cv::Mat & perViewErrors,      //!< per-view error of each photos
            bool showFoundCorners,        //!< if true, photos marked on corners will be displayed
            bool showUndistortedImages,   //!< if true, undistorted photos will be displayed
            bool printInfo,               //!< if true, debug information will be displayed
            int calibFlags = 0,           //!< calibration flag
            cv::TermCriteria criteria =   //!< criteria
            cv::TermCriteria(cv::TermCriteria::COUNT +
                             cv::TermCriteria::EPS,
                             30, DBL_EPSILON)
            ) const
    {
        return calibrateCameraFromImageFileNames(
                    fileSeq.allFullPathFileNames(),
                    patternSize,
                    blockSize,
                    cmat, dvec, rvecs, tvecs,
                    stdevIntrinsic, stdevExtrinsic, perViewErrors,
                    showFoundCorners, showUndistortedImages, printInfo,
                    calibFlags, criteria);
    }

    int trackingTargetsUsingEcc(
                const std::vector<Target> & targets,
                const FileSeq &             fileSeq,
                std::vector<cv::Mat>      & dispOfTargets)
    {
//        int nDofsFromEccMode[] = {-1, 2, 3, 6, 8};
        bool showGuessImg = true;
        bool showFoundImg = true;
        bool printInfo = true;
        int nTargets = targets.size();
        int nSteps = fileSeq.num_files();
        int nDofs = dispOfTargets[0].cols;
        // Start running loops of nSteps and nTargets
        for (int iStep = 0; iStep < nSteps; iStep++) {
            // wait and get image
            cv::Mat fullImg;
            while (true) {
                std::string filename = fileSeq.fullPathOfFile(iStep);
                fullImg = cv::imread(filename, cv::IMREAD_GRAYSCALE);
                if (fullImg.cols <= 0 || fullImg.rows <= 0) {
                    printf("# Waiting for file %s ...\n", filename.c_str());
                    std::this_thread::sleep_for (std::chrono::milliseconds(1000));
                    continue;
                } else
                    break;
            } // end of waiting for image file
            // for each target
            for (int iTarget = 0; iTarget < nTargets; iTarget++)
            {
                // clone a target image
                Target target = this->target("Target01");
                cv::Mat targetImg = target.getTargetImage(false, false).clone();
                if (targetImg.channels() == 3)
                    cv::cvtColor(targetImg, targetImg, cv::COLOR_BGR2GRAY);
                if (targetImg.channels() == 4)
                    cv::cvtColor(targetImg, targetImg, cv::COLOR_BGRA2GRAY);
                // Guess disp.
                cv::Vec3f guessDisp;
                if (iStep == 0 || iStep == 1) {
                    guessDisp[0] = dispOfTargets[iTarget].at<float>(0, 0);
                    guessDisp[1] = dispOfTargets[iTarget].at<float>(0, 1);
                    guessDisp[2] = dispOfTargets[iTarget].at<float>(0, 2);
                } else if (iStep == 2){
                    guessDisp[0] = 2 * dispOfTargets[iTarget].at<float>(1, 0)
                                 - 1 * dispOfTargets[iTarget].at<float>(0, 0);
                    guessDisp[1] = 2 * dispOfTargets[iTarget].at<float>(1, 1)
                                 - 1 * dispOfTargets[iTarget].at<float>(0, 1);
                    guessDisp[2] = dispOfTargets[iTarget].at<float>(iStep - 1, 2);
                } else {
                    guessDisp[0] = 3 * dispOfTargets[iTarget].at<float>(iStep - 1, 0)
                                 - 3 * dispOfTargets[iTarget].at<float>(iStep - 2, 0)
                                 + 1 * dispOfTargets[iTarget].at<float>(iStep - 3, 0);
                    guessDisp[1] = 3 * dispOfTargets[iTarget].at<float>(iStep - 1, 1)
                                 - 3 * dispOfTargets[iTarget].at<float>(iStep - 2, 1)
                                 + 1 * dispOfTargets[iTarget].at<float>(iStep - 3, 1);
                    guessDisp[2] = dispOfTargets[iTarget].at<float>(iStep - 1, 2);
                }
                std::vector<double> eccResult(5);
                if (showGuessImg)
                {
                    int w = target.getRect().width;
                    int h = target.getRect().height;
                    float refx = target.getRefPoint().x;
                    float refy = target.getRefPoint().y;
                    cv::Rect roi((int)(guessDisp[0] + .5f - refx),
                                 (int)(guessDisp[1] + .5f - refy),
                                 w, h);
                    cv::Mat imgShow = fullImg(roi).clone();
                    cv::resize(imgShow, imgShow, cv::Size(300, 300), 0., 0., cv::INTER_CUBIC);
                    cv::imshow("Guessed", imgShow);
                    cv::waitKey(10);
                }
                if (printInfo)
                {
                    printf("iStep %d, iTarget %d, Guessed: %f %f %f\n",
                           iStep, iTarget, guessDisp[0], guessDisp[1], guessDisp[2]);
                }
                int iTrial = 0;
                std::default_random_engine gen;
                std::normal_distribution<float> distDx(guessDisp[0], target.getRect().width );
                std::normal_distribution<float> distDy(guessDisp[1], target.getRect().height);
                std::normal_distribution<float> distRt(guessDisp[2], 5.f);
                while (true) {
                    iTrial++;
                    float runGuessDisp[3];
                    if (iTrial == 1) {
                        runGuessDisp[0] = guessDisp[0];
                        runGuessDisp[1] = guessDisp[1];
                        runGuessDisp[2] = guessDisp[2];
                    } else {
                        runGuessDisp[0] = distDx(gen);
                        runGuessDisp[1] = distDy(gen);
                        runGuessDisp[2] = distRt(gen);
                    }
                    enhancedCorrelationWithReference(
                                    fullImg,
                                    targetImg,
                                    target.getRefPoint().x,
                                    target.getRefPoint().y,
                                    runGuessDisp[0],
                                    runGuessDisp[1],
                                    runGuessDisp[2],
                                    eccResult);
                    if (eccResult[0] == 0.0) {
                        printf("ECC failed with the given guess: iStep %d, iTarget %d, Guessed: %f %f %f\n",
                               iStep, iTarget, runGuessDisp[0], runGuessDisp[1], runGuessDisp[2]);
                        cout.flush();
                        continue;
                    }
                }

                if (printInfo)
                {
                    printf("iStep %d, iTarget %d, Found: %f %f %f\n",
                           iStep, iTarget, eccResult[0], eccResult[1], eccResult[2]);
                }
                if (showFoundImg)
                {
                    int w = target.getRect().width;
                    int h = target.getRect().height;
                    float refx = target.getRefPoint().x;
                    float refy = target.getRefPoint().y;
                    cv::Rect roi((int)(eccResult[0] + .5f - refx),
                                 (int)(eccResult[1] + .5f - refy),
                                 w, h);
                    cv::Mat imgShow = fullImg(roi).clone();
                    cv::resize(imgShow, imgShow, cv::Size(300, 300), 0., 0., cv::INTER_CUBIC);
                    cv::imshow("Found", imgShow);
                    cv::waitKey(0);
                }
                dispOfTargets[iTarget].at<float>(iStep, 0) = eccResult[0];
                dispOfTargets[iTarget].at<float>(iStep, 1) = eccResult[1];
                dispOfTargets[iTarget].at<float>(iStep, 2) = eccResult[2];
                cv::destroyAllWindows();
             } // end of iTarget loop
        } // end of iStep loop
        return 0;
    } // end of trackingTargetsUsingEcc()

    static void testMain();

private:
    std::map<std::string, std::vector<std::string> > fileSequences;
    std::map<std::string, FileSeq> fileSeqs;
//    std::map<std::string, std::vector<cv::Mat> >     targets;
    std::map<std::string, Target>                    targets;
    std::map<std::string, std::vector<std::vector<cv::Mat> > >
                                                     matArrayArrays;
    std::map<std::string, std::vector<cv::Mat> >     matArrays;
    std::map<std::string, cv::Mat>                   mats;
};

#endif // CVIMPROJECT_H
