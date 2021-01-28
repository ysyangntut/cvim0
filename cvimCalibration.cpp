#include <cvimCalibration.h>

#include <iostream>

#include <vector>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

int printIntrinsic(
        const cv::Mat & cmat,
        const cv::Mat & dvec,
        const cv::Mat & stdevIntrinsic,
        const cv::Mat & perViewErrors,
        std::ostream & sout)
{
    char buf[1000];
    sout << "# ==========================\n";
    sout << "# Camera calibration result:\n";
    snprintf(buf, 1000, "# Fx is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(0, 0), stdevIntrinsic.at<double>(0));
    sout << buf;
    snprintf(buf, 1000, "# Fy is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(1, 1), stdevIntrinsic.at<double>(1));
    sout << buf;
    snprintf(buf, 1000, "# Cx is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(0, 2), stdevIntrinsic.at<double>(2));
    sout << buf;
    snprintf(buf, 1000, "# Cy is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(1, 2), stdevIntrinsic.at<double>(3));
    sout << buf;
    for (int i = 0; i < dvec.cols * dvec.rows; i++) {
        snprintf(buf, 1000, "# Dist. coef. [%d] is %12.4f"
                            "(stdev: %12.4f)\n", i,
             dvec.at<double>(i), stdevIntrinsic.at<double>(i + 4));
        sout << buf;
    }
    sout << "# Calibration per-view errors: \n";
    for (int i = 0; i < perViewErrors.cols * perViewErrors.rows; i++)
    {
        snprintf(buf, 1000, "#   Per-view error [%d] is %12.4f\n",
                 i, perViewErrors.at<double>(i));
        sout << buf;

    }
    sout << "# End of calibration info.\n";
    return 0;
}

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
        bool showCorners,
        bool showUndistorted,
        bool printInfo,
        int flags,
        cv::TermCriteria criteria
        )
{
    bool standAloneInteraction = false;
    // Step 1: Get the photos
    int numCalibPhotos;
    std::vector<std::string> filenames;
    std::vector<std::vector<cv::Vec2f>> imgPoints;
    std::vector<std::vector<cv::Vec3f>> objPoints;
    cv::Size imgSize;

    if (standAloneInteraction == true) {
        std::cout << "# How many photos do you have? \n";
        std::cin >> numCalibPhotos;
    } else {
        numCalibPhotos = (int) fullFileNames.size();
    }

    filenames.resize(numCalibPhotos);
    imgPoints.resize(numCalibPhotos);
    objPoints.resize(numCalibPhotos);

    if (standAloneInteraction == true) {
        std::cout << "# Enter full path file names one by one: \n";
        for (int i = 0; i < numCalibPhotos; i++) {
            std::cin >> filenames[i];
        }
    } else {
        for (int i = 0; i < numCalibPhotos; i++)
            filenames[i] = fullFileNames[i];
    }

    // ask how many corners do you have on the board
    int nRowsBlocks, nColsBlocks;
    float bSizeX, bSizeY;
    if (standAloneInteraction == true) {
        std::cout << "# How many corners (x by y) are in your"
                  << " calibrate board?\n";
        std::cin >> nRowsBlocks >> nColsBlocks;
        std::cout << "# What are the size (along x and y) "
                  << "of the block in the board?\n";
        std::cin >> bSizeX >> bSizeY;
    } else {
        nRowsBlocks = patternSize.width;
        nColsBlocks = patternSize.height;
        bSizeX = calibBlockSize[0];
        bSizeY = calibBlockSize[1];
    }

    for (int i = 0; i < numCalibPhotos; i++) {
        imgPoints[i].resize(nRowsBlocks * nColsBlocks);
        objPoints[i].resize(nRowsBlocks * nColsBlocks);
    }

    // Step 2: Read the next photo
    for (int i = 0; i < numCalibPhotos; i++) {
        // read image
        cv::Mat theImg;
        bool patternFound = false;
        // a while loop that read image and find corners until corners are found.
        while (true) {
            // a while loop that waits for a valid image
            while (true) {
                theImg = cv::imread(filenames[i],
                                            cv::IMREAD_GRAYSCALE);
                if (theImg.cols <= 0 || theImg.rows <= 0) {
                    for (int k = 0; k < 999; k++) printf("\b");
                    printf("# Warning from calibrateCameraFromImageFileNames()"
                           ". Waiting for a valid image from file %s."
                           " (clock %d)",
                           filenames[i].c_str(), (int) clock());
                    // wait if the file is not an image file
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000) );
                } else {
                    break;
                }
            }

            imgSize.width = theImg.cols;
            imgSize.height = theImg.rows;
            if (printInfo == true) {
                printf("# Read image %s successfully which size is %d x %d.\n",
                       filenames[i].c_str(), theImg.cols, theImg.rows);
            }

            // Step 3: Get the image locations of the corners
            if (printInfo == true) printf("# Trying to find corners ...");
            patternFound =
            cv::findChessboardCorners(theImg,
                  cv::Size(nRowsBlocks, nColsBlocks),
                  imgPoints[i]);
            if (patternFound == true) {
                if (printInfo == true)
                    printf("# Corners found.\n");
            } else {
                printf("# Corners not found.\n");
                printf("# Warning: Cannot find corners in file %s. "
                       "Try to replace this image (or blur background by a photo editor."
                       "This program will wait.\n", filenames[i].c_str());
                continue;
            }
            cv::cornerSubPix(theImg, imgPoints[i],
                             cv::Size(3, 3),
                             cv::Size(-1, -1),
                             criteria);
            break;
        } // end of loop of reading image and finding corners until success.

        if (showCorners == true) {
            cv::Mat colorfulImg(theImg.rows, theImg.cols, CV_8UC3);
            cv::cvtColor(theImg, colorfulImg, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(colorfulImg,
                  cv::Size(nRowsBlocks, nColsBlocks),
                  imgPoints[i], patternFound);
            cv::imshow("Corners Found", colorfulImg);
            cv::waitKey(1000);
            cv::destroyWindow("Corners Found");
        }

        if (printInfo == true) {
            std::cout << "# Image points: \n";//
            for (int j = 0; j < nRowsBlocks * nColsBlocks; j++)
                printf("# %9.2f %9.2f\n",
                       imgPoints[i][j][0], imgPoints[i][j][1]);

        }

        // Step 4: Go back to Step 2 for next photo
    }

    // Step 5: Define the 3D coordinate of the corners (in chessboard coordinate)
    for (int i = 0; i < numCalibPhotos; i++) {
        for (int j = 0; j < nColsBlocks; j++) {
            for (int k = 0; k < nRowsBlocks; k++) {
                objPoints[i][k + j * nRowsBlocks] = cv::Vec3f(k * bSizeX, j * bSizeY, 0.0f);
            }
        }
    } // end of loop of each calibration photo

    // Step 6: Calibrate the camera by using OpenCV (cv::calibrateCamera())
//    cv::Mat cmat(3, 3, CV_64F), dvec(8, 1, CV_64F);
//    std::vector<cv::Mat> rvecs; //(numCalibPhotos, cv::Mat::zeros(3, 1, CV_64F));
//    std::vector<cv::Mat> tvecs; //(numCalibPhotos, cv::Mat::zeros(3, 1, CV_64F));
    cmat = cv::Mat::zeros(3, 3, CV_64F);
    dvec = cv::Mat::zeros(12, 1, CV_64F);

//    int flag = flags;
//               cv::CALIB_FIX_K2 |
//               cv::CALIB_FIX_K3 |
//               cv::CALIB_FIX_K4 |
//               cv::CALIB_FIX_K5 |
//               cv::CALIB_FIX_K6 ;

    if (printInfo == true) {
        printf("# Calibrating the camera ...");
    }
    cv::calibrateCamera(objPoints,
        imgPoints,
        imgSize,
        cmat, dvec, rvecs, tvecs,
        stdevIntrinsic,
        stdevExtrinsic,
        perViewErrors,
        flags, criteria);

    if (printInfo == true) {
        printf("# OK\n");
    }

    // Step 7: Print the parameters
    if (printInfo) {
        printIntrinsic(cmat, dvec, stdevIntrinsic, perViewErrors);
    } // end of if printInfo

    if (showUndistorted) {
        // Step 8: Undistortion each of the photos
        // Step 9: Compare the original photos and the undistorted photos
        for (int i = 0; i < numCalibPhotos; i++) {
            // Read the image again
            cv::Mat theImg = cv::imread(filenames[i]);
            // Undistort the image
            cv::Mat undImg;   //!< undistorted image
            cv::undistort(theImg, undImg, cmat, dvec);
            // Write the undistorted image to a JPG file
            cv::imwrite(filenames[i] + "_undistorted.JPG", undImg);
            // Show the images
            cv::imshow("Original image", theImg);
            cv::imshow("Undistorted image", undImg);
            cv::waitKey(0);
            cv::destroyWindow("Original image");
            cv::destroyWindow("Undistorted image");
        } // end of loop of each calibration photo
    }

    return 0;
}
