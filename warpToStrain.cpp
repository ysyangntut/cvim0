#include <warpToStrain.h>

// namespace cvim {

cv::Mat deformToDxyMat(float w, float h) {
    // constants
    const double pi = 3.14159265358979323846;
    const float pif = (float) pi;

    // variable to return
    cv::Mat def2Dxy = cv::Mat::ones(8, 8, CV_32F) * 99.9;  // initialized as a 99.9 for debugging

    // assign matrix coefficients
    // Dx == 1
    def2Dxy.at<float>(0, 0) = 1.0f;
    def2Dxy.at<float>(1, 0) = 0.0f;
    def2Dxy.at<float>(2, 0) = 1.0f;
    def2Dxy.at<float>(3, 0) = 0.0f;
    def2Dxy.at<float>(4, 0) = 1.0f;
    def2Dxy.at<float>(5, 0) = 0.0f;
    def2Dxy.at<float>(6, 0) = 1.0f;
    def2Dxy.at<float>(7, 0) = 0.0f;
    // Dy == 1
    def2Dxy.at<float>(0, 1) = 0.0f;
    def2Dxy.at<float>(1, 1) = 1.0f;
    def2Dxy.at<float>(2, 1) = 0.0f;
    def2Dxy.at<float>(3, 1) = 1.0f;
    def2Dxy.at<float>(4, 1) = 0.0f;
    def2Dxy.at<float>(5, 1) = 1.0f;
    def2Dxy.at<float>(6, 1) = 0.0f;
    def2Dxy.at<float>(7, 1) = 1.0f;
    // Rotz == 1 degree
    def2Dxy.at<float>(0, 2) = +0.5f * h / (180.f / pif);
    def2Dxy.at<float>(1, 2) = -0.5f * w / (180.f / pif);
    def2Dxy.at<float>(2, 2) = +0.5f * h / (180.f / pif);
    def2Dxy.at<float>(3, 2) = +0.5f * w / (180.f / pif);
    def2Dxy.at<float>(4, 2) = -0.5f * h / (180.f / pif);
    def2Dxy.at<float>(5, 2) = +0.5f * w / (180.f / pif);
    def2Dxy.at<float>(6, 2) = -0.5f * h / (180.f / pif);
    def2Dxy.at<float>(7, 2) = -0.5f * w / (180.f / pif);
    // exx == 1
    def2Dxy.at<float>(0, 3) = -0.5f * w;
    def2Dxy.at<float>(1, 3) = 0.0f;
    def2Dxy.at<float>(2, 3) = +0.5f * w;
    def2Dxy.at<float>(3, 3) = 0.0f;
    def2Dxy.at<float>(4, 3) = +0.5f * w;
    def2Dxy.at<float>(5, 3) = 0.0f;
    def2Dxy.at<float>(6, 3) = -0.5f * w;
    def2Dxy.at<float>(7, 3) = 0.0f;
    // eyy == 1
    def2Dxy.at<float>(0, 4) = 0.0f;
    def2Dxy.at<float>(1, 4) = -0.5f * h;
    def2Dxy.at<float>(2, 4) = 0.0f;
    def2Dxy.at<float>(3, 4) = -0.5f * h;
    def2Dxy.at<float>(4, 4) = 0.0f;
    def2Dxy.at<float>(5, 4) = +0.5f * h;
    def2Dxy.at<float>(6, 4) = 0.0f;
    def2Dxy.at<float>(7, 4) = +0.5f * h;
    // exy == 1
    def2Dxy.at<float>(0, 5) =  -0.5f * w * w * h / (w * w + h * h);
    def2Dxy.at<float>(1, 5) =  -0.5f * h * w * h / (w * w + h * h);
    def2Dxy.at<float>(2, 5) =  -0.5f * w * w * h / (w * w + h * h);
    def2Dxy.at<float>(3, 5) =  +0.5f * h * w * h / (w * w + h * h);
    def2Dxy.at<float>(4, 5) =  +0.5f * w * w * h / (w * w + h * h);
    def2Dxy.at<float>(5, 5) =  +0.5f * h * w * h / (w * w + h * h);
    def2Dxy.at<float>(6, 5) =  +0.5f * w * w * h / (w * w + h * h);
    def2Dxy.at<float>(7, 5) =  -0.5f * h * w * h / (w * w + h * h);
    // bendx == 1
    def2Dxy.at<float>(0, 6) =  -def2Dxy.at<float>(0, 3);
    def2Dxy.at<float>(1, 6) =  -def2Dxy.at<float>(1, 3);
    def2Dxy.at<float>(2, 6) =  -def2Dxy.at<float>(2, 3);
    def2Dxy.at<float>(3, 6) =  -def2Dxy.at<float>(3, 3);
    def2Dxy.at<float>(4, 6) =  +def2Dxy.at<float>(4, 3);
    def2Dxy.at<float>(5, 6) =  +def2Dxy.at<float>(5, 3);
    def2Dxy.at<float>(6, 6) =  +def2Dxy.at<float>(6, 3);
    def2Dxy.at<float>(7, 6) =  +def2Dxy.at<float>(7, 3);
    // bendy == 1
    def2Dxy.at<float>(0, 7) =  -def2Dxy.at<float>(0, 4);
    def2Dxy.at<float>(1, 7) =  -def2Dxy.at<float>(1, 4);
    def2Dxy.at<float>(2, 7) =  +def2Dxy.at<float>(2, 4);
    def2Dxy.at<float>(3, 7) =  +def2Dxy.at<float>(3, 4);
    def2Dxy.at<float>(4, 7) =  +def2Dxy.at<float>(4, 4);
    def2Dxy.at<float>(5, 7) =  +def2Dxy.at<float>(5, 4);
    def2Dxy.at<float>(6, 7) =  -def2Dxy.at<float>(6, 4);
    def2Dxy.at<float>(7, 7) =  -def2Dxy.at<float>(7, 4);

    return def2Dxy;
}

// warp: a 2x3 or 3x3 CV_32F matrix.
// deformation: a 8-element std::vector<float> that contains
// Dx, Dy, Rotz (in degree), exx, eyy, exy, bendx, bendx.
// Bendx == 1.0 means lower/upper boundary exx == -0.5/+0.5.
// Bendy == 1.0 means left/right boundary exx == -0.5/+0.5.

const bool debugDeformFromWap = true;
std::vector<float> deformFromWarp(const cv::Mat & _W,
                                  cv::Point2f ref)
{
    // constants
    const double pi = 3.14159265358979323846;
    const float pif = (float) pi;

    // veriable to return;
    std::vector<float> deformVec(8, 0.0f); // Dx, Dy, Rotz (deg), exx, eyy, exy, hourglassing x, and hourglassing y

    const float w = 1.0f, h = 1.0f;
    if (debugDeformFromWap == true) {
        std::cout << "A rectangular patch width and height are \n" << w << " and " << h << std::endl;
    }

    // Make sure W is 3x3
    cv::Mat W = cv::Mat::eye(3, 3, CV_32F);
    if (_W.rows == 3 && _W.cols == 3 && _W.type() == CV_32F) {
        W = _W.clone();
    } else if (_W.rows == 2 && _W.cols == 3 && _W.type() == CV_32F) {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 3; j++)
                W.at<float>(i, j) = _W.at<float>(i, j);
    } else if (_W.rows == 3 && _W.cols == 3 && _W.type() == CV_64F) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                W.at<float>(i, j) = (float) _W.at<double>(i, j);
    } else if (_W.rows == 2 && _W.cols == 3 && _W.type() == CV_64F) {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 3; j++)
                W.at<float>(i, j) = (float) _W.at<double>(i, j);
    } else {
        // Give a warning message
        std::cout << "Warning: cvim::deformFromWarp() requires a 2x3 or"
             << " 3x3 CV_32F or CV_64F matrix but it gets a " << _W.rows
             << " by " << _W.rows << " matrix with type "
             << _W.type() << ". So it returns zero deformation.\n";
        return deformVec;
    }

    // if reference point is not at (0.f, 0.f)
    if (ref.x != 0.f || ref.y != 0.f) {
        cv::Mat trans = cv::Mat::eye(3, 3, CV_32F);
        trans.at<float>(0, 2) = -ref.x;
        trans.at<float>(1, 2) = -ref.y;
        cv::Mat __W = trans * W * trans.inv();
        __W.copyTo(W);
    }

    // decompose warp matrix to rotMat and defMat that W = defMat * rotMat
    cv::Mat rotMat(3, 3, CV_32F), deformMat33(3, 3, CV_32F);
    float rotzInDeg;
    cv::Mat projMat = cv::Mat::zeros(3, 4, CV_32F);
    projMat.at<float>(0, 0) = W.at<float>(0, 0);
    projMat.at<float>(0, 1) = W.at<float>(0, 1);
    projMat.at<float>(1, 0) = W.at<float>(1, 0);
    projMat.at<float>(1, 1) = W.at<float>(1, 1);
    projMat.at<float>(2, 2) = W.at<float>(2, 2);

    const int decomposeMethod = 2;
    if (decomposeMethod == 1) {
        // decompose using cv::decomposeProjectionMatrix()
        cv::Mat camMat, transVec, rMatX, rMatY, rMatZ, eulerAngles;
        cv::decomposeProjectionMatrix(projMat, camMat, rotMat, transVec, rMatX, rMatY, rMatZ, eulerAngles);
        if (debugDeformFromWap == true) {
            std::cout << "Warp:\n" << W << std::endl;
            std::cout << "ProjMat:\n" << projMat << std::endl;
            std::cout << "CamMat:\n" << camMat << std::endl;
            std::cout << "TransVec:\n" << transVec << std::endl;
            std::cout << "RotMat:\n" << rotMat << std::endl;
            std::cout << "RotMatX:\n" << rMatX << std::endl;
            std::cout << "RotMatY:\n" << rMatY << std::endl;
            std::cout << "RotMatZ:\n" << rMatZ << std::endl;
            std::cout << "eulerAngles:\n" << eulerAngles << std::endl;
            std::cout.flush();
            if (eulerAngles.type() == CV_32F)
                rotzInDeg = eulerAngles.at<float>(2);
            else if (eulerAngles.type() == CV_64F)
                rotzInDeg = (float) eulerAngles.at<double>(2);
        }
    } // end of if using decomposeProjectionMatrix
    else if (decomposeMethod == 2)
    {
        cv::Mat rotVec(1, 3, CV_32F);
        // decompose using Rodrigues
        rotMat = cv::Mat::eye(3, 3, CV_32F);
        rotMat.at<float>(0, 0) = W.at<float>(0, 0);
        rotMat.at<float>(0, 1) = W.at<float>(0, 1);
        rotMat.at<float>(1, 0) = W.at<float>(1, 0);
        rotMat.at<float>(1, 1) = W.at<float>(1, 1);
        cv::Rodrigues(rotMat, rotVec);
        if (debugDeformFromWap == true) {
            std::cout << "Rodrigues converts from Mat 3x3:\n"
                 << rotMat << std::endl;
            std::cout << " to a vector:\n" << rotVec << std::endl;
        }
        cv::Rodrigues(rotVec, rotMat);
        if (debugDeformFromWap == true) {
            std::cout << "And converts back to matrix:\n"
                 << rotMat << std::endl;
        }
        rotzInDeg = (float) rotVec.at<float>(2) * 180.f / pif;
    }
    else if (decomposeMethod == 3)
    {
        // decompose using cv::decomposeHomographyMat()
        cv::Mat camMat = cv::Mat::eye(3, 3, CV_64F);
        std::vector<cv::Mat> rots, trans, normals;
        cv::decomposeHomographyMat(W, camMat, rots, trans, normals);
        if (debugDeformFromWap == true) {
            std::cout << "Array of rot mats: (" << rots.size() << ")\n";
            for (int i = 0; i < rots.size(); i++) std::cout << rots[i] << std::endl;
            std::cout << "Array of trans: (" << trans.size() << ")\n";
            for (int i = 0; i < trans.size(); i++) std::cout << trans[i] << std::endl;
            std::cout << "Array of normals: (" << normals.size() << ")\n";
            for (int i = 0; i < normals.size(); i++) std::cout << normals[i] << std::endl;
        }
        rotMat = rots[rots.size() - 1].clone();
        rotzInDeg = std::acos(rotMat.at<float>(0)) * (float)(180.0 / pi);
    } // end of if using decomposeHomography

    // Calculate deformation matrix by W and rotMat (rigid rot.)
    deformMat33 = W * rotMat.inv();

    if (debugDeformFromWap == true)
    {
        std::cout << "---------------\n";
        std::cout << "Warp:\n" << W << std::endl;
        std::cout << "Rot:\n" << rotMat << std::endl;
        std::cout << "DeformMat:\n" << deformMat33 << std::endl;
        std::cout << "Rot in deg: " << rotzInDeg << std::endl;
        std::cout << "---------------\n";
        std::cout.flush();
    }

    // rectangle coordinates before and after deformation (deformMat33)
    // (deformMat33, not W, as W contains rigid body motion rotMat and deformation)
    cv::Mat X0 = cv::Mat::ones(3, 4, CV_32F);  // rectangle coordinates before deformation
    cv::Mat X1 = cv::Mat::ones(3, 4, CV_32F);  // rectangle coordinates after deformation
    cv::Mat Dx;
    X0.at<float>(0, 0) = -.5f * w; X0.at<float>(1, 0) = -.5f * h;
    X0.at<float>(0, 1) = +.5f * w; X0.at<float>(1, 1) = -.5f * h;
    X0.at<float>(0, 2) = +.5f * w; X0.at<float>(1, 2) = +.5f * h;
    X0.at<float>(0, 3) = -.5f * w; X0.at<float>(1, 3) = +.5f * h;
    X1 = deformMat33 * X0;
    Dx = X1 - X0;
    if (debugDeformFromWap == true){
        std::cout << "Assuming 4 corners of the patch:\n" << X0 << std::endl;
        std::cout << "Ignoring rotation, after translation and deformation of warping:\n" << X1 << std::endl;
        std::cout << "Displacement:\n" << Dx << std::endl;
    }

    // deformToDxy (8x8 matrix)
    cv::Mat def2Dxy = deformToDxyMat(w, h);
    cv::Mat DxVec(8, 1, CV_32F);
    DxVec.at<float>(0) = Dx.at<float>(0, 0);
    DxVec.at<float>(1) = Dx.at<float>(1, 0);
    DxVec.at<float>(2) = Dx.at<float>(0, 1);
    DxVec.at<float>(3) = Dx.at<float>(1, 1);
    DxVec.at<float>(4) = Dx.at<float>(0, 2);
    DxVec.at<float>(5) = Dx.at<float>(1, 2);
    DxVec.at<float>(6) = Dx.at<float>(0, 3);
    DxVec.at<float>(7) = Dx.at<float>(1, 3);
    cv::Mat deformVecMat = def2Dxy.inv() * DxVec;
    for (int i = 0; i < 8; i++)
        deformVec[i] = deformVecMat.at<float>(i);
    deformVec[2] += rotzInDeg;
    if (debugDeformFromWap)
    {
        std::cout << "The deformation vector is: \n";
        std::cout << "Translation dx: " << deformVec[0] << std::endl;
        std::cout << "Translation dy: " << deformVec[1] << std::endl;
        std::cout << "Rotation (deg): " << deformVec[2] << std::endl;
        std::cout << "Strain xx:      " << deformVec[3] << std::endl;
        std::cout << "Strain yy:      " << deformVec[4] << std::endl;
        std::cout << "Shear strain:   " << deformVec[5] << std::endl;
        std::cout << "Hourglassing X: " << deformVec[6] << std::endl;
        std::cout << "Hourglassing Y: " << deformVec[7] << std::endl;
    }
    return deformVec;
}



// } // end of namespace cvim
