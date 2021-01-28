#include <iostream>
using namespace std;

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"


class PlaneConstrainProjectionCostFunction :public cv::MinProblemSolver::Function {
public:
	// called by optimization
	double calc(const double* x) const {
		double dx, dy, err; 
		cv::Mat trial3d(1, 1, CV_64FC3), trial2d(1, 1, CV_64FC2); 
		trial3d.at<cv::Point3d>(0, 0).x = xc[0] + x[0] * xcVec1[0] + x[1] * xcVec2[0];
		trial3d.at<cv::Point3d>(0, 0).y = xc[1] + x[0] * xcVec1[1] + x[1] * xcVec2[1];
		trial3d.at<cv::Point3d>(0, 0).z = xc[2] + x[0] * xcVec1[2] + x[1] * xcVec2[2];
		cv::projectPoints(trial3d, rvec, tvec, camMat, disVec, trial2d);
		dx = trial2d.at<cv::Point2d>(0, 0).x - imgPoint[0];
		dy = trial2d.at<cv::Point2d>(0, 0).y - imgPoint[1];
//		cout << "Triai point: " << trial3d << endl;
//		cout << "Proj 2d: " << trial2d << endl;
		err = dx * dx + dy * dy;
		return err; 
	}
	virtual int getDims() const { return 2; }
	// setting
	cv::Mat camMat;
	cv::Mat disVec;
	cv::Mat rvec; 
	cv::Mat tvec; 
	double imgPoint[2];
	double xc[3]; 
	double xcVec1[3];
	double xcVec2[3];
};


int plane_constrained3dPositioning(
	const cv::Mat & camMat,      // 3x3 camera matrix
	const cv::Mat & disVec,      // 4x1 or 5x1 (or Nx1) distortion coefficients
	const cv::Mat & rmat,        // 3x3 Rotation matrix or 3x1 or 1x3 vector of this camera (wrt global coord.).
	const cv::Mat & tvec,        // 3x1 or 1x3 Translation vector of the camera.
	const cv::Mat & imgPoints,   // Nx1 2-channel image points on photo. N is number of points to track.
	const cv::Mat & xc,          // Nx1 3-channel constraining points, which define surfaces
	const cv::Mat & xcVec1,      // Nx1 3-channel vector 1 of each tracking point, which define surfaces
	const cv::Mat & xcVec2,      // Nx1 3-channel vector 2 of each tracking point, which define surfaces
	      cv::Mat & xp           // Nx1 3-channel positioned points, will be on the surface
)
{
	int ret = 0;
	// check
	if (camMat.cols != 3 || camMat.rows != 3) {
		cerr << "plane_constrained3dPositioning: camMat has wrong size " << camMat.rows << "x" << camMat.cols << endl;
		return -1;
	}
	if (disVec.cols != 1 && disVec.rows != 1) {
		cerr << "plane_constrained3dPositioning: disVec has wrong size " << disVec.rows << "x" << disVec.cols << endl;
		return -1;
	}
	if (rmat.cols * rmat.rows != 3 && !(rmat.cols == 3 && rmat.rows == 3)) {
		cerr << "plane_constrained3dPositioning: rmat has wrong size " << rmat.rows << "x" << rmat.cols << endl;
		return -1;
	}
	if (tvec.cols * tvec.rows != 3) {
		cerr << "plane_constrained3dPositioning: tvec has wrong size " << tvec.rows << "x" << tvec.cols << endl;
		return -1;
	}
	if (imgPoints.cols != 1 || imgPoints.rows < 1) {
		cerr << "plane_constrained3dPositioning: imgPoints has wrong size " << imgPoints.rows << "x" << imgPoints.cols << endl;
		return -1;
	}
	if (imgPoints.type() != CV_32FC2) {
		cerr << "plane_constrained3dPositioning: imgPoints has wrong type (should be CV_32FC2 but) " << imgPoints.type() << endl;
	}
	if (xc.cols != 1 || xc.rows < 1) {
		cerr << "plane_constrained3dPositioning: xc has wrong size " << xc.rows << "x" << xc.cols << endl;
		return -1;
	}
	if (xc.type() != CV_64FC3) {
		cerr << "plane_constrained3dPositioning: xc has wrong type (should be CV_64FC3 but) " << xc.type() << endl;
	}
	if (xcVec1.cols != 1 || xcVec1.rows < 1) {
		cerr << "plane_constrained3dPositioning: xcVec1 has wrong size " << xcVec1.rows << "x" << xcVec1.cols << endl;
		return -1;
	}
	if (xcVec1.type() != CV_64FC3) {
		cerr << "plane_constrained3dPositioning: xcVec1 has wrong type (should be CV_64FC3 but) " << xcVec1.type() << endl;
	}
	if (xcVec2.cols != 1 || xcVec2.rows < 1) {
		cerr << "plane_constrained3dPositioning: xcVec2 has wrong size " << xcVec2.rows << "x" << xcVec2.cols << endl;
		return -1;
	}
	if (xcVec2.type() != CV_64FC3) {
		cerr << "plane_constrained3dPositioning: xcVec2 has wrong type (should be CV_64FC3 but) " << xcVec2.type() << endl;
	}
	if (xp.cols != 1 || xp.rows < 1) {
		xp = cv::Mat::zeros(xcVec2.rows, 1, CV_64FC3); 
	}
	if (xp.type() != CV_64FC3) {
		xp = cv::Mat::zeros(xcVec2.rows, 1, CV_64FC3);
	}
	if (imgPoints.rows != xc.rows || xc.rows != xcVec1.rows || xcVec1.rows != xcVec2.rows && xcVec2.rows != xp.rows) {
		cerr << "plane_constrained3dPositioning: imgPoints/xc/xcVec1/xcVec2/xp do not have consistent # of rows: "
			<< imgPoints.rows << " " << xc.rows << " " << xcVec1.rows << " " << xcVec2.rows << " " << xp.rows << endl;
		return -1;
	}

	int nPoint = imgPoints.rows;
	// cost function
	PlaneConstrainProjectionCostFunction * costFunc = new PlaneConstrainProjectionCostFunction;
	camMat.copyTo(costFunc->camMat);
	disVec.copyTo(costFunc->disVec);
	tvec.copyTo(costFunc->tvec);
	cv::Mat rvec(3, 1, CV_64F);
	if (rmat.cols == 3 && rmat.rows == 3) {
		cv::Rodrigues(rmat, rvec);
		rvec.copyTo(costFunc->rvec);
	}
	else {
		rmat.copyTo(costFunc->rvec);
	}
//	cout << "camMat: \n" << camMat << endl;
//	cout << "disVec: \n" << disVec << endl;
//	cout << "rvec: \n" << rmat << endl;
//	cout << "tvec: \n" << tvec << endl;
//	cout << "imgPoints: \n" << imgPoints << endl;
//	cout << "xc: \n" << xc << endl;
//	cout << "xcVec1: \n" << xcVec1 << endl;
//	cout << "xcVec2: \n" << xcVec2 << endl;
//	cout << "xp: \n" << xp << endl;

	// Solver 
	cv::Ptr<cv::MinProblemSolver::Function> costFuncPtr(costFunc);
	//	cv::Ptr<cv::ConjGradSolver> mySolver = cv::ConjGradSolver::create();
	cv::Ptr<cv::DownhillSolver> mySolver = cv::DownhillSolver::create();
	mySolver->setInitStep(10 * cv::Mat::ones(1, 2, CV_64F));
	mySolver->setFunction(costFuncPtr);
	// Positioning
	for (int i = 0; i < nPoint; i++)
	{
		// set cost function object 
		costFunc->imgPoint[0] = imgPoints.at<cv::Point2f>(i, 0).x; 
		costFunc->imgPoint[1] = imgPoints.at<cv::Point2f>(i, 0).y;
		costFunc->xc[0] = xc.at<cv::Point3d>(i, 0).x;
		costFunc->xc[1] = xc.at<cv::Point3d>(i, 0).y;
		costFunc->xc[2] = xc.at<cv::Point3d>(i, 0).z;
		costFunc->xcVec1[0] = xcVec1.at<cv::Point3d>(i, 0).x;
		costFunc->xcVec1[1] = xcVec1.at<cv::Point3d>(i, 0).y;
		costFunc->xcVec1[2] = xcVec1.at<cv::Point3d>(i, 0).z;
		costFunc->xcVec2[0] = xcVec2.at<cv::Point3d>(i, 0).x;
		costFunc->xcVec2[1] = xcVec2.at<cv::Point3d>(i, 0).y;
		costFunc->xcVec2[2] = xcVec2.at<cv::Point3d>(i, 0).z;

		// solve
		cv::Mat dx = cv::Mat::zeros(2, 1, CV_64F);
//		cout << "Before minimization\n";
		cv::TermCriteria criteria = cv::TermCriteria(
			cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-8);
		mySolver->setTermCriteria(criteria); 
		double res = mySolver->minimize(dx);
//		cout << "After minimization\n";

		// xp
		xp.at<cv::Point3d>(i, 0).x = xc.at<cv::Point3d>(i, 0).x
			+ dx.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).x
			+ dx.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).x; 
		xp.at<cv::Point3d>(i, 0).y = xc.at<cv::Point3d>(i, 0).y
			+ dx.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).y
			+ dx.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).y;
		xp.at<cv::Point3d>(i, 0).z = xc.at<cv::Point3d>(i, 0).z
			+ dx.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).z
			+ dx.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).z;
	}

//	cout << "xp: \n" << xp << endl;
	return ret;
}

int plane_constrained3dPositioning_newton(
	const cv::Mat & camMat,      // 3x3 camera matrix
	const cv::Mat & disVec,      // 4x1 or 5x1 (or Nx1) distortion coefficients
	const cv::Mat & rmat,        // 3x3 Rotation matrix or 3x1 or 1x3 vector of this camera (wrt global coord.).
	const cv::Mat & tvec,        // 3x1 or 1x3 Translation vector of the camera.
	const cv::Mat & imgPoints,   // Nx1 2-channel image points on photo. N is number of points to track.
	const cv::Mat & xc,          // Nx1 3-channel constraining points, which define surfaces
	const cv::Mat & xcVec1,      // Nx1 3-channel vector 1 of each tracking point, which define surfaces
	const cv::Mat & xcVec2,      // Nx1 3-channel vector 2 of each tracking point, which define surfaces
	cv::Mat & xp           // Nx1 3-channel positioned points, will be on the surface
)
{
	int ret = 0;
	// check
	if (camMat.cols != 3 || camMat.rows != 3) {
		cerr << "plane_constrained3dPositioning: camMat has wrong size " << camMat.rows << "x" << camMat.cols << endl;
		return -1;
	}
	if (disVec.cols != 1 && disVec.rows != 1) {
		cerr << "plane_constrained3dPositioning: disVec has wrong size " << disVec.rows << "x" << disVec.cols << endl;
		return -1;
	}
	if (rmat.cols * rmat.rows != 3 && !(rmat.cols == 3 && rmat.rows == 3)) {
		cerr << "plane_constrained3dPositioning: rmat has wrong size " << rmat.rows << "x" << rmat.cols << endl;
		return -1;
	}
	if (tvec.cols * tvec.rows != 3) {
		cerr << "plane_constrained3dPositioning: tvec has wrong size " << tvec.rows << "x" << tvec.cols << endl;
		return -1;
	}
	if (imgPoints.cols != 1 || imgPoints.rows < 1) {
		cerr << "plane_constrained3dPositioning: imgPoints has wrong size " << imgPoints.rows << "x" << imgPoints.cols << endl;
		return -1;
	}
	if (imgPoints.type() != CV_32FC2) {
		cerr << "plane_constrained3dPositioning: imgPoints has wrong type (should be CV_32FC2 but) " << imgPoints.type() << endl;
	}
	if (xc.cols != 1 || xc.rows < 1) {
		cerr << "plane_constrained3dPositioning: xc has wrong size " << xc.rows << "x" << xc.cols << endl;
		return -1;
	}
	if (xc.type() != CV_64FC3) {
		cerr << "plane_constrained3dPositioning: xc has wrong type (should be CV_64FC3 but) " << xc.type() << endl;
	}
	if (xcVec1.cols != 1 || xcVec1.rows < 1) {
		cerr << "plane_constrained3dPositioning: xcVec1 has wrong size " << xcVec1.rows << "x" << xcVec1.cols << endl;
		return -1;
	}
	if (xcVec1.type() != CV_64FC3) {
		cerr << "plane_constrained3dPositioning: xcVec1 has wrong type (should be CV_64FC3 but) " << xcVec1.type() << endl;
	}
	if (xcVec2.cols != 1 || xcVec2.rows < 1) {
		cerr << "plane_constrained3dPositioning: xcVec2 has wrong size " << xcVec2.rows << "x" << xcVec2.cols << endl;
		return -1;
	}
	if (xcVec2.type() != CV_64FC3) {
		cerr << "plane_constrained3dPositioning: xcVec2 has wrong type (should be CV_64FC3 but) " << xcVec2.type() << endl;
	}
	if (xp.cols != 1 || xp.rows < 1) {
		xp = cv::Mat::zeros(xcVec2.rows, 1, CV_64FC3);
	}
	if (xp.type() != CV_64FC3) {
		xp = cv::Mat::zeros(xcVec2.rows, 1, CV_64FC3);
	}
	if (imgPoints.rows != xc.rows || xc.rows != xcVec1.rows || xcVec1.rows != xcVec2.rows && xcVec2.rows != xp.rows) {
		cerr << "plane_constrained3dPositioning: imgPoints/xc/xcVec1/xcVec2/xp do not have consistent # of rows: "
			<< imgPoints.rows << " " << xc.rows << " " << xcVec1.rows << " " << xcVec2.rows << " " << xp.rows << endl;
		return -1;
	}

	int nPoint = imgPoints.rows;
	// cost function
	cv::Mat rvec(3, 1, CV_64F);
	if (rmat.cols == 3 && rmat.rows == 3) {
		cv::Rodrigues(rmat, rvec);
		rvec.copyTo(rvec);
	}
	else {
		rmat.copyTo(rvec);
	}
	//	cout << "camMat: \n" << camMat << endl;
	//	cout << "disVec: \n" << disVec << endl;
	//	cout << "rvec: \n" << rmat << endl;
	//	cout << "tvec: \n" << tvec << endl;
	//	cout << "imgPoints: \n" << imgPoints << endl;
	//	cout << "xc: \n" << xc << endl;
	//	cout << "xcVec1: \n" << xcVec1 << endl;
	//	cout << "xcVec2: \n" << xcVec2 << endl;
	//	cout << "xp: \n" << xp << endl;

	// Solver 

	// Positioning
	//   Positioning: estimating the jacobian finite step size
	//             
	double tol = 1e-4; 
	double dx[2] = {};
	for (int i = 0; i < nPoint; i++) {
		double xi = xc.at<cv::Point3d>(i, 0).x; 
		double yi = xc.at<cv::Point3d>(i, 0).y;
		double zi = xc.at<cv::Point3d>(i, 0).z;
		dx[0] += sqrt(xi * xi + yi * yi + zi * zi) / nPoint; 
	}
	dx[0] *= 1e-9; 
	dx[1] = dx[0]; 
//	cout << "    Finite step: " << dx[0] << endl;
		//   Positioning: running iterations
	for (int i = 0; i < nPoint; i++)
	{
		// from imgPoint, xc, xcVec1, xcVec2, to xp
//		double target[2] = {}, trial[2] = {}, x3trial[3] = {}, xitrial[2] = {}, err[2];
		double target[2]; 
		int nIterMax = 20;
		// init
		cv::Mat X = cv::Mat::zeros(2, 1, CV_64F);
		cv::Mat Y = cv::Mat::zeros(2, 1, CV_64F);
		cv::Mat J = cv::Mat::zeros(2, 2, CV_64F);
		cv::Mat XJ = cv::Mat::zeros(2, 1, CV_64F);
		cv::Mat YJ = cv::Mat::zeros(2, 1, CV_64F);
		target[0] = imgPoints.at<cv::Point2f>(i, 0).x;
		target[1] = imgPoints.at<cv::Point2f>(i, 0).y;
		// iteration
		double projErr;
		int itr; 
		for (itr = 0; itr < nIterMax; itr++) {
			cv::Mat trial3d(1, 1, CV_64FC3), trial2d(1, 1, CV_64FC2);
			// Newton's method
//			double jtrial[2];
			// Newton's method: Test current trial point (trial (0, 0))
			XJ = X;
			trial3d.at<cv::Point3d>(0, 0).x = xc.at<cv::Point3d>(i, 0).x + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).x + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).x;
			trial3d.at<cv::Point3d>(0, 0).y = xc.at<cv::Point3d>(i, 0).y + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).y + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).y;
			trial3d.at<cv::Point3d>(0, 0).z = xc.at<cv::Point3d>(i, 0).z + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).z + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).z;
			cv::projectPoints(trial3d, rvec, tvec, camMat, disVec, trial2d);
			Y.at<double>(0, 0) = trial2d.at<cv::Point2d>(0, 0).x - imgPoints.at<cv::Point2f>(i, 0).x;
			Y.at<double>(1, 0) = trial2d.at<cv::Point2d>(0, 0).y - imgPoints.at<cv::Point2f>(i, 0).y;
			projErr = Y.at<double>(0, 0) * Y.at<double>(0, 0) + Y.at<double>(1, 0) * Y.at<double>(1, 0); 
//			cout << "camMat: \n" << camMat << endl;
//			cout << "disVec: \n" << disVec << endl;
//			cout << "rvec: \n" << rvec << endl;
//			cout << "tvec: \n" << tvec << endl;
//			cout << "Newton X: " << X << endl;
//			cout << "Triai point: " << trial3d << endl;
//			cout << "Proj 2d: " << trial2d << endl;
//			cout << "Img 2d: " << imgPoints.at<cv::Point2f>(i, 0) << endl;
//			cout << "Projected Y: " << Y << endl;
//			cout << "Proj Err: " << projErr << endl;
			if (projErr < tol) break;
			// Newton's method: Jacobean Vector 1 (trial (1, 0))
			XJ.at<double>(0, 0) = X.at<double>(0, 0) + dx[0];
			XJ.at<double>(1, 0) = X.at<double>(1, 0) ;
			trial3d.at<cv::Point3d>(0, 0).x = xc.at<cv::Point3d>(i, 0).x + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).x + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).x;
			trial3d.at<cv::Point3d>(0, 0).y = xc.at<cv::Point3d>(i, 0).y + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).y + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).y;
			trial3d.at<cv::Point3d>(0, 0).z = xc.at<cv::Point3d>(i, 0).z + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).z + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).z;
			cv::projectPoints(trial3d, rvec, tvec, camMat, disVec, trial2d);
//			cout << "Triai point: " << trial3d << endl;
//			cout << "Proj 2d: " << trial2d << endl;
			YJ.at<double>(0, 0) = trial2d.at<cv::Point2d>(0, 0).x - imgPoints.at<cv::Point2f>(i, 0).x;
			YJ.at<double>(1, 0) = trial2d.at<cv::Point2d>(0, 0).y - imgPoints.at<cv::Point2f>(i, 0).y;
			J.at<double>(0, 0) = (YJ.at<double>(0, 0) - Y.at<double>(0, 0)) / dx[0];
			J.at<double>(1, 0) = (YJ.at<double>(1, 0) - Y.at<double>(1, 0)) / dx[0];
			// Newton's method: Jacobean Vector 2 (trial (0, 1))
			XJ.at<double>(0, 0) = X.at<double>(0, 0);
			XJ.at<double>(1, 0) = X.at<double>(1, 0) + dx[1];
			trial3d.at<cv::Point3d>(0, 0).x = xc.at<cv::Point3d>(i, 0).x + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).x + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).x;
			trial3d.at<cv::Point3d>(0, 0).y = xc.at<cv::Point3d>(i, 0).y + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).y + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).y;
			trial3d.at<cv::Point3d>(0, 0).z = xc.at<cv::Point3d>(i, 0).z + XJ.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).z + XJ.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).z;
			cv::projectPoints(trial3d, rvec, tvec, camMat, disVec, trial2d);
//			cout << "Triai point: " << trial3d << endl;
//			cout << "Proj 2d: " << trial2d << endl;
			YJ.at<double>(0, 0) = trial2d.at<cv::Point2d>(0, 0).x - imgPoints.at<cv::Point2f>(i, 0).x;
			YJ.at<double>(1, 0) = trial2d.at<cv::Point2d>(0, 0).y - imgPoints.at<cv::Point2f>(i, 0).y;
			J.at<double>(0, 1) = (YJ.at<double>(0, 0) - Y.at<double>(0, 0)) / dx[1];
			J.at<double>(1, 1) = (YJ.at<double>(1, 0) - Y.at<double>(1, 0)) / dx[1];
			// Next X
			X = X - 0.5 * J.inv() * Y; 
//			cout << "J : " << J << endl;
//			cout << "Jinv : " << J.inv() << endl;
//			cout << endl << endl;
		}
//		cout << " Newton solver: " << itr << " iterations. Proj err: " << projErr << endl;
//		exit(1); 
//		cv::projectPoints(trial3d, rvec, tvec, camMat, disVec, trial2d);

		// xp
		xp.at<cv::Point3d>(i, 0).x = xc.at<cv::Point3d>(i, 0).x
			+ X.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).x
			+ X.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).x;
		xp.at<cv::Point3d>(i, 0).y = xc.at<cv::Point3d>(i, 0).y
			+ X.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).y
			+ X.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).y;
		xp.at<cv::Point3d>(i, 0).z = xc.at<cv::Point3d>(i, 0).z
			+ X.at<double>(0, 0) * xcVec1.at<cv::Point3d>(i, 0).z
			+ X.at<double>(1, 0) * xcVec2.at<cv::Point3d>(i, 0).z;
	}

	//	cout << "xp: \n" << xp << endl;
	return ret;
}

