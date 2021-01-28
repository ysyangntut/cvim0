#ifndef USERPOINTCALIBRATIONDIALOG_H
#define USERPOINTCALIBRATIONDIALOG_H

#include <QDialog>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Ui {
class UserPointCalibrationDialog;
}

class UserPointCalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit UserPointCalibrationDialog(QWidget *parent = nullptr);
    ~UserPointCalibrationDialog();

    const int nPointsPerColinear = 4;
    cv::Mat img;
    std::string imgFilename;
    cv::Mat cmat, dvec, rvec, tvec;
    cv::Mat uImgPoints;          //!< image coord. of user defined points. (1, nPoints, CV_32FC2)
    cv::Mat uGlobalPoints;       //!< 3D global coord. of user defined points (1, nPoints, CV_32FC3)
    cv::Mat uColinearImgPoints;  //!< image coord. of user defined lines. (nLines, nPointsPerColinear, CV_32FC2)
    cv::Size imgSize;            //!< image size of this camera

    QImage opencvMatToQImage(const cv::Mat & img, std::string filename = "");

private slots:
    void on_pbSelectImg_clicked();

    void on_pbImgGv_clicked();

    void on_pbImgInteractive_clicked();

    void on_pbImshow1600_clicked();

    void on_edGlobalPoints_textChanged();

    void on_edImgPoints_textChanged();

private:
    Ui::UserPointCalibrationDialog *ui;
};

#endif // USERPOINTCALIBRATIONDIALOG_H
