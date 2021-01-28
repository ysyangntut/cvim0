#include "UserPointCalibrationDialog.h"
#include "ui_UserPointCalibrationDialog.h"

#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "pickAPoint.h"
#include "impro_util.h"


UserPointCalibrationDialog::UserPointCalibrationDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::UserPointCalibrationDialog)
{
    ui->setupUi(this);
}

UserPointCalibrationDialog::~UserPointCalibrationDialog()
{
    delete ui;
}

QImage UserPointCalibrationDialog::opencvMatToQImage(const cv::Mat &img, std::string filename)
{
    QImage qimg;
    if (img.channels() == 3)
        qimg = QImage(this->img.data, this->img.cols, this->img.rows,
                      this->img.step, QImage::Format_BGR888);
    else if (img.channels() == 1)
        qimg = QImage(this->img.data, this->img.cols, this->img.rows,
                      this->img.step, QImage::Format_Grayscale8);
    else if (img.channels() == 4)  {
        QMessageBox msgBox;
        msgBox.setText("Warning: 4-channel image. Assuming ARGB32: " + QString::fromStdString(filename));
        msgBox.exec();
        qimg = QImage(this->img.data, this->img.cols, this->img.rows,
                      this->img.step, QImage::Format_ARGB32);
    }
    else {
        QMessageBox msgBox;
        msgBox.setText("Warning: Unknown format: " + QString::fromStdString(filename));
        msgBox.exec();
    }
    return qimg;
}

void UserPointCalibrationDialog::on_pbSelectImg_clicked()
{
    // Declare variables
    QString imgQFilename;
    int wLabel = this->ui->lbImg->width();
    int hLabel = this->ui->lbImg->height();


    // Get file
    imgQFilename = QFileDialog::getOpenFileName();
    if (imgQFilename.length() > 1)
        this->imgFilename = imgQFilename.toStdString();
    else
        return;

    // Read image
    this->img = cv::imread(this->imgFilename);
    if (this->img.cols <= 0 || this->img.rows <= 0)
    {
        QMessageBox msgBox;
        msgBox.setText("Error: Cannot read image from file " + imgQFilename);
        msgBox.exec();
        return;
    }

    // Show image
    QImage qimg;
    qimg = opencvMatToQImage(this->img, this->imgFilename);

    QPixmap qpixmap = QPixmap::fromImage(qimg).scaled(wLabel, hLabel,
                      Qt::KeepAspectRatio, Qt::SmoothTransformation);
    this->ui->lbImg->setPixmap(qpixmap);
}

void UserPointCalibrationDialog::on_pbImgGv_clicked()
{
    // Show image in full resolution with QGraphicsView
    if (this->img.cols > 0 && this->img.rows > 0)
    {
        QGraphicsScene* scene = new QGraphicsScene();
        QGraphicsView* view = new QGraphicsView(scene);
        QImage qimg;
        qimg = opencvMatToQImage(this->img, this->imgFilename);
        QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(qimg));
        scene->addItem(item);
        view->show();
    }
}

void UserPointCalibrationDialog::on_pbImgInteractive_clicked()
{
    cv::Point2f dummyPoint;
    pickAPoint("Interctive view", this->img, dummyPoint);
}

void UserPointCalibrationDialog::on_pbImshow1600_clicked()
{
    imshow_resize("Image Show 1600", this->img, cv::Size(1600, 900));
}

void UserPointCalibrationDialog::on_edGlobalPoints_textChanged()
{
    std::stringstream ss;
    std::vector<float> floatsInText;
    // get text (entire content to a single string)
    ss << this->ui->edGlobalPoints->toPlainText().toStdString();
    // read floats from text
    while (true) {
        float tmp;
        std::string stmp;
        // check EOF of the stream
        if (ss.eof() == true) break;
        // read a string
        ss >> stmp;
        // convert to a float
        try {
            tmp = std::stof(stmp);
        }  catch (...) {
            tmp = std::nanf("");
        }
        // append to vector
        if (std::isnan(tmp) == false)
            floatsInText.push_back(tmp);
    }
    // check if number of floats are multiple of three
    this->ui->lbNumGlobalPoints->setStyleSheet("color: black;");
    if (floatsInText.size() % 3 == 0) {
        // if multiple of three, display number of points, and copy to this->uGlobalPoints
        int npoint = (int) (floatsInText.size() / 3);
        char buf[1000];
        snprintf(buf, 1000, "%d points", npoint);
        this->ui->lbNumGlobalPoints->setText(buf);
        this->ui->lbNumGlobalPoints->setStyleSheet("color: black;");
        this->uGlobalPoints = cv::Mat(1, npoint, CV_32FC3);
        for (int i = 0; i < npoint; i++) {
            this->uGlobalPoints.at<cv::Point3f>(0, i).x = floatsInText[i * 3 + 0];
            this->uGlobalPoints.at<cv::Point3f>(0, i).y = floatsInText[i * 3 + 1];
            this->uGlobalPoints.at<cv::Point3f>(0, i).z = floatsInText[i * 3 + 2];
        }
    } else {
        // if not multiple of three, display number of floats by red fonts
        char buf[1000];
        snprintf(buf, 1000, "%d floats", (int) floatsInText.size());
        this->ui->lbNumGlobalPoints->setText(buf);
        this->ui->lbNumGlobalPoints->setStyleSheet("color: red;");
    }
}

void UserPointCalibrationDialog::on_edImgPoints_textChanged()
{
    std::stringstream ss;
    std::vector<float> floatsInText;
    // get text (entire content to a single string)
    ss << this->ui->edImgPoints->toPlainText().toStdString();
    // read floats from text
    while (true) {
        float tmp;
        std::string stmp;
        // check EOF of the stream
        if (ss.eof() == true) break;
        // read a string
        ss >> stmp;
        // convert to a float
        try {
            tmp = std::stof(stmp);
        }  catch (...) {
            tmp = std::nanf("");
        }
        // append to vector
        if (std::isnan(tmp) == false)
            floatsInText.push_back(tmp);
    }
    // check if number of floats are multiple of two
    this->ui->lbNumImgPoints->setStyleSheet("color: black;");
    if (floatsInText.size() % 2 == 0) {
        // if multiple of two, display number of points, and copy to this->uGlobalPoints
        int npoint = (int) (floatsInText.size() / 2);
        char buf[1000];
        snprintf(buf, 1000, "%d points", npoint);
        this->ui->lbNumImgPoints->setText(buf);
        this->ui->lbNumImgPoints->setStyleSheet("color: black;");
        this->uImgPoints = cv::Mat(1, npoint, CV_32FC2);
        for (int i = 0; i < npoint; i++) {
            this->uImgPoints.at<cv::Point2f>(0, i).x = floatsInText[i * 2 + 0];
            this->uImgPoints.at<cv::Point2f>(0, i).y = floatsInText[i * 2 + 1];
        }
    } else {
        // if not multiple of two, display number of floats by red fonts
        char buf[1000];
        snprintf(buf, 1000, "%d floats", (int) floatsInText.size());
        this->ui->lbNumImgPoints->setText(buf);
        this->ui->lbNumImgPoints->setStyleSheet("color: red;");
    }
}

