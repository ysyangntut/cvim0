#include "CvimMainWindow.h"
#include "ui_CvimMainWindow.h"

#include "UserPointCalibrationDialog.h"

#include <iostream>

CvimMainWindow::CvimMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CvimMainWindow)
{
    ui->setupUi(this);
}

CvimMainWindow::~CvimMainWindow()
{
    delete ui;
}


void CvimMainWindow::on_action_User_Point_Calibration_triggered()
{
    UserPointCalibrationDialog calibDialog;
    calibDialog.setWindowTitle("Calibration by Customized Points");
    calibDialog.exec();
    return;
}
