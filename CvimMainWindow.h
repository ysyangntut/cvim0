#ifndef CVIMMAINWINDOW_H
#define CVIMMAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class CvimMainWindow;
}

class CvimMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit CvimMainWindow(QWidget *parent = nullptr);
    ~CvimMainWindow();

private slots:
    void on_action_User_Point_Calibration_triggered();

private:
    Ui::CvimMainWindow *ui;
};

#endif // CVIMMAINWINDOW_H
