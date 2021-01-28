#include "CvimMainWindow.h"

#include <QApplication>


#include <iostream>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CvimMainWindow w;
    w.show();
    return a.exec();
}
