#include "mainwindow.h"
#include "lidar/wheeltec-N10/wheeltec_n10_protocol.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    struct WHEELTEC_N10_FRAME *frame = NULL;

    wheeltec_n10_frame_unpack(NULL, 0, &frame);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
