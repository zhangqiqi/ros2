#include "mainwindow.h"

#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), sp(new QSerialPort)
{
    sp->setPortName("COM8");
    sp->setBaudRate(230400);

    connect(sp, SIGNAL(readyRead()), this, SLOT(slot_read_serialport_data()));

    sp->open(QSerialPort::ReadOnly);
}

MainWindow::~MainWindow()
{
}

void MainWindow::slot_read_serialport_data()
{
    QByteArray data;
    while (this->sp->bytesAvailable()) {
        data += this->sp->readAll();
    }

    qDebug() << "read bytes num: " << data.length();
}
