#include "n10lidarbridgewidget.h"
#include "wheeltec_n10_protocol.h"

#include <QSerialPortInfo>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>

N10LidarBridgeWidget::N10LidarBridgeWidget(QWidget *parent)
    : QWidget{parent}, sp{nullptr}, socket{nullptr}
{

    QGridLayout *widget_layout = new QGridLayout(this);
    this->setLayout(widget_layout);

    QGridLayout *sp_layout = new QGridLayout();

    sp_layout->addWidget(new QLabel("端口号：", this), 0, 0);
    sp_list = new QComboBox(this);
    sp_layout->addWidget(sp_list, 0, 1);

    sp_layout->addWidget(new QLabel("波特率：", this), 1, 0);
    sp_baudrate = new QLineEdit("230400", this);
    sp_layout->addWidget(sp_baudrate, 1, 1);

    sp_connect = new QPushButton("建立连接", this);
    connect(sp_connect, SIGNAL(clicked(bool)), this, SLOT(slot_sp_connect(bool)));
    sp_layout->addWidget(sp_connect, 2, 0, 1, 2);

    widget_layout->addLayout(sp_layout, 0, 0);


    QGridLayout *socket_layout = new QGridLayout();

    socket_layout->addWidget(new QLabel("服务器地址：", this), 0, 0);
    socket_addr = new QLineEdit("127.0.0.1", this);
    socket_layout->addWidget(socket_addr, 0, 1);

    socket_layout->addWidget(new QLabel("服务器端口：", this), 1, 0);
    socket_port = new QLineEdit("10001", this);
    socket_layout->addWidget(socket_port, 1, 1);

    socket_connect = new QPushButton("建立连接", this);
    connect(socket_connect, SIGNAL(clicked(bool)), this, SLOT(slot_socket_connect(bool)));
    socket_layout->addWidget(socket_connect, 2, 0, 1, 2);

    widget_layout->addLayout(socket_layout, 0, 1);

    bridge_connect = new QPushButton("建立数据桥接关系", this);
    connect(bridge_connect, SIGNAL(clicked(bool)), this, SLOT(slot_bridge_connect(bool)));
    widget_layout->addWidget(bridge_connect, 1, 0, 1, 2);

    serialportWidgetInit();
    tcpSocketWidgetInit();
}

void N10LidarBridgeWidget::serialportWidgetInit()
{
    this->sp_list->clear();

    foreach (auto sp_info, QSerialPortInfo::availablePorts()) {
        this->sp_list->addItem(sp_info.portName());
    }
}

void N10LidarBridgeWidget::tcpSocketWidgetInit()
{

}

void N10LidarBridgeWidget::slot_sp_connect(bool checked)
{
    if (nullptr == this->sp)
    {
        this->sp = new QSerialPort(this);
        connect(sp, SIGNAL(readyRead()), this, SLOT(slot_sp_data_recv()));
    }

    QString port_name = this->sp_list->currentText();
    QString baudrate = this->sp_baudrate->text();

    sp->setPortName(port_name);
    sp->setBaudRate(baudrate.toInt());

    if (false == sp->open(QSerialPort::ReadWrite))
    {
        QMessageBox::information(this, "串口打开失败", sp->errorString());
    }
    else
    {
        QMessageBox::information(this, "串口打开成功", "串口打开成功");
    }
}

void N10LidarBridgeWidget::slot_socket_connect(bool checked)
{
    if (nullptr == this->socket)
    {
        this->socket = new QTcpSocket(this);
        connect(socket, SIGNAL(readyRead()), this, SLOT(slot_socket_data_recv()));
    }

    QString addr = this->socket_addr->text();
    QString port = this->socket_port->text();

    this->socket->connectToHost(QHostAddress(addr), port.toInt(), QTcpSocket::ReadWrite);

    if (this->socket->waitForConnected(3000))
    {
        QMessageBox::information(this, "服务器连接成功", "服务器连接成功");
    }
    else
    {
        QMessageBox::information(this, "服务器连接失败", this->socket->errorString());
    }
}

void N10LidarBridgeWidget::slot_bridge_connect(bool checked)
{

}

void N10LidarBridgeWidget::slot_sp_data_recv()
{
    while (this->sp->bytesAvailable() > 0)
    {
        sp_raw_data += this->sp->readAll();
    }

    while (sp_raw_data.length() > 0)
    {
        struct WHEELTEC_N10_FRAME *frame = nullptr;
        int32_t processd_size = wheeltec_n10_frame_unpack((uint8_t *)sp_raw_data.data(), sp_raw_data.length(), &frame);

        qDebug() << "ready read serialport bytes: " << sp_raw_data.length()
                << "processed size: " << processd_size;


        if (nullptr != frame)
        {
            qDebug() << "speed: " << (frame->speed_h << 8) + frame->speed_l
                    << "start angle: " << (frame->start_angle_h << 8) + frame->start_angle_l
                    << "stop angle: " << (frame->stop_angle_h << 8) + frame->stop_angle_l;
        }

        sp_raw_data.remove(0, processd_size);
    }

}

void N10LidarBridgeWidget::slot_socket_data_recv()
{

}
