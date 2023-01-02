#ifndef N10LIDARBRIDGEWIDGET_H
#define N10LIDARBRIDGEWIDGET_H

#include <QWidget>
#include <QTcpSocket>
#include <QTimer>
#include <QtSerialPort/QSerialPort>

#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>


class N10LidarBridgeWidget : public QWidget
{
    Q_OBJECT
public:
    explicit N10LidarBridgeWidget(QWidget *parent = nullptr);

    void serialportWidgetInit(void);

    void tcpSocketWidgetInit(void);
signals:

private slots:
    void slot_sp_connect(bool checked = false);
    void slot_socket_connect(bool checked = false);
    void slot_bridge_connect(bool checked = false);

    void slot_sp_data_recv();
    void slot_socket_data_recv();
private:
    QSerialPort *sp;
    QTcpSocket *socket;

    QComboBox *sp_list;
    QLineEdit *sp_baudrate;
    QPushButton *sp_connect;

    QLineEdit *socket_addr;
    QLineEdit *socket_port;
    QPushButton *socket_connect;

    QPushButton *bridge_connect;

    QByteArray sp_raw_data;
};

#endif // N10LIDARBRIDGEWIDGET_H
