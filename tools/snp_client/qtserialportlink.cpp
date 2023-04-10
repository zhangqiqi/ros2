#include "qtserialportlink.h"

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"

#include <QSerialPort>


#ifdef __cplusplus
extern "C" {
#endif

static snp_qt_serialport_link_read(void *handle, struct SNP_BUFFER *buffer)
{
    QSerialPort *link = (QSerialPort *)handle;

    if (link->bytesAvailable() > 0)
    {
        QByteArray data = link->readAll();

        snp_buffer_write(buffer, (uint8_t *)data.data(), data.size());
    }
}


static snp_qt_serialport_link_write(void *handle, struct SNP_BUFFER *buffer)
{
    QSerialPort *link = (QSerialPort *)handle;

    uint8_t *data = nullptr;
    int32_t len = 1024;

    len = snp_buffer_copyout_ptr(buffer, &data, len);

    link->write((char *)data, len);
    snp_buffer_drain(buffer, len);
}


#ifdef __cplusplus
}
#endif

QtSerialportLink::QtSerialportLink(void *snp_handle, QString portname, qint32 baudrate, QObject *parent)
    : QObject(parent)
{
    QSerialPort *sp = new QSerialPort(this);

    snp_create_physical_node((struct SNP *)snp_handle, snp_qt_serialport_link_read, snp_qt_serialport_link_write, sp);

    sp->setPortName(portname);
    sp->setBaudRate(baudrate);

    sp->open(QSerialPort::ReadWrite);
}

