#include "qttcplink.h"

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"

#include <QTcpSocket>

#ifdef __cplusplus
extern "C" {
#endif

static int32_t snp_qt_tcp_link_read(void *handle, struct SNP_BUFFER *buffer)
{
    QTcpSocket *link = (QTcpSocket *)handle;

    if (link->bytesAvailable() > 0)
    {
        QByteArray data = link->readAll();

        snp_buffer_write(buffer, (uint8_t *)data.data(), data.size());
    }
}


static int32_t snp_qt_tcp_link_write(void *handle, struct SNP_BUFFER *buffer)
{
    QTcpSocket *link = (QTcpSocket *)handle;

    uint8_t *data = nullptr;
    int32_t len = 1024;

    len = snp_buffer_copyout_ptr(buffer, &data, len);

    link->write((char *)data, len);
    snp_buffer_drain(buffer, len);
}


#ifdef __cplusplus
}
#endif


QtTcpLink::QtTcpLink(void *snp_handle, QString ip, uint16_t port, QObject *parent)
    : QObject(parent)
{
    QTcpSocket *tcp_client = new QTcpSocket(this);

    snp_create_physical_node((struct SNP *)snp_handle, snp_qt_tcp_link_read, snp_qt_tcp_link_write, tcp_client);

    tcp_client->connectToHost(ip, port);
}
