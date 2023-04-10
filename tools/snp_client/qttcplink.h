#ifndef QTTCPLINK_H
#define QTTCPLINK_H

#include "snp.h"
#include "snp_defs.h"

#include <QObject>

class QtTcpLink : public QObject
{
    Q_OBJECT
public:
    explicit QtTcpLink(void *snp_handle, QString ip, uint16_t port, QObject *parent = nullptr);

signals:

public slots:
};

#endif // QTTCPLINK_H
