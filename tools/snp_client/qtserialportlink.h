#ifndef QTSERIALPORTLINK_H
#define QTSERIALPORTLINK_H

#include <QObject>

class QtSerialportLink : public QObject
{
    Q_OBJECT
public:
    explicit QtSerialportLink(void *snp_handle, QString portname, qint32 baudrate, QObject *parent = nullptr);

signals:

public slots:
};

#endif // QTSERIALPORTLINK_H
