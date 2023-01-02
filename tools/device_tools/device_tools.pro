QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets serialport network

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += \
    ../../common/include/ \
    ../../common/wtb_protocol/ \
    ../../device/lidar/wheeltec-N10/include/

SOURCES += \
    ../../common/wtb_protocol/wtb_protocol.c \
    ../../device/lidar/wheeltec-N10/wheeltec_n10_protocol.c \
    main.cpp \
    mainwindow.cpp \
    n10lidarbridgewidget.cpp

HEADERS += \
    ../../common/include/common_defs.h \
    ../../common/wtb_protocol/wtb_protocol.h \
    ../../device/lidar/wheeltec-N10/include/wheeltec_n10_protocol.h \
    mainwindow.h \
    n10lidarbridgewidget.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
