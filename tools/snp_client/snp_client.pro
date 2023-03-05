#-------------------------------------------------
#
# Project created by QtCreator 2023-03-03T13:47:08
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets network

TARGET = snp_client
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


INCLUDEPATH += ../libsnp/include/ \
               ../libsnp/src/include

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    ../libsnp/src/snp.c \
    ../libsnp/src/snp_buffer.c \
    ../libsnp/src/snp_buffer_link.c \
    ../libsnp/src/snp_internal.c \
    ../libsnp/src/snp_link.c \
    ../libsnp/src/snp_msgs.c \
    ../libsnp/src/snp_node.c \
    ../libsnp/src/snp_std_process.c \
    ../libsnp/src/snp_timer.c \
    qttcplink.cpp \
    snpshellwidget.cpp \
    nodeselector.cpp

HEADERS += \
        mainwindow.h \
    ../libsnp/include/snp.h \
    ../libsnp/include/snp_buffer.h \
    ../libsnp/include/snp_buffer_link.h \
    ../libsnp/include/snp_defs.h \
    ../libsnp/include/snp_link.h \
    ../libsnp/include/snp_msgs.h \
    ../libsnp/include/snp_node.h \
    ../libsnp/include/snp_std_msgs.h \
    ../libsnp/include/snp_timer.h \
    qttcplink.h \
    snpshellwidget.h \
    nodeselector.h
