#include "mainwindow.h"
#include "n10lidarbridgewidget.h"

#include <QTabWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QTabWidget *tabwidget = new QTabWidget(this);
    this->setCentralWidget(tabwidget);

    N10LidarBridgeWidget *n10bridge_widget = new N10LidarBridgeWidget(tabwidget);
    tabwidget->addTab(n10bridge_widget, "N10 Lidar Bridge");
}

MainWindow::~MainWindow()
{
}

