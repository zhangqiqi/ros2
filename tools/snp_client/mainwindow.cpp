#include "mainwindow.h"

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"

#include "qttcplink.h"
#include "snpshellwidget.h"
#include "nodeselector.h"

#include <QTimer>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), snp_handle(nullptr), interval(10)
{
    snp_set_log_level(SLT_NOTICE);

    snp_handle = snp_create("relay_server", SDT_RELAY_SERVER, 999);
    QtTcpLink *new_tcp_link = new QtTcpLink(snp_handle, "127.0.0.1", 9999, this);

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(snp_exec_timer()));
    timer->start(interval);

    QVBoxLayout *layout = new QVBoxLayout();

    NodeSelector *node_selector = new NodeSelector(snp_handle, this);

    QTabWidget *tab_widget = new QTabWidget(this);
    SnpShellWidget *shell_widget = new SnpShellWidget(snp_handle, tab_widget);
    connect(node_selector, SIGNAL(signal_node_select(int32_t)), shell_widget, SLOT(slot_set_select_node_id(int32_t)));

    tab_widget->addTab(shell_widget, "shell");

    layout->addWidget(node_selector);
    layout->addWidget(tab_widget);

    QWidget *central_widget = new QWidget(this);
    central_widget->setLayout(layout);

    setCentralWidget(central_widget);
}

MainWindow::~MainWindow()
{

}

void MainWindow::snp_exec_timer()
{
    snp_exec((struct SNP *)snp_handle, interval);
}
