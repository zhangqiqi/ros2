#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void snp_exec_timer();


private:
    void *snp_handle;
    int32_t interval;
};

#endif // MAINWINDOW_H
