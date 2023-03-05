#ifndef SNPSHELLWIDGET_H
#define SNPSHELLWIDGET_H

#include <QWidget>
#include <QTextEdit>
#include <QLineEdit>
#include <QCheckBox>

class SnpShellWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SnpShellWidget(void *snp_handle, QWidget *parent = nullptr);

    void text_widget_append_str(QString str);
signals:

public slots:
    void slot_set_select_node_id(int32_t node_id);

private slots:
    void cmd_input();


private:
    void *snp;
    int32_t cur_select_node_id;

    QTextEdit *shell_text_widget;
    QLineEdit *cmd_input_edit;
    QCheckBox *broadcast_enable_check;
};

#endif // SNPSHELLWIDGET_H
