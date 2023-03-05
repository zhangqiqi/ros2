#include "snpshellwidget.h"

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"

#include <QDebug>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QLineEdit>

#ifdef __cplusplus
extern "C" {
#endif


static int32_t snp_shell_client_recv_shell_res_msg(void *cb_handle, struct SNP_LINK *link, struct SNP_FRAME *msg)
{
    SnpShellWidget *snp_shell = (SnpShellWidget *)cb_handle;
    struct SSM_SHELL_RES_MSG *_msg = (struct SSM_SHELL_RES_MSG *)msg->payload;

    snp_shell->text_widget_append_str(_msg->res_str);
    return 0;
}


#ifdef __cplusplus
}
#endif


SnpShellWidget::SnpShellWidget(void *snp_handle, QWidget *parent)
    : QWidget(parent), snp(snp_handle)
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    shell_text_widget = new QTextEdit(this);
    shell_text_widget->setReadOnly(true);
    cmd_input_edit = new QLineEdit(this);
    broadcast_enable_check = new QCheckBox("使能广播", this);

    layout->addWidget(shell_text_widget);
    layout->addWidget(broadcast_enable_check);
    layout->addWidget(cmd_input_edit);

    this->setLayout(layout);

    connect(cmd_input_edit, SIGNAL(returnPressed()), this, SLOT(cmd_input()));

    snp_node_listen_msg_to(snp_get_local_node((struct SNP *)snp), SSM_SHELL_RES, snp_shell_client_recv_shell_res_msg, (void *)this);
}

void SnpShellWidget::text_widget_append_str(QString str)
{
    shell_text_widget->textCursor().atEnd();
    shell_text_widget->textCursor().insertText(str);
    shell_text_widget->textCursor().insertText("\n");
    shell_text_widget->moveCursor(QTextCursor::End);
}

void SnpShellWidget::slot_set_select_node_id(int32_t node_id)
{
    cur_select_node_id = node_id;
}

void SnpShellWidget::cmd_input()
{
    QString str = cmd_input_edit->text();
    cmd_input_edit->clear();

    char shell_str[256] = {0};
    struct SSM_SHELL_REQ_MSG *_shell_msg = (struct SSM_SHELL_REQ_MSG *)shell_str;

    _shell_msg->req_len = str.length() + 1;
    strncpy(_shell_msg->req_str, str.toStdString().c_str(), sizeof(shell_str) - sizeof(struct SSM_SHELL_REQ_MSG));

    if (broadcast_enable_check->isChecked())
    {
        snp_broadcast_msg((struct SNP *)snp, SSM_SHELL_REQ, _shell_msg, sizeof(struct SSM_SHELL_REQ_MSG) + _shell_msg->req_len);
    }
    else
    {
        qDebug() << "send cmd : " << SSM_SHELL_REQ << " to node id " << cur_select_node_id;

        snp_send_msg_by_id((struct SNP *)snp, cur_select_node_id, SSM_SHELL_REQ, _shell_msg, sizeof(struct SSM_SHELL_REQ_MSG) + _shell_msg->req_len);
    }
}
