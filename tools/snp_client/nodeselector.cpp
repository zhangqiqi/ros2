#include "nodeselector.h"

#include "snp.h"
#include "snp_node.h"
#include "snp_buffer.h"
#include "snp_msgs.h"
#include "snp_node_internal.h"
#include "snp_buffer_link.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDebug>

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif


NodeSelector::NodeSelector(void *snp_handle, QWidget *parent)
    : QWidget(parent), snp(snp_handle)
{
    QVBoxLayout *vbox_layout = new QVBoxLayout();
    setLayout(vbox_layout);

    QHBoxLayout *hbox_layout = new QHBoxLayout();
    nodes_combobox = new QComboBox(this);
    connect(nodes_combobox, SIGNAL(currentIndexChanged(QString)), this, SLOT(slot_update_node_info_lable(QString)));

    QPushButton *nodes_list_select = new QPushButton("select", this);
    connect(nodes_list_select, SIGNAL(clicked(bool)), this, SLOT(slot_select_node(bool)));

    QPushButton *nodes_list_update = new QPushButton("update", this);
    connect(nodes_list_update, SIGNAL(clicked(bool)), this, SLOT(slot_update_nodes_list(bool)));

    hbox_layout->addWidget(nodes_combobox);
    hbox_layout->addWidget(nodes_list_select);
    hbox_layout->addWidget(nodes_list_update);

    vbox_layout->addLayout(hbox_layout);

    node_info = new QLabel(this);
    vbox_layout->addWidget(node_info);
}

void NodeSelector::slot_update_nodes_list(bool checked)
{
    struct SNP_NODE_INFO nodes[32];

    int32_t num = snp_get_nodes_info((struct SNP *)snp, nodes, 32);

    nodes_combobox->clear();
    nodes_info_map.clear();

    for (int i = 0; i < num; i++)
    {
        QString str = QString("id: %1, name: %2, type: %3")
                .arg(QString::number(nodes[i].id))
                .arg(nodes[i].name)
                .arg(QString::number(nodes[i].type));

        nodes_combobox->addItem(QString::number(nodes[i].id));

        nodes_info_map.insert(nodes[i].id, str);
    }

    slot_update_node_info_lable(nodes_combobox->currentText());
}

void NodeSelector::slot_select_node(bool checked)
{
    emit signal_node_select(nodes_combobox->currentText().toInt());
}

void NodeSelector::slot_update_node_info_lable(QString node_id)
{
    node_info->setText(nodes_info_map.value(node_id.toInt()));
}
