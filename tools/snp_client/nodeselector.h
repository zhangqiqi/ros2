#ifndef NODESELECTOR_H
#define NODESELECTOR_H

#include <QWidget>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QMap>


class NodeSelector : public QWidget
{
    Q_OBJECT
public:
    explicit NodeSelector(void *snp_handle, QWidget *parent = nullptr);

signals:
    void signal_node_select(int32_t id);

public slots:
    void slot_update_nodes_list(bool checked);
    void slot_select_node(bool checked);
    void slot_update_node_info_lable(QString node_id);
private:
    void *snp;

    QComboBox *nodes_combobox;
    QPushButton *nodes_list_update;
    QPushButton *nodes_list_select;
    QLabel *node_info;

    QMap<int32_t, QString> nodes_info_map;
};

#endif // NODESELECTOR_H
