#ifndef MYSQLDELETE_DIALOG_H
#define MYSQLDELETE_DIALOG_H

#include "dockwidget_dialog.h"


//创建网格设置窗口
class MysqlDeleteDialog : public QDialog
{
public:
    explicit MysqlDeleteDialog(QWidget *parent = Q_NULLPTR);
    QString const getComboBoxText() { return projectname_combox->currentText(); }
    void readDialogShowPose(); 
    void setDialogClosePose();

public:
    QComboBox *projectname_combox = nullptr;

};

#endif // MYSQLDELETE_DIALOG_H
