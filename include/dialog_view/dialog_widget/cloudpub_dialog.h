#ifndef CLOUDPUB_DIALOG_H
#define CLOUDPUB_DIALOG_H

#include "dockwidget_dialog.h"

class MainWindow;
#include "mainwindow.h"

//创建网格设置窗口
class CloudPubDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CloudPubDialog(MainWindow *mainwindow_, QWidget *parent = Q_NULLPTR);
    QString getParkIdText() const { return parkId_edit->text(); }
    int getDataTypeIndex() const { return dataType_combox->currentIndex() % 3; }
    void readDialogShowPose(); 
    void setDialogClosePose();
    bool disConnectToClear();
    bool connectToClear();

public Q_SLOTS:
    void prerelease_checkbox_clicked(bool);
    void produce_checkbox_clicked(bool);
    void initpose_checkbox_clicked(bool);
    void mapdata_checkbox_clicked(bool);
    void alldata_checkbox_clicked(bool);

public:
    QComboBox *parkName_combox = nullptr;
    QLineEdit *parkId_edit = nullptr;
    QCheckBox *prerelease_checkbox = nullptr;
    QCheckBox *produce_checkbox = nullptr;
    QCheckBox *initpose_checkbox = nullptr;
    QCheckBox *mapdata_checkbox = nullptr;
    QCheckBox *alldata_checkbox = nullptr;
    QCheckBox *create_checkbox = nullptr;
    QCheckBox *modify_checkbox = nullptr;
    QCheckBox *delete_checkbox = nullptr;
    QComboBox *dataType_combox = nullptr;

private:
    void setDataTypeCombox();
    MainWindow *mainwindow;

};

#endif // CLOUDPUB_DIALOG_H
