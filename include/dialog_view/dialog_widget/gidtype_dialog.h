#ifndef GIDTYPE_DIALOG_H
#define GIDTYPE_DIALOG_H

#include "dockwidget_dialog.h"

class MainWindow;
#include "mainwindow.h"

enum Edit_Index {
    WEBID = 0,
    LORA,
    WIFINAME,
    WIFIPASS,
    SIGNAL,
    EDIT_NUM
};
//创建强度信息窗口
class SetGidDialog : public QDialog
{
public:
    explicit SetGidDialog(MainWindow *mainwindow_, QWidget *parent = Q_NULLPTR);

    int getCurrentGidNumb() const { return gidnumb_spinbox->value(); }
    QString enteredGidWebId() const { return gidwebid_edit->text(); }
    QString enteredGidLoar() const { return gidlora_edit->text(); }
    QString enteredGidWifiName() const { return gidwifiname_edit->text(); }
    QString enteredGidWifiPass() const { return gidwifipass_edit->text(); }
    QString enteredGidSignal() const { return gidsignal_edit->text(); }
    void readDialogShowPose();
    void setDialogClosePose();
    void setEditEnabled(bool, bool);

    QSpinBox *gidnumb_spinbox;
    QLineEdit *gidwebid_edit;
    QLineEdit *gidlora_edit;
    QLineEdit *gidwifiname_edit;
    QLineEdit *gidwifipass_edit;
    QLineEdit *gidsignal_edit;

public Q_SLOTS:
    void gidnumb_spinbox_clicked(bool);

private:
    void getQLineEditStyle(QLineEdit * & lineedit);
    MainWindow *mainwindow;
};

#endif // GIDTYPE_DIALOG_H
