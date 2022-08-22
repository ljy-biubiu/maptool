#ifndef REGULAR_DIALOG_H
#define REGULAR_DIALOG_H

#include "dockwidget_dialog.h"

class MainWindow;
#include "mainwindow.h"

//创建网格设置窗口
class RegularInfoDialog : public QDialog
{
public:
    explicit RegularInfoDialog(MainWindow *mainwindow, QWidget *parent = Q_NULLPTR);
    void readDialogShowPose();
    void setDialogClosePose();

public:
    QLineEdit *refid_edit_1 = nullptr;
    QLineEdit *refidrole_edit_1 = nullptr;
    QLineEdit *refid_edit_2 = nullptr;
    QLineEdit *refidrole_edit_2 = nullptr;
    QLineEdit *refid_edit_3 = nullptr;
    QLineEdit *refidrole_edit_3 = nullptr;
    QLineEdit *refid_edit_4 = nullptr;
    QLineEdit *refidrole_edit_4 = nullptr;
    QLineEdit *refid_edit_5 = nullptr;
    QLineEdit *refidrole_edit_5 = nullptr;
    
    QComboBox *maneuver_comboBox = nullptr;
    QComboBox *element_comboBox = nullptr;
    QDialogButtonBox *buttonBox;

private:
    QLineEdit *getLineEditStyle();
    QLineEdit *getLineEditroleStyle();

    MainWindow *mainwindow_;
};

#endif // REGULAR_DIALOG_H
