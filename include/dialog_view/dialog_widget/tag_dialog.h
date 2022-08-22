#ifndef TAG_DIALOG_H
#define TAG_DIALOG_H

#include "dockwidget_dialog.h"

class MainWindow;
#include "mainwindow.h"

//创建网格设置窗口
class TagInfoDialog : public QDialog
{
public:
    explicit TagInfoDialog(MainWindow *mainwindow, QWidget *parent = Q_NULLPTR);
    void readDialogShowPose();
    void setDialogClosePose();

public:
    QLabel *idtag_label = nullptr;
    QLineEdit *key_edit_1 = nullptr;
    QLineEdit *value_edit_1 = nullptr;
    QLineEdit *key_edit_2 = nullptr;
    QLineEdit *value_edit_2 = nullptr;

    QDialogButtonBox *buttonBox;

private:
    QLineEdit *getLineEditStyle();

    MainWindow *mainwindow_;
};

#endif // TAG_DIALOG_H
