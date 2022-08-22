#ifndef INTENSITY_DIALOG_H
#define INTENSITY_DIALOG_H

#include "dockwidget_dialog.h"

//创建强度信息窗口
class CreatIntensityDialog : public QDialog
{
public:
    explicit CreatIntensityDialog(QWidget *parent = Q_NULLPTR);

    QString enteredIntensityMin() const { return intensity_min->text(); }
    QString enteredIntensityMax() const { return intensity_max->text(); }
    void readDialogShowPose();
    void setDialogClosePose();

private:
    QLineEdit *intensity_min;
    QLineEdit *intensity_max;
};

#endif // INTENSITY_DIALOG_H
