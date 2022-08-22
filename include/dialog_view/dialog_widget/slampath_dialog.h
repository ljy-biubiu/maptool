#ifndef SLAMPATH_DIALOG_H
#define SLAMPATH_DIALOG_H

#include "dockwidget_dialog.h"

//创建重载pcd窗口
class GetSlamDataPathDialog : public QDialog
{
public:
    explicit GetSlamDataPathDialog(QWidget *parent = Q_NULLPTR);

    QString enteredSlamDataPath() const { return datapath_edit->text(); }
    bool getVehicleCheckBoxStatus();
    bool getBagsCheckBoxStatus();
    void setDialogClosePose();
    void readDialogShowPose();

public Q_SLOTS:
    void vehicle_checkbox_clicked(bool);
    void bags_checkbox_clicked(bool);

private:
    QLineEdit *datapath_edit;
    QCheckBox *vehicle_box;
    QCheckBox *bags_box;
//     MapWidget *mapwidget_;
};
#endif // SLAMPATH_DIALOG_H
