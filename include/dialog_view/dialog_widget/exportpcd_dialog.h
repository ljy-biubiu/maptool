#ifndef EXPORTPCD_DIALOG_H
#define EXPORTPCD_DIALOG_H


#include "dockwidget_dialog.h"

class MainWindow;
#include "mainwindow.h"

//创建重载pcd窗口
class ExportCloudPcdDialog : public QDialog
{
public:
    explicit ExportCloudPcdDialog(MainWindow *mainwindow,QWidget *parent = Q_NULLPTR);

    double getCurrentFilterNum() { return filter_size_spinbox->value(); }
    void readDialogShowPose();
    void setDialogClosePose();

private:
    QCheckBox *default_type;
    QPushButton *original_pcd_ex;
    QDoubleSpinBox *filter_size_spinbox;
    QPushButton *filter_pcd_ex;
    QPushButton *data_file_pbtn;
    
    MainWindow *mainwindow_;
};
#endif // EXPORTPCD_DIALOG_H
