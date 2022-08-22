#ifndef DOCKSVIEW_DIALOG_H
#define DOCKSVIEW_DIALOG_H

#include "dockwidget_dialog.h"

class MainWindow;
#include "mainwindow.h"
class MapWidget;
#include "map_widget.h"

//创建网格设置窗口
class DocksVehicleViewDialog : public QDialog
{
public:
    explicit DocksVehicleViewDialog(MainWindow *mainwindow, MapWidget *mapwidget_, QWidget *parent = Q_NULLPTR);
    void readDialogShowPose(); 
    void setDialogClosePose();

public:
    QComboBox *projectname_combox = nullptr;
    QComboBox *vehicleinfo_combox = nullptr;

private:
    MainWindow *mainwindow_;
    MapWidget *mapwidget_;
};

#endif // DOCKSVIEW_DIALOG_H
