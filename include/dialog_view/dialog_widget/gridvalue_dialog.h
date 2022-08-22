#ifndef GRIDVALUE_DIALOG_H
#define GRIDVALUE_DIALOG_H

#include "dockwidget_dialog.h"

class MapWidget;
#include "map_widget/map_widget.h"
class MainWindow;
#include "mainwindow.h"

//创建网格设置窗口
class GetGridValueDialog : public QDialog
{
public:
    explicit GetGridValueDialog(MapWidget *mapwidget, MainWindow *mainwindow, QWidget *parent = Q_NULLPTR);
    QString enteredCellColor() const { return color_info->text(); }
    QString enteredCellCount() const { return count_edit->text(); }
    QString enteredCellLength() const { return length_edit->text(); }
    QString enteredCellWidth() const { return width_edit->text(); }
    void readDialogShowPose();
    void setDialogClosePose();
//     Ogre::ColourValue getQColor();
public:
    QLineEdit *color_info;
    QLineEdit *count_edit;
    QLineEdit *length_edit;
    QLineEdit *width_edit;
// public Q_SLOTS:
//     void getGridValidColor();

private:
    QPushButton *gird_color_pbtn;
    MapWidget *mapwidget_;
    MainWindow *mainwindow_;
};

#endif // GRIDVALUE_DIALOG_H
