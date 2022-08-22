#ifndef AIXCOLOR_DIALOG_H
#define AIXCOLOR_DIALOG_H


#include "dockwidget_dialog.h"

class MapWidget;
#include "map_widget/map_widget.h"

//创建重载pcd窗口
class SetAixColorDialog : public QDialog
{
public:
    explicit SetAixColorDialog(MapWidget *mapwidget,QWidget *parent = Q_NULLPTR);

    QString enteredPcdHeightMin() const { return z_min->text(); }
    QString enteredPcdHeightMax() const { return z_max->text(); }
    void setDialogClosePose();
    void readDialogShowPose();

private:
    QLineEdit *z_min;
    QLineEdit *z_max;
    MapWidget *mapwidget_;
};
#endif // AIXCOLOR_DIALOG_H
