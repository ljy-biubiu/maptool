#ifndef PCDRELOAD_DIALOG_H
#define PCDRELOAD_DIALOG_H


#include "dockwidget_dialog.h"

class MapWidget;
#include "map_widget/map_widget.h"

//创建重载pcd窗口
class CreatMapHeightDialog : public QDialog
{
public:
    explicit CreatMapHeightDialog(MapWidget *mapwidget,QWidget *parent = Q_NULLPTR);

    QString enteredPcdHeightMin() const { return z_min->text(); }
    QString enteredPcdHeightMax() const { return z_max->text(); }
    QString enteredPcdSzie() const { return pcd_size->text(); }
    bool isCheckBoxChecked();
    void readDialogShowPose();
    void setDialogClosePose();

private:
    QLineEdit *z_min;
    QLineEdit *z_max;
    QLineEdit *pcd_size;
    QCheckBox *zero_back;
    MapWidget *mapwidget_;
};
#endif // PCDRELOAD_DIALOG_H
