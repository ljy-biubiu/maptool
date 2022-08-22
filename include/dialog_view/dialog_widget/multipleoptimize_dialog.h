#ifndef MULTIPLEOPTIMIZE_DIALOG_H
#define MULTIPLEOPTIMIZE_DIALOG_H

#include "dockwidget_dialog.h"

// class Unoptimized_map;
// #include "unoptimized_map.h"

//创建网格设置窗口
class MultipleOptimizeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MultipleOptimizeDialog(QWidget *parent = Q_NULLPTR);
    void readDialogShowPose(); 
    void setDialogClosePose();
    void setCheckBoxLayout(QList<QMap<QString, QString>>);
    void getCheckBoxIndex(std::vector<int> &);

// public Q_SLOTS:


private:
    void deleteItem(QLayout *);
    
    QString OPTIMIZEKey = "OPTIMIZE";
    QString OPTIMIZEtime = "OPTIMIZETIME";

    QList<QCheckBox *> listCheckBox_;
    QGridLayout *multiplelayout_;
    // Unoptimized_map *unoptimize_;
};

#endif // MULTIPLEOPTIMIZE_DIALOG_H
