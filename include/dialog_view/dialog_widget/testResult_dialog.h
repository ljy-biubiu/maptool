#ifndef TESTRESULT_DIALOG_H
#define TESTRESULT_DIALOG_H

#include "dockwidget_dialog.h"

class CTest;
#include "test/ctest.h"

//创建网格设置窗口
class TestResultDialog : public QDialog
{
public:
    explicit TestResultDialog(CTest *ctest,QWidget *parent = Q_NULLPTR);
    void readDialogShowPose();
    void setDialogClosePose();

private:
    std::string resulterr;
    std::string dataresult;
    std::string clearattresult;
    QLabel *resulterr_label;
    QLabel *dataresult_label;
    QLabel *clearattresult_label;
    CTest *ctest_;
};

#endif // TESTRESULT_DIALOG_H
