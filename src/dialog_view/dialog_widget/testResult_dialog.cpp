#include "testResult_dialog.h"

#include <stdlib.h>

//创建网格设置窗口
TestResultDialog::TestResultDialog(CTest *ctest,QWidget *parent)
    :QDialog(parent)
    ,ctest_(ctest)
    ,resulterr_label(new QLabel(this))
    ,dataresult_label(new QLabel(this))
    ,clearattresult_label(new QLabel(this))
{
    setWindowTitle(tr("调试结果报告"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    // std::vector<std::string> resulterrs = ctest->resulterrs_;
    if (ctest->resulterrs_.size() > 0) {
        for (auto result : ctest->resulterrs_) {
            resulterr = resulterr + result;
        }
        if (resulterr.length() > 20) {
            resulterr_label->setStyleSheet("color:red;");
        }
    }
    if (ctest->dataresults_.size() > 0) {
        for (auto result : ctest->dataresults_) {
            dataresult = dataresult + result;
        }
        if (dataresult.length() > 20) {
            dataresult_label->setStyleSheet("color:red;");
        }
    }
    if (ctest->clearattresults_.size() > 0) {
        for (auto result : ctest->clearattresults_) {
            clearattresult = clearattresult + result;
        }
        if (clearattresult.length() > 20) {
            clearattresult_label->setStyleSheet("color:red;");
        }
    }

    resulterr_label->setText(QString::fromStdString(resulterr));
    dataresult_label->setText(QString::fromStdString(dataresult));
    clearattresult_label->setText(QString::fromStdString(clearattresult));

    layout->addWidget(new QLabel(tr("路线检测结果:")), 0, 0);
    layout->addWidget(resulterr_label, 0, 1);
    layout->addWidget(new QLabel(tr("数据可靠性检测结果:")), 1, 0);
    layout->addWidget(dataresult_label, 1, 1);
    layout->addWidget(new QLabel(tr("清扫区属性检测结果:")), 2, 0);
    layout->addWidget(clearattresult_label, 2, 1);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 3, 0, 1, 2);
}

void TestResultDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/testresult.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("testresultGeometry/x").toInt();
  int y = settings->value("testresultGeometry/y").toInt();
  int width = settings->value("testresultGeometry/width").toInt();
  int height = settings->value("testresultGeometry/height").toInt();
  QDesktopWidget* desktopWidget = QApplication::desktop();
  // QRect clientRect = desktopWidget->availableGeometry();
  // QRect targRect0 = QRect(clientRect.width()/4,clientRect.height()/4,clientRect.width()/2,clientRect.height()/2);
  QRect targRect = QRect(x,y,width,height);
  //如果上一次关闭软件的时候，窗口位置不正常，则本次显示在显示器的正中央
  // if(width == 0|| height == 0 || x<0 || x>clientRect.width() || y<0 || y>clientRect.height())
  // {
  //     targRect = targRect0;
  // }
  this->setGeometry(targRect);//设置主窗口的大小
}

void TestResultDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/testresult.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("testresultGeometry/x",this->x());
  settings->setValue("testresultGeometry/y",this->y());
  settings->setValue("testresultGeometry/width",this->width());
  settings->setValue("testresultGeometry/height",this->height());
}