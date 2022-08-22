#include "intensity_dialog.h"

#include <stdlib.h>

//创建强度信息窗口
CreatIntensityDialog::CreatIntensityDialog(QWidget *parent)
    :QDialog(parent)
    ,intensity_min(new QLineEdit(this))
    ,intensity_max(new QLineEdit(this))
{
    setWindowTitle(tr("强度信息"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    layout->addWidget(new QLabel(tr("强度最小值：")), 0, 0);
    layout->addWidget(intensity_min, 0, 1);
    layout->addWidget(new QLabel(tr("强度最大值：")), 1, 0);
    layout->addWidget(intensity_max, 1, 1);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 2, 0, 1, 2);
}

void CreatIntensityDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/intensity.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("intensityGeometry/x").toInt();
  int y = settings->value("intensityGeometry/y").toInt();
  int width = settings->value("intensityGeometry/width").toInt();
  int height = settings->value("intensityGeometry/height").toInt();
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

void CreatIntensityDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/intensity.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("intensityGeometry/x",this->x());
  settings->setValue("intensityGeometry/y",this->y());
  settings->setValue("intensityGeometry/width",this->width());
  settings->setValue("intensityGeometry/height",this->height());
}