#include "aixcolor_dialog.h"

#include <stdlib.h>

//创建重载pcd窗口
SetAixColorDialog::SetAixColorDialog(MapWidget *mapwidget,QWidget *parent)
    :QDialog(parent)
    ,mapwidget_(mapwidget)
    ,z_min(new QLineEdit(this))
    ,z_max(new QLineEdit(this))
{
    setWindowTitle(tr("高度信息:"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    //判断
    if (fabs(mapwidget->getPcdZmin()) > DOUBLE_MIN && fabs(mapwidget->getPcdZmax()) < DOUBLE_MAX) {
      z_min->setText(QString::number(mapwidget->getPcdZmin(), 'f', 1));
      z_max->setText(QString::number(mapwidget->getPcdZmax(), 'f', 1));
    } else {
      z_min->setText("");
      z_max->setText("");
    }

    layout->addWidget(new QLabel(tr("高度最大值：")), 0, 0);
    layout->addWidget(z_max, 0, 1);
    layout->addWidget(new QLabel(tr("高度最小值：")), 1, 0);
    layout->addWidget(z_min, 1, 1);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 2, 0, 1, 2);
}

void SetAixColorDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/aixcolor.ini";
  std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("aixcolorGeometry/x").toInt();
  int y = settings->value("aixcolorGeometry/y").toInt();
  int width = settings->value("aixcolorGeometry/width").toInt();
  int height = settings->value("aixcolorGeometry/height").toInt();
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

void SetAixColorDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/aixcolor.ini";
  std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("aixcolorGeometry/x",this->x());
  settings->setValue("aixcolorGeometry/y",this->y());
  settings->setValue("aixcolorGeometry/width",this->width());
  settings->setValue("aixcolorGeometry/height",this->height());
}