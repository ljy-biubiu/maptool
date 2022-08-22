#include "pcdreload_dialog.h"

#include <stdlib.h>

//创建重载pcd窗口
CreatMapHeightDialog::CreatMapHeightDialog(MapWidget *mapwidget,QWidget *parent)
    :QDialog(parent)
    ,mapwidget_(mapwidget)
    ,z_min(new QLineEdit(this))
    ,z_max(new QLineEdit(this))
    ,pcd_size(new QLineEdit(this))
    ,zero_back(new QCheckBox(this))
{
    setWindowTitle(tr("重载pcd:"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    //判断
    if (fabs(mapwidget->getPcdZmin()) > DOUBLE_MIN && fabs(mapwidget->getPcdZmax()) < DOUBLE_MAX) {
      z_min->setText(QString::number(mapwidget->getPcdZmin(), 'f', 2));
      z_max->setText(QString::number(mapwidget->getPcdZmax(), 'f', 2));
    } else {
      z_min->setText("--");
      z_max->setText("--");
    }
    
    if (fabs(mapwidget->getPcdSize()) > FLOAT_MIN) {
      pcd_size->setText(QString::number(mapwidget->getPcdSize(), 'f', 2));
    } else {
      pcd_size->setText("--");
    }

    layout->addWidget(zero_back, 0, 0);
    layout->addWidget(new QLabel(tr("回归零点")),0,1);
    layout->addWidget(new QLabel(tr("高度最大值：")), 1, 0);
    layout->addWidget(z_max, 1, 1);
    layout->addWidget(new QLabel(tr("高度最小值：")), 2, 0);
    layout->addWidget(z_min, 2, 1);
    layout->addWidget(new QLabel(tr("点云大小:")), 3, 0);
    layout->addWidget(pcd_size, 3, 1);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 4, 0, 1, 2);
}
bool CreatMapHeightDialog::isCheckBoxChecked() {
  if (zero_back->isChecked())
  {
    return true;
  }
  else {
    return false;
  }
}

void CreatMapHeightDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/pcdreload.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("pcdreloadGeometry/x").toInt();
  int y = settings->value("pcdreloadGeometry/y").toInt();
  int width = settings->value("pcdreloadGeometry/width").toInt();
  int height = settings->value("pcdreloadGeometry/height").toInt();
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

void CreatMapHeightDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/pcdreload.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("pcdreloadGeometry/x",this->x());
  settings->setValue("pcdreloadGeometry/y",this->y());
  settings->setValue("pcdreloadGeometry/width",this->width());
  settings->setValue("pcdreloadGeometry/height",this->height());
}
