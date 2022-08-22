#include "slampath_dialog.h"

#include <stdlib.h>

//创建重载pcd窗口
GetSlamDataPathDialog::GetSlamDataPathDialog(QWidget *parent)
    :QDialog(parent)
    ,datapath_edit(new QLineEdit(this))
    ,vehicle_box(new QCheckBox(this))
    ,bags_box(new QCheckBox(this))
{
    setWindowTitle(tr("原始数据处理:"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    vehicle_box->setChecked(true);    
    layout->addWidget(new QLabel(tr("数据路径:")), 0, 0, 1, 1);
    layout->addWidget(datapath_edit, 0, 1, 1, 7);
    layout->addWidget(vehicle_box, 1, 0, 1, 1);
    layout->addWidget(new QLabel(tr("车端建图数据")), 1, 1, 1, 1);
    layout->addWidget(bags_box, 1, 2, 1, 1);
    layout->addWidget(new QLabel(tr("背包建图数据")), 1, 3, 1, 1);

    connect(vehicle_box, SIGNAL(clicked(bool)), this, SLOT(vehicle_checkbox_clicked(bool)));
    connect(bags_box, SIGNAL(clicked(bool)), this, SLOT(bags_checkbox_clicked(bool)));

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 2, 3, 1, 2);
}
void GetSlamDataPathDialog::vehicle_checkbox_clicked(bool status)
{
  if (vehicle_box->isChecked())
  {
    bags_box->setChecked(false);
  }
}
void GetSlamDataPathDialog::bags_checkbox_clicked(bool status)
{
  if (bags_box->isChecked())
  {
    vehicle_box->setChecked(false);
  }
}
bool GetSlamDataPathDialog::getVehicleCheckBoxStatus() {
  if (vehicle_box->isChecked())
  {
    bags_box->setChecked(false);
    return true;
  }
  else
  {
    return false;
  }
}

bool GetSlamDataPathDialog::getBagsCheckBoxStatus() {
  if (bags_box->isChecked())
  {
    vehicle_box->setChecked(false);
    return true;
  }
  else
  {
    return false;
  }
}

void GetSlamDataPathDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/slampath.ini";
  std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("slampathGeometry/x").toInt();
  int y = settings->value("slampathGeometry/y").toInt();
  int width = settings->value("slampathGeometry/width").toInt();
  int height = settings->value("slampathGeometry/height").toInt();
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

void GetSlamDataPathDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/slampath.ini";
  std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("slampathGeometry/x",this->x());
  settings->setValue("slampathGeometry/y",this->y());
  settings->setValue("slampathGeometry/width",this->width());
  settings->setValue("slampathGeometry/height",this->height());
}