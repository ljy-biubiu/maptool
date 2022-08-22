#include "gridvalue_dialog.h"

#include <stdlib.h>

//创建网格设置窗口
GetGridValueDialog::GetGridValueDialog(MapWidget *mapwidget,MainWindow *mainwindow,QWidget *parent)
    :QDialog(parent)
    ,mapwidget_(mapwidget)
    ,mainwindow_(mainwindow)
    ,color_info(new QLineEdit(this))
    ,count_edit(new QLineEdit(this))
    ,length_edit(new QLineEdit(this))
    ,width_edit(new QLineEdit(this))
{
    setWindowTitle(tr("网格设置"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    gird_color_pbtn = new QPushButton("...", this);
    gird_color_pbtn->setFixedSize(15,15);

    layout->addWidget(new QLabel(tr("颜色")), 1, 0);
    layout->addWidget(color_info, 1, 1);
    layout->addWidget(gird_color_pbtn, 1, 2);

    layout->addWidget(new QLabel(tr("数量")), 0, 0);
    layout->addWidget(count_edit, 0, 1);
    layout->addWidget(new QLabel(tr("长度")), 2, 0);
    layout->addWidget(length_edit, 2, 1);
    layout->addWidget(new QLabel(tr("宽度")), 3, 0);
    layout->addWidget(width_edit, 3, 1);

    connect(gird_color_pbtn, SIGNAL(clicked()), mainwindow, SLOT(getGridValidColor()));

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(gridsize_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(gridsize_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 4, 0, 1, 2);
}

void GetGridValueDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/gridvalue.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("gridvalueGeometry/x").toInt();
  int y = settings->value("gridvalueGeometry/y").toInt();
  int width = settings->value("gridvalueGeometry/width").toInt();
  int height = settings->value("gridvalueGeometry/height").toInt();
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

void GetGridValueDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/gridvalue.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("gridvalueGeometry/x",this->x());
  settings->setValue("gridvalueGeometry/y",this->y());
  settings->setValue("gridvalueGeometry/width",this->width());
  settings->setValue("gridvalueGeometry/height",this->height());
}