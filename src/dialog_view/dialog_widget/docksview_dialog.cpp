#include "docksview_dialog.h"

#include <stdlib.h>

//创建网格设置窗口
DocksVehicleViewDialog::DocksVehicleViewDialog(MainWindow *mainwindow, MapWidget *mapwidget, QWidget *parent)
    :QDialog(parent)
    ,mapwidget_(mapwidget)
    ,mainwindow_(mainwindow)
    ,projectname_combox(new QComboBox(this))
    ,vehicleinfo_combox(new QComboBox(this))
{
    setWindowTitle(tr("桩点车辆调试信息"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);
    // std::unordered_map<std::string, std::vector<std::string>> robots_info = 
    //                                             mapwidget->getVehicleRobotName();
    // if (!robots_info.empty()) {
    //     vehicleinfo_combox->addItem(tr("全部"));
    //     for (auto robot_info : robots_info) {
    //         for (auto robot_i : robot_info.second) {
    //             QString robotname = QString::fromStdString(robot_i);
    //             vehicleinfo_combox->addItem(robotname);
    //         }
    //         projectname_edit->setText(QString::fromStdString(robot_info.first));
    //         projectname_edit->setEnabled(false);
    //     }
    // }
    
    layout->addWidget(new QLabel(tr("园区:")), 0, 0);
    layout->addWidget(projectname_combox, 0, 1);
    layout->addWidget(new QLabel(tr("车辆:")), 0, 2);
    layout->addWidget(vehicleinfo_combox, 0, 3);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(docksview_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(docksview_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 1, 1, 1, 2);
}

void DocksVehicleViewDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/docksview.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("docksviewGeometry/x").toInt();
  int y = settings->value("docksviewGeometry/y").toInt();
  int width = settings->value("docksviewGeometry/width").toInt();
  int height = settings->value("docksviewGeometry/height").toInt();
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

void DocksVehicleViewDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/docksview.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("docksviewGeometry/x",this->x());
  settings->setValue("docksviewGeometry/y",this->y());
  settings->setValue("docksviewGeometry/width",this->width());
  settings->setValue("docksviewGeometry/height",this->height());
}