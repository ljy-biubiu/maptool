#include "exportpcd_dialog.h"

#include <stdlib.h>

//创建重载pcd窗口
ExportCloudPcdDialog::ExportCloudPcdDialog(MainWindow *mainwindow,QWidget *parent)
    :QDialog(parent)
    ,mainwindow_(mainwindow)
    ,default_type(new QCheckBox(this))
    ,original_pcd_ex(new QPushButton("导出原始点云",this))
    ,filter_size_spinbox(new QDoubleSpinBox(this))
    ,filter_pcd_ex(new QPushButton("导出滤波后原始点云",this))
    ,data_file_pbtn(new QPushButton("导出原始数据文件",this))
{
    setWindowTitle(tr("导出pcd文件:"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);
    
    filter_size_spinbox->setValue(0.20);
    default_type->setChecked(true);
    filter_size_spinbox->setSingleStep(0.10);

    layout->addWidget(default_type, 0, 0, 1, 1);
    layout->addWidget(new QLabel(tr("默认模式")), 0, 1, 1, 1);
    layout->addWidget(original_pcd_ex, 1, 0, 1, 2);
    layout->addWidget(filter_size_spinbox, 2, 0, 1, 2);
    layout->addWidget(filter_pcd_ex, 3, 0, 1, 2);
    layout->addWidget(data_file_pbtn, 4, 0, 1, 2);

    connect(original_pcd_ex, SIGNAL(clicked()), mainwindow, SLOT(originalpcdsave_pbtn_clicked()));
    connect(filter_pcd_ex, SIGNAL(clicked()), mainwindow, SLOT(filterpcdsave_pbtn_clicked()));
    connect(data_file_pbtn, SIGNAL(clicked()), mainwindow, SLOT(datafilesave_pbtn_clicked()));

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(exportclod_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(exportclod_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 5, 0, 1, 2);
}

void ExportCloudPcdDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/exportpcd.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("exportpcdGeometry/x").toInt();
  int y = settings->value("exportpcdGeometry/y").toInt();
  int width = settings->value("exportpcdGeometry/width").toInt();
  int height = settings->value("exportpcdGeometry/height").toInt();
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

void ExportCloudPcdDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/exportpcd.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("exportpcdGeometry/x",this->x());
  settings->setValue("exportpcdGeometry/y",this->y());
  settings->setValue("exportpcdGeometry/width",this->width());
  settings->setValue("exportpcdGeometry/height",this->height());
}
