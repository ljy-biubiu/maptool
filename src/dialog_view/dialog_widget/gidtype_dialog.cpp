#include "gidtype_dialog.h"
#include "manage_map/allstylesheets.h"

#include <stdlib.h>

//创建强度信息窗口
SetGidDialog::SetGidDialog(MainWindow *mainwindow_, QWidget *parent)
    :QDialog(parent)
    ,mainwindow(mainwindow_)
    ,gidnumb_spinbox(new QSpinBox(this))
{
    setWindowTitle(tr("gid类型设置"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    getQLineEditStyle(gidwifiname_edit);
    getQLineEditStyle(gidwifipass_edit);
    getQLineEditStyle(gidwebid_edit);
    getQLineEditStyle(gidlora_edit);
    getQLineEditStyle(gidsignal_edit);

    layout->addWidget(new QLabel(tr("gid序号：")), 0, 0);
    layout->addWidget(gidnumb_spinbox, 0, 1);
    layout->addWidget(new QLabel(tr("WIFI名称：")), 1, 0);
    layout->addWidget(gidwifiname_edit, 1, 1);
    layout->addWidget(new QLabel(tr("WIFI密码：")), 2, 0);
    layout->addWidget(gidwifipass_edit, 2, 1);
    layout->addWidget(new QLabel(tr("网络ID：")), 3, 0);
    layout->addWidget(gidwebid_edit, 3, 1);
    layout->addWidget(new QLabel(tr("LORA频率：")), 4, 0);
    layout->addWidget(gidlora_edit, 4, 1);
    layout->addWidget(new QLabel(tr("信号灯信号：")), 5, 0);
    layout->addWidget(gidsignal_edit, 5, 1);

    connect(gidnumb_spinbox, SIGNAL(valueChanged(int)), mainwindow, SLOT(gidnumb_spinbox_valueChanged(int)));
  
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(setgid_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(setgid_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 6, 0, 1, 2);
}

void SetGidDialog::getQLineEditStyle(QLineEdit * & lineedit)
{
    lineedit = new QLineEdit(this);
    lineedit->setEnabled(false);
    lineedit->setStyleSheet(myStyleSheets::myLineEdit::tasklistUiLineEdit);
    lineedit->setMaximumWidth(150);
}

//设置edit编辑权限
void SetGidDialog::setEditEnabled(bool status, bool state) {
    gidwebid_edit->setEnabled(status);
    gidlora_edit->setEnabled(status);
    // gidwifiname_edit->setEnabled(status);
    // gidwifipass_edit->setEnabled(status);
    gidsignal_edit->setEnabled(state);
    //清空edit内容，方便下一次填写
    gidwebid_edit->setText("");
    gidlora_edit->setText("");
    gidsignal_edit->setText("");
}

void SetGidDialog::readDialogShowPose() {
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

void SetGidDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/intensity.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("intensityGeometry/x",this->x());
  settings->setValue("intensityGeometry/y",this->y());
  settings->setValue("intensityGeometry/width",this->width());
  settings->setValue("intensityGeometry/height",this->height());
}