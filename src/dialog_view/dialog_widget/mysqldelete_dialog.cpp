#include "mysqldelete_dialog.h"

#include <stdlib.h>

//创建网格设置窗口
MysqlDeleteDialog::MysqlDeleteDialog(QWidget *parent)
    :QDialog(parent)
    ,projectname_combox(new QComboBox(this))
{
    setWindowTitle(tr("数据库项目删除"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);
    
    projectname_combox->addItem("全部");
    layout->addWidget(new QLabel(tr("项目:")), 0, 0, 1, 1);
    layout->addWidget(projectname_combox, 0, 1, 1, 6);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 1, 5, 1, 2);
}

void MysqlDeleteDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/mysqldelete.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("mysqldeleteGeometry/x").toInt();
  int y = settings->value("mysqldeleteGeometry/y").toInt();
  int width = settings->value("mysqldeleteGeometry/width").toInt();
  int height = settings->value("mysqldeleteGeometry/height").toInt();
  QDesktopWidget* desktopWidget = QApplication::desktop();

  QRect targRect = QRect(x,y,width,height);
  this->setGeometry(targRect);//设置主窗口的大小
}

void MysqlDeleteDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/mysqldelete.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("mysqldeleteGeometry/x",this->x());
  settings->setValue("mysqldeleteGeometry/y",this->y());
  settings->setValue("mysqldeleteGeometry/width",this->width());
  settings->setValue("mysqldeleteGeometry/height",this->height());
}