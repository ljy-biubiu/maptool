#include "multipleoptimize_dialog.h"

#include <stdlib.h>

//创建网格设置窗口
MultipleOptimizeDialog::MultipleOptimizeDialog(QWidget *parent)
    :QDialog(parent)
{
    setWindowTitle(tr("待优化多选列表"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    multiplelayout_ = new QGridLayout(this);
}

void MultipleOptimizeDialog::setCheckBoxLayout(QList<QMap<QString, QString>> listOptimizeDatas) {
  listCheckBox_.clear();
  deleteItem(multiplelayout_);
  int optimizeNum = listOptimizeDatas.size();
  for (size_t i = 0; i < listOptimizeDatas.size(); i++)
  {
    QString boxName = listOptimizeDatas.at(i).find(OPTIMIZEKey).value() + "-" + 
                      listOptimizeDatas.at(i).find(OPTIMIZEtime).value();
    QCheckBox *checkBox = new QCheckBox(this);
    listCheckBox_.push_back(checkBox);
    multiplelayout_->addWidget(new QLabel(boxName),i,1);
    multiplelayout_->addWidget(checkBox,i,0);
  }
  QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
  multiplelayout_->addWidget(buttonBox, optimizeNum, 1, 1, 2);
}

void MultipleOptimizeDialog::getCheckBoxIndex(std::vector<int> &listIndex) {
  listIndex.clear();
  for (size_t i = 0; i < listCheckBox_.size(); i++)
  {
    QCheckBox *checkBox = listCheckBox_.at(i);
    if (checkBox->isChecked())
    {
      listIndex.push_back(i);
    }
  }
}

/*删除layout*/ 
void MultipleOptimizeDialog::deleteItem(QLayout *layout)
{
    QLayoutItem *child;
    while ((child = layout->takeAt(0)) != nullptr)
    {
        //setParent为NULL，防止删除之后界面不消失
        if (child->widget())
        {
            child->widget()->setParent(nullptr);
            delete child->widget();
        }
        else if (child->layout())
        {
            deleteItem(child->layout());
            child->layout()->deleteLater();
        }
        delete child;
    }
}

void MultipleOptimizeDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/multipleoptimize.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("multipleoptimizeGeometry/x").toInt();
  int y = settings->value("multipleoptimizeGeometry/y").toInt();
  int width = settings->value("multipleoptimizeGeometry/width").toInt();
  int height = settings->value("multipleoptimizeGeometry/height").toInt();
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

void MultipleOptimizeDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/multipleoptimize.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("multipleoptimizeGeometry/x",this->x());
  settings->setValue("multipleoptimizeGeometry/y",this->y());
  settings->setValue("multipleoptimizeGeometry/width",this->width());
  settings->setValue("multipleoptimizeGeometry/height",this->height());
}