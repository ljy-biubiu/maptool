#include "cloudpub_dialog.h"

#include <stdlib.h>

//创建网格设置窗口
CloudPubDialog::CloudPubDialog(MainWindow *mainwindow_, QWidget *parent)
    :QDialog(parent)
    ,mainwindow(mainwindow_)
    ,parkName_combox(new QComboBox(this))
    ,parkId_edit(new QLineEdit(this))
    ,prerelease_checkbox(new QCheckBox(this))
    ,produce_checkbox(new QCheckBox(this))
    ,initpose_checkbox(new QCheckBox(this))
    ,mapdata_checkbox(new QCheckBox(this))
    ,alldata_checkbox(new QCheckBox(this))
    ,dataType_combox(new QComboBox(this))
    ,create_checkbox(new QCheckBox(this))
    ,modify_checkbox(new QCheckBox(this))
    ,delete_checkbox(new QCheckBox(this))
{
    setWindowTitle(tr("发布-平台:"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    // create_checkbox = new QCheckBox(this);
    // modify_checkbox = new QCheckBox(this);
    // delete_checkbox = new QCheckBox(this);
    // prerelease_checkbox->setChecked(true);
    //设置不可编辑
    setDataTypeCombox();
    create_checkbox->setEnabled(false);
    modify_checkbox->setEnabled(false);
    delete_checkbox->setEnabled(false);
    prerelease_checkbox->setEnabled(false);
    produce_checkbox->setEnabled(false);

    alldata_checkbox->setChecked(true);
    
    layout->addWidget(new QLabel(tr("预发布:")), 0, 0, 1, 1);
    layout->addWidget(prerelease_checkbox, 0, 1, 1, 1);
    layout->addWidget(new QLabel(tr("生产:")), 0, 2, 1, 1);
    layout->addWidget(produce_checkbox, 0, 3, 1, 5);

    layout->addWidget(new QLabel(tr("发布内容:")), 1, 0, 1, 1);
    layout->addWidget(new QLabel(tr("初始定位点")), 1, 1, 1, 1);
    layout->addWidget(initpose_checkbox, 1, 2, 1, 1);
    layout->addWidget(new QLabel(tr("地图数据信息")), 1, 3, 1, 1);
    layout->addWidget(mapdata_checkbox, 1, 4, 1, 1);
    layout->addWidget(new QLabel(tr("以上两种")), 1, 5, 1, 1);
    layout->addWidget(alldata_checkbox, 1, 6, 1, 1);

    layout->addWidget(new QLabel(tr("操作:")), 2, 0, 1, 1);
    layout->addWidget(new QLabel(tr("新建")), 2, 1, 1, 1);
    layout->addWidget(create_checkbox, 2, 2, 1, 1);
    layout->addWidget(new QLabel(tr("修改")), 2, 3, 1, 1);
    layout->addWidget(modify_checkbox, 2, 4, 1, 1);
    layout->addWidget(new QLabel(tr("删除")), 2, 5, 1, 1);
    layout->addWidget(delete_checkbox, 2, 6, 1, 1);

    layout->addWidget(new QLabel(tr("园区名称:")), 3, 0, 1, 1);
    layout->addWidget(parkName_combox, 3, 1, 1, 5);
    layout->addWidget(new QLabel(tr("园区ID:")), 3, 6, 1, 1);
    layout->addWidget(parkId_edit, 3, 7, 1, 5);
    layout->addWidget(new QLabel(tr("数据类型:")), 3, 12, 1, 1);
    layout->addWidget(dataType_combox, 3, 13, 1, 5);

    connect(parkName_combox, SIGNAL(currentIndexChanged(int)), mainwindow, SLOT(parkName_comboBox_IndexChanged()));
    connect(prerelease_checkbox, SIGNAL(clicked(bool)), this, SLOT(prerelease_checkbox_clicked(bool)));
    connect(produce_checkbox, SIGNAL(clicked(bool)), this, SLOT(produce_checkbox_clicked(bool)));
    connect(initpose_checkbox, SIGNAL(clicked(bool)), this, SLOT(initpose_checkbox_clicked(bool)));
    connect(mapdata_checkbox, SIGNAL(clicked(bool)), this, SLOT(mapdata_checkbox_clicked(bool)));
    connect(alldata_checkbox, SIGNAL(clicked(bool)), this, SLOT(alldata_checkbox_clicked(bool)));

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(cloudpub_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(cloudpub_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 4, 12, 1, 2);
}

bool CloudPubDialog::disConnectToClear() {
    disconnect(parkName_combox, SIGNAL(currentIndexChanged(int)), mainwindow, SLOT(parkName_comboBox_IndexChanged()));
    return true;
}

bool CloudPubDialog::connectToClear() {
    connect(parkName_combox, SIGNAL(currentIndexChanged(int)), mainwindow, SLOT(parkName_comboBox_IndexChanged()));
    return true;
}

void CloudPubDialog::prerelease_checkbox_clicked(bool status) {
  if (status == true && produce_checkbox->isChecked())
  {
    produce_checkbox->setChecked(false);
  }
  else {
    if (!prerelease_checkbox->isChecked() && !produce_checkbox->isChecked())
    {
      prerelease_checkbox->setChecked(true);
    }
  }
}
void CloudPubDialog::produce_checkbox_clicked(bool status) {
  if (status == true && prerelease_checkbox->isChecked())
  {
    prerelease_checkbox->setChecked(false);
  }
  else {
    if (!prerelease_checkbox->isChecked() && !produce_checkbox->isChecked())
    {
      produce_checkbox->setChecked(true);
    }
  }
}

void CloudPubDialog::initpose_checkbox_clicked(bool status) {
  if (status == true && (mapdata_checkbox->isChecked() || alldata_checkbox->isChecked()))
  {
    mapdata_checkbox->setChecked(false);
    alldata_checkbox->setChecked(false);
  }
  else {
    if (!initpose_checkbox->isChecked() && !alldata_checkbox->isChecked() && !mapdata_checkbox->isChecked())
    {
      initpose_checkbox->setChecked(true);
    }
  }
  dataType_combox->setEnabled(false);
}
void CloudPubDialog::mapdata_checkbox_clicked(bool status) {
  if (status == true && (initpose_checkbox->isChecked() || alldata_checkbox->isChecked()))
  {
    initpose_checkbox->setChecked(false);
    alldata_checkbox->setChecked(false);
  }
  else {
    if (!initpose_checkbox->isChecked() && !alldata_checkbox->isChecked() && !mapdata_checkbox->isChecked())
    {
      mapdata_checkbox->setChecked(true);
    }
  }
  dataType_combox->setEnabled(true);
}
void CloudPubDialog::alldata_checkbox_clicked(bool status) {
  if (status == true && (initpose_checkbox->isChecked() || mapdata_checkbox->isChecked()))
  {
    initpose_checkbox->setChecked(false);
    mapdata_checkbox->setChecked(false);
  }
  else {
    if (!initpose_checkbox->isChecked() && !alldata_checkbox->isChecked() && !mapdata_checkbox->isChecked())
    {
      alldata_checkbox->setChecked(true);
    }
  }
  dataType_combox->setEnabled(true);
}

void CloudPubDialog::setDataTypeCombox() {
  dataType_combox->addItem("CLEAN");
  dataType_combox->addItem("DRIVE");
  dataType_combox->addItem("ALL");
}

void CloudPubDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/cloudpub.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("cloudpubGeometry/x").toInt();
  int y = settings->value("cloudpubGeometry/y").toInt();
  int width = settings->value("cloudpubGeometry/width").toInt();
  int height = settings->value("cloudpubGeometry/height").toInt();
  QDesktopWidget* desktopWidget = QApplication::desktop();

  QRect targRect = QRect(x,y,width,height);

  this->setGeometry(targRect);//设置主窗口的大小
}

void CloudPubDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/cloudpub.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("cloudpubGeometry/x",this->x());
  settings->setValue("cloudpubGeometry/y",this->y());
  settings->setValue("cloudpubGeometry/width",this->width());
  settings->setValue("cloudpubGeometry/height",this->height());
}