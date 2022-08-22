#include "loginUi.h"

#include <QMainWindow>
#include "manage_map/allstylesheets.h"

#include <stdlib.h>

LoginDialog::LoginDialog(MainWindow *mainwindow_, QWidget *parent)
    : QDialog(parent)
    , mainwindow(mainwindow_)
    , prerelease_checkbox(new QCheckBox(this))
    , produce_checkbox(new QCheckBox(this))
    , ordinary_checkbox(new QCheckBox(this))
    , deploy_checkbox(new QCheckBox(this))
{   
    setWindowTitle(tr("Login maptool"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);
    //获取上一次记住状态
    isRemberFlag_ = false;

    setQLineEditStyle(userName);
    setQLineEditStyle(passWord);
    setQCheckBoxStyle(remberBox);
    readRemberedPassword();
    passWord->setEchoMode(QLineEdit::Password);
    prerelease_checkbox->setChecked(true);
    remberBox->setChecked(isRemberFlag_);
    ordinary_checkbox->setChecked(true);

    //环境选择
    layout->addWidget(new QLabel(tr("预发布:")), 0, 0, 1, 1);
    layout->addWidget(prerelease_checkbox, 0, 1, 1, 1);
    layout->addWidget(new QLabel(tr("生产:")), 0, 2, 1, 1);
    layout->addWidget(produce_checkbox, 0, 3, 1, 5);
    //用户权限选择
    layout->addWidget(new QLabel(tr("普通用户:")), 1, 0, 1, 1);
    layout->addWidget(ordinary_checkbox, 1, 1, 1, 1);
    layout->addWidget(new QLabel(tr("部署用户:")), 1, 2, 1, 1);
    layout->addWidget(deploy_checkbox, 1, 3, 1, 5);
    //帐号密码输入
    layout->addWidget(new QLabel(tr("username:")), 2, 0, 1, 1);
    layout->addWidget(userName, 2, 1, 1, 4);
    layout->addWidget(new QLabel(tr("password:")), 3, 0, 1, 1);
    layout->addWidget(passWord, 3, 1, 1, 4);
    layout->addWidget(remberBox, 4, 1, 1, 1);
    layout->addWidget(new QLabel(tr("rember password")), 4, 0, 1, 1);
    
    connect(prerelease_checkbox, SIGNAL(clicked(bool)), this, SLOT(prerelease_checkbox_Clicked(bool)));
    connect(produce_checkbox, SIGNAL(clicked(bool)), this, SLOT(produce_checkbox_Clicked(bool)));
    connect(ordinary_checkbox, SIGNAL(clicked(bool)), this, SLOT(ordinary_checkbox_Clicked(bool)));
    connect(deploy_checkbox, SIGNAL(clicked(bool)), this, SLOT(deploy_checkbox_Clicked(bool)));

    connect(remberBox, SIGNAL(clicked(bool)), this, SLOT(remberBox_checkbox_clicked(bool)));
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(loginui_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(loginui_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 5, 1, 1, 2);
    // addAction()
}

void LoginDialog::prerelease_checkbox_Clicked(bool status) {
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

void LoginDialog::produce_checkbox_Clicked(bool status) {
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

void LoginDialog::ordinary_checkbox_Clicked(bool status) {
  if (status == true && deploy_checkbox->isChecked())
  {
    deploy_checkbox->setChecked(false);
  }
  else {
    if (!deploy_checkbox->isChecked() && !ordinary_checkbox->isChecked())
    {
      ordinary_checkbox->setChecked(true);
    }
  }
}

void LoginDialog::deploy_checkbox_Clicked(bool status) {
  if (status == true && ordinary_checkbox->isChecked())
  {
    ordinary_checkbox->setChecked(false);
  }
  else {
    if (!ordinary_checkbox->isChecked() && !deploy_checkbox->isChecked())
    {
      deploy_checkbox->setChecked(true);
    }
  }
}

void LoginDialog::setQLineEditStyle(QLineEdit * & lineedit)
{
    lineedit = new QLineEdit(this);
    lineedit->setStyleSheet(myStyleSheets::myLineEdit::tasklistUiLineEdit);
    lineedit->setMaximumWidth(150);
}

void LoginDialog::setQCheckBoxStyle(QCheckBox * & checkbox)
{
    checkbox = new QCheckBox(this);
    checkbox->setChecked(false);
    // checkbox->setStyleSheet(myStyleSheets::myCheckBox::tasklistUiCheckBox);
}

void LoginDialog::remberBox_checkbox_clicked(bool status)
{
    isRemberFlag_ = status;
    setRemberedPassword();
}

void LoginDialog::readRemberedPassword() {
  QString savesetting = qApp->applicationDirPath() + "/ini/passconfig.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  QString username = settings->value("passconfigGeometry/username").toString();
  QString password = settings->value("passconfigGeometry/password").toString();
  bool isflag = settings->value("passconfigGeometry/falg").toBool();
  if (isflag)
  {
    userName->setText(username);
    passWord->setText(password);
  }
  isRemberFlag_ = isflag;
}

void LoginDialog::setRemberedPassword() {
  if (remberBox->isChecked())
  {
    QString savesetting = qApp->applicationDirPath() + "/ini/passconfig.ini";
  //   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
    QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
    settings->clear();//清空当前配置文件中的内容
    settings->setValue("passconfigGeometry/username",this->getUserName());
    settings->setValue("passconfigGeometry/password",this->getPassWord());
    settings->setValue("passconfigGeometry/falg",this->isRemberFlag_);

    settings->sync();
  }
}