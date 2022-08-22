#include "tag_dialog.h"
#include "manage_map/allstylesheets.h"

#include <stdlib.h>

//创建网格设置窗口
TagInfoDialog::TagInfoDialog(MainWindow *mainwindow, QWidget *parent)
    :QDialog(parent)
    ,mainwindow_(mainwindow)
    ,idtag_label(new QLabel(tr("--"), this))
{
    key_edit_1 = getLineEditStyle();
    value_edit_1 = getLineEditStyle();
    key_edit_2 = getLineEditStyle();
    value_edit_2 = getLineEditStyle();

    setWindowTitle(tr("规则编辑"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    layout->addWidget(idtag_label, 0, 0, 1, 3);
    layout->addWidget(new QLabel(tr("Tag1:")), 1, 0);
    layout->addWidget(new QLabel(tr("Key:")), 1, 1);
    layout->addWidget(key_edit_1, 1, 2);
    layout->addWidget(new QLabel(tr("Value:")), 1, 3);
    layout->addWidget(value_edit_1, 1, 4);
    layout->addWidget(new QLabel(tr("Tag2:")), 2, 0);
    layout->addWidget(new QLabel(tr("Key:")), 2, 1);
    layout->addWidget(key_edit_2, 2, 2);
    layout->addWidget(new QLabel(tr("Value:")), 2, 3);
    layout->addWidget(value_edit_2, 2, 4);

    QWidget::setTabOrder(key_edit_1, value_edit_1);
    QWidget::setTabOrder(value_edit_1, key_edit_2);
    QWidget::setTabOrder(key_edit_2, value_edit_2);

    buttonBox = new QDialogButtonBox(this);
    buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
    buttonBox->setCenterButtons(true);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(taginfo_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(taginfo_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 3, 2, 1, 2);
}

QLineEdit *TagInfoDialog::getLineEditStyle() {
    QLineEdit *lineedit = new QLineEdit();
    lineedit->setStyleSheet(myStyleSheets::myLineEdit::tasklistUiLineEdit);
    lineedit->setMaximumWidth(120);
    lineedit->setText(QString(""));
    return lineedit;
}

void TagInfoDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/taginfo.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("taginfoGeometry/x").toInt();
  int y = settings->value("taginfoGeometry/y").toInt();
  int width = settings->value("taginfoGeometry/width").toInt();
  int height = settings->value("taginfoGeometry/height").toInt();
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

void TagInfoDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/taginfo.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("taginfoGeometry/x",this->x());
  settings->setValue("taginfoGeometry/y",this->y());
  settings->setValue("taginfoGeometry/width",this->width());
  settings->setValue("taginfoGeometry/height",this->height());
}