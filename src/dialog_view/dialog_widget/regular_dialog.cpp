#include "regular_dialog.h"
#include "manage_map/allstylesheets.h"

#include <stdlib.h>

//创建网格设置窗口
RegularInfoDialog::RegularInfoDialog(MainWindow *mainwindow, QWidget *parent)
    :QDialog(parent)
    ,mainwindow_(mainwindow)
    ,maneuver_comboBox(new QComboBox(this))
    ,element_comboBox(new QComboBox(this))
{
    refid_edit_1 = getLineEditStyle();
    refid_edit_2 = getLineEditStyle();
    refid_edit_3 = getLineEditStyle();
    refid_edit_4 = getLineEditStyle();
    refid_edit_5 = getLineEditStyle();

    refidrole_edit_1 = getLineEditroleStyle();
    refidrole_edit_2 = getLineEditroleStyle();
    refidrole_edit_3 = getLineEditroleStyle();
    refidrole_edit_4 = getLineEditroleStyle();
    refidrole_edit_5 = getLineEditroleStyle();

    setWindowTitle(tr("规则编辑"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    layout->addWidget(new QLabel(tr("ref_id(1):")), 0, 0);
    layout->addWidget(refid_edit_1, 0, 1);
    layout->addWidget(refidrole_edit_1, 0, 2);
    layout->addWidget(new QLabel(tr("ref_id(2):")), 1, 0);
    layout->addWidget(refid_edit_2, 1, 1);
    layout->addWidget(refidrole_edit_2, 1, 2);
    layout->addWidget(new QLabel(tr("ref_id(3):")), 2, 0);
    layout->addWidget(refid_edit_3, 2, 1);
    layout->addWidget(refidrole_edit_3, 2, 2);
    layout->addWidget(new QLabel(tr("ref_id(4):")), 3, 0);
    layout->addWidget(refid_edit_4, 3, 1);
    layout->addWidget(refidrole_edit_4, 3, 2);
    layout->addWidget(new QLabel(tr("ref_id(5):")), 4, 0);
    layout->addWidget(refid_edit_5, 4, 1);
    layout->addWidget(refidrole_edit_5, 4, 2);
    layout->addWidget(new QLabel(tr("属性:")), 5, 0);
    layout->addWidget(maneuver_comboBox, 5, 1);
    layout->addWidget(element_comboBox, 5, 2);

    QWidget::setTabOrder(refid_edit_1, refidrole_edit_1);
    QWidget::setTabOrder(refidrole_edit_1, element_comboBox);
    QWidget::setTabOrder(element_comboBox, maneuver_comboBox);
    QWidget::setTabOrder(maneuver_comboBox, refid_edit_2);
    QWidget::setTabOrder(refid_edit_2, refidrole_edit_2);
    QWidget::setTabOrder(refidrole_edit_2, refid_edit_3);
    QWidget::setTabOrder(refid_edit_3, refidrole_edit_3);
    QWidget::setTabOrder(refidrole_edit_3, refid_edit_4);
    QWidget::setTabOrder(refid_edit_4, refidrole_edit_4);
    QWidget::setTabOrder(refidrole_edit_4, refid_edit_5);
    QWidget::setTabOrder(refid_edit_5, refidrole_edit_5);

    buttonBox = new QDialogButtonBox(this);
    buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
    buttonBox->setCenterButtons(true);
    connect(buttonBox, SIGNAL(rejected()), mainwindow, SLOT(regulinfo_dialog_cancelpBtn_clicked()));
    connect(buttonBox, SIGNAL(accepted()), mainwindow, SLOT(regulinfo_dialog_okpBtn_clicked()));
    layout->addWidget(buttonBox, 7, 1, 1, 2);
}

QLineEdit *RegularInfoDialog::getLineEditStyle() {
    QLineEdit *lineedit = new QLineEdit();
    lineedit->setStyleSheet(myStyleSheets::myLineEdit::tasklistUiLineEdit);
    lineedit->setMaximumWidth(120);
    lineedit->setText(QString(""));
    return lineedit;
}
QLineEdit *RegularInfoDialog::getLineEditroleStyle() {
    QLineEdit *lineedit = new QLineEdit();
    lineedit->setStyleSheet(myStyleSheets::myLineEdit::tasklistUiLineEdit);
    lineedit->setMaximumWidth(150);
    lineedit->setText(QString(""));
    return lineedit;
}

void RegularInfoDialog::readDialogShowPose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/regular.ini";
//   std::cout << "-------------------------read pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  int x = settings->value("regularGeometry/x").toInt();
  int y = settings->value("regularGeometry/y").toInt();
  int width = settings->value("regularGeometry/width").toInt();
  int height = settings->value("regularGeometry/height").toInt();
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

void RegularInfoDialog::setDialogClosePose() {
  QString savesetting = qApp->applicationDirPath() + "/ini/regular.ini";
//   std::cout << "-------------------------save pose: " <<  savesetting.toStdString() << std::endl;
  QSettings *settings = new QSettings(savesetting,QSettings::IniFormat);//用QSetting获取ini文件中的数据
  settings->clear();//清空当前配置文件中的内容
  settings->setValue("regularGeometry/x",this->x());
  settings->setValue("regularGeometry/y",this->y());
  settings->setValue("regularGeometry/width",this->width());
  settings->setValue("regularGeometry/height",this->height());
}