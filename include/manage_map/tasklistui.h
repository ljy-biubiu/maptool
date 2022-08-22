#ifndef TASKLISTUI_H
#define TASKLISTUI_H

#include <QWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "QCheckBox"
#include "QLineEdit"
#include "QComboBox"
#include "QRadioButton"
#include "QSpinBox"
#include "QHeaderView"

class MainWindow;
#include "mainwindow.h"

class TaskListUi : public QWidget
{
    Q_OBJECT
public:
    explicit TaskListUi(MainWindow *mainwindow_ = nullptr);

private:
    MainWindow *mainwindow;

    void getQRadiobuttonItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,QRadioButton * & radiobutton,const QString &info);
    void getQCheckBoxItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,QCheckBox * & checkbox,const QString &info);
    void getQLineEditItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,QLineEdit * & lineedit,const QString &info);
    void getQComboBoxItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,QComboBox * & combobox,const QString &info);
    void getQSpinBoxItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,QSpinBox * & spinbox,const QString &info);
    void getQLabelItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,QLabel * & label,const QString &info);
    void getQPushButtonItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,QPushButton * & pushbutton,const QString &info);
    
    QCheckBox* creteQCheckBoxItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,const QString &info);
    QLineEdit* creteQlineEditItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,const QString &info);
    QComboBox* creteQComboBoxEditItem(QTreeWidget* & mainTreeWidget,QTreeWidgetItem* & mainItem,const QString &info,const QStringList &list);

// signals:

// public slots:
};

#endif // TASKLISTUI_H
