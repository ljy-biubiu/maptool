#ifndef DOCKWIDGET_DIALOG_H
#define DOCKWIDGET_DIALOG_H

#include <QToolBar>
#include <QDialog>
#include <QMainWindow>
#include <QMenu>
#include <QPainter>
#include <QPainterPath>
#include <QSpinBox>
#include <QLabel>
#include <QToolTip>
#include <QToolButton>
#include <QActionGroup>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <mainwindow.h>
#include <QApplication>

// extern QString savesetting;
QT_FORWARD_DECLARE_CLASS(QAction)
QT_FORWARD_DECLARE_CLASS(QActionGroup)
QT_FORWARD_DECLARE_CLASS(QMenu)
QT_FORWARD_DECLARE_CLASS(QSpinBox)

//创建新的dockwidget
class CreateDockWidgetDialog : public QDialog
{
public:
    explicit CreateDockWidgetDialog(QWidget *parent = Q_NULLPTR);

    QString enteredObjectName() const { return m_objectName->text(); }
    Qt::DockWidgetArea location() const;
    void readDialogShowPose();
    void setDialogClosePose();

private:
    QLineEdit *m_objectName;
    QComboBox *m_location;
};

#endif // DOCKWIDGET_DIALOG_H
