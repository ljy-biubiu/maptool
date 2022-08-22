#ifndef TOOLBAR_H
#define TOOLBAR_H

#include <QToolBar>

QT_FORWARD_DECLARE_CLASS(QAction)
QT_FORWARD_DECLARE_CLASS(QActionGroup)
QT_FORWARD_DECLARE_CLASS(QMenu)
QT_FORWARD_DECLARE_CLASS(QSpinBox)

class MainWindow;
#include "mainwindow.h"

class ToolBar : public QToolBar
{
    Q_OBJECT

public:
    explicit ToolBar(const QString &title, MainWindow *mainwindow_, QWidget *parent = nullptr);

    QMenu *toolbarMenu() const { return menu; }

public:
    QToolButton *filebutton;
    QToolButton *mapbutton;
    QToolButton *slamfilebutton;
    QToolButton *databutton;
    QToolButton *savebutton;
    QToolButton *exportbutton;
    QToolButton *modebutton;
    QToolButton *isviewbutton;
    QToolButton *colorbutton;
    QToolButton *sizebutton;
    QToolButton *mapchangebutton;
    QToolButton *gridbutton;
    QToolButton *view2d3dbutton;
    QToolButton *imagebutton;
    // QToolButton *updatebutton;
    QToolButton *uploadbutton;
    QToolButton *testmapbutton;
    QToolButton *docksviewbutton;
    QToolButton *clearbutton;
    QToolButton *zerobutton;
    QToolButton *versionbutton;
    QToolButton *cloudLinkbutton;

private Q_SLOTS:
    QToolButton *creatQToolButtonMenu();

private:
    void allow(Qt::ToolBarArea area, bool allow);
    void place(Qt::ToolBarArea area, bool place);

    QMenu *menu;
    QMenu *pcdmenu;
    QMenu *vmapmenu;
    QMenu *savemenu;
    QMenu *viewAngle;
    QMenu *paintertool;
    QAction *orderAction;
    QAction *randomizeAction;

    QAction *movableAction;

    QActionGroup *allowedAreasActions;
    QAction *allowLeftAction;
    QAction *allowRightAction;
    QAction *allowTopAction;
    QAction *allowBottomAction;

    QActionGroup *areaActions;
    QAction *leftAction;
    QAction *rightAction;
    QAction *topAction;
    QAction *bottomAction;

    MainWindow *mainwindow;
};

#endif // TOOLBAR_H
