#ifndef DIALOG_VIEW_H
#define DIALOG_VIEW_H

#include <QDir>
#include "dialog_widget/dockwidget_dialog.h"
#include "dialog_widget/gridvalue_dialog.h"
#include "dialog_widget/intensity_dialog.h"
#include "dialog_widget/pcdreload_dialog.h"
#include "dialog_widget/testResult_dialog.h"
// #include "dialog_widget/regular_dialog.h"
#include "dialog_widget/aixcolor_dialog.h"
#include "dialog_widget/slampath_dialog.h"
#include "dialog_widget/exportpcd_dialog.h"
#include "dialog_widget/mysqldelete_dialog.h"
class MainWindow;
#include "mainwindow.h"

//创建网格设置窗口
class DialogWidgetTools : public QMainWindow
{
public:
    DialogWidgetTools();
    ~DialogWidgetTools();

    void setGroupAction(QAction *action, QMenu *menu, 
                        QActionGroup *group, QList<QAction *> list);
    void createPushButtonSize(QPushButton *pushbutton, const QString &info, QList<QPushButton *> list);
    void creatQToolButton(QToolButton *toolButton, const QString &icon, const QString &info, bool status);
    void setupMenuAddAction(QAction *action, const QString &icon,
                            QMenu *menu, QList<QAction *> list);
    void setupMenuAddAction(QAction *action, QMenu *menu, QList<QAction *> list);
    void setupMenuAddAction(QAction *action, QMenu *menu, 
                            QActionGroup *actiongroup, QList<QAction *> list);
    bool setMainWidgetView(QList<QSplitter *> extraSplitters, QSplitter *splitter, bool status,bool state);
    void setStatusBarLabelView(int size, QLabel *label);
    
    void SplitString(const std::string& s, std::vector<std::string>& v, const std::string &c);
    QStringList projectStringDeal(const std::string& home_str, QString &project);
    void deleteItem(QLayout *layout);
    void updateComboBox(QComboBox *cb, const std::string *names, 
                        int num, bool isShow);
    void updateQLabel(QLabel *cb, bool isShow);
    void updateQLineEdit(QLineEdit *cb, bool isShow);
    void updateQPushButton(QPushButton *cb, bool isShow);
    void deleteQActionsIter(QList<QAction *> &qActions);
    void deleteQPushButtonsIter(QList<QPushButton *> &qPushButtons);
    QString changeColorNumberToString(Ogre::ColourValue& color);
    Ogre::ColourValue changeColorNumberToString(QColor color);
    bool isFileExistance(QString &path, QString &suffix, QStringList &string_list);
    bool getFileName(QString &path, QString &suffix, QStringList &string_list);
    bool isReadOk(QString &file);
    bool delDir(const QString &path);
    std::vector<std::string> spaceSplit(std::string const &input);
    
private:
    MainWindow *mainwindow_;
};

#endif // DIALOG_VIEW_H
