#ifndef CHOOSEMAPUI_H
#define CHOOSEMAPUI_H

#include <QWidget>
#include <QPushButton>
#include <QSignalMapper>
#include <QScrollArea>
#include <QMouseEvent>

#include <qlabel.h>
#include <QLineEdit>
#include <QGridLayout>

class Choose_map;
#include "manage_map.h"
class DialogWidgetTools;
#include "dialog_view/dialog_view.h"
class MultipleOptimizeDialog;
#include "dialog_view/dialog_widget/multipleoptimize_dialog.h"

class UpdateLog;
class LoadingDataUi;

class ChooseMapUi : public QWidget
{
    Q_OBJECT
public:
    explicit ChooseMapUi(QWidget *parent = nullptr);
    bool getDownLoadDataSuccess();
    bool eventFilter(QObject *obj, QEvent *event);

    QScrollArea * getScrollArea();
    QPushButton * upgradeProjectTimeButton;
    QPushButton * createNewProjectButton;
    QPushButton * taskProjectButton;
    // QPushButton * drawHelpButton;
    QPushButton * ordinaryProjectButton;
    UpdateLog *updatelog;
    LoadingDataUi *loadingDataUi;
    QGridLayout *cardLayout;
    // QGridLayout *taskLayout;

    QString CardCommonStyleSheet;
    QString MainButtonCommonStyleSheet;


private:
    DialogWidgetTools *dialogView_;

    QString projectKey= "PROJECT";
    QString MAPkey= "MAP";
    QString VMAPkey= "VMAP";
    QString LOCALVMAPkey = "LOCALVMAP";
    QString LOCALMAPkey = "LOCALMAP";
    QString COMPAREVERTIONkey = "COMPAREVERTION";
    QString VMAPtime = "VMAPTIME";
    QString MAPtime = "MAPTIME";
    QString LOCALVMAPtime = "LOCALVMAPTIME";
    QString LOCALMAPtime = "LOCALMAPTIME";

    QString TASKkey = "TASK";
    QString LOCALTASKkey = "LOCALTASK";
    //--
    QString TASKtime = "TASKTIME";
    QString LOCALTASKtime = "LOCALTASKTIME";
    //--
    QString DATAsign = "DATASIGN";
    QString LOCALDATAsign = "LOCALDATASIGN";
    //--
    QString OPTIMIZEKey = "OPTIMIZE";
    QString LOCALOPIMIZEKey = "LOCALOPTIMIZE";
    //--
    QString OPTIMIZEtime = "OPTIMIZETIME";
    QString LOCALOPTIMIZEtime = "LOCALOPTIMIZETIME";

    QWidget* widget;
    QList< QPushButton*> button_list;
    QList< QPushButton*> taskbutton_list_;
    // QList< QMenu*> menu_list_;
    // QList< QMenu*> taskmenu_list_;
    QSignalMapper* signalMapper;
    QSignalMapper* tasksignalMapper;
    QSignalMapper* optimizesignalMapper;
    QScrollArea* scrollArea ;
    QList<bool> button_list_status;
    QList<QMap<QString, QString>> listMapDatasCP_;
    QList<QMap<QString, QString>> listTaskDatasCP_;
    QList<QMap<QString, QString>> listOptimizeDatasCP_;
    QLineEdit* searchBox_;

    QMenu *taskMenu_;
    QMenu *ordinaryMenu_;
    QAction *multipleAct_;
    QAction *deletAct_;
    QAction *completeAct_;
    QAction *abandonAct_;
    QAction *modifyAct_;
    QAction *deletOrdinaryAct_;
    QAction *modifyOrdinaryAct_;

    QList<QAction *> actList_;
    int ordinaryIndex_;
    int taskIndex_;

    bool downLoadDataSuccess{false};
    void reloadButtons(QList< QPushButton *> &);
    void layoutCardButtons(const QList<int> &, QList< QPushButton *> &);
    void deleteItem(QLayout *layout);
    void createButtons(QPushButton* ,const int &,const int &,const QList<QMap<QString,QString>> &);
    void createTaskButtons(QPushButton *, const int &,const int &,const QList<QMap<QString, QString>> &);
    void createOptimizeButtons(QPushButton *,const int &,const int &,const QList<QMap<QString, QString>> &);
    void initTaskRightMenuAndAct();
    void initRightMenuAndAct();
//    void setUpUi();

// private Q_SLOTS:
//     void on_pushButton_customContextMenuRequested(const QPoint &);
//     void on_pushButton_task_customContextMenuRequested(const QPoint &);

Q_SIGNALS:
    void sendButtonArrayData(int);
    void sendTaskButtonArrayData(int);
    void sendOptimizeButtonArrayData(int);
    void createNewProjectSwitch(QString);
    void uploadProjectNameSwitch(QString);
    void uploadProjectSwitch();
    void refreshChooseUi();
    void sendMapDataButtonDelet(int);
    void sendOptimizeButtonAbandon(int);
    void sendTaskButtonAbandon(int);
    void sendOptimizeButtonDelet(int);
    void sendTaskButtonDelet(int);
    void sendOptimizeButtonComplete(int);
    void sendTaskButtonComplete(int);
    void sendOptimizeButtonModify(int);
    void sendTaskButtonModify(int);
    void sendMultipleOptimizeButtonData(std::vector<int>);
    void sentDeleteMysqlProject(QString);
    void sentModifyMysql();
    void sentCreateMysql();

public Q_SLOTS:
    void getMapDataTotal(bool ,QList<QMap<QString,QString>>);
    void getTaskDataList(bool ,QList<QMap<QString, QString>>);
    void getOptimizeDataList(bool ,QList<QMap<QString, QString>>);
    void sendButtonRank(int);
    void sendTaskButtonRank(int);
    void sendOptimizeButtonRank(int);
    void reLoadMapUiSlot(int);
    void reLoadTaskUiSlot(int);
    void reLoadOptimizeUiSlot(int);
    void searchBox(QString);
    void createDialog();
    void getNewProjectPath();
    void getNewProjectTime();
    void uploadProjectName(QString);
    void loadingDatas();
    void EndloadingDatas();
    void taskProjectList_pbtn_clicked();
    void ordinaryProjectList_pbtn_clicked();
    void multiple_act_triggered();

    void delet_act_triggered();
    void complete_act_triggered();
    void abandon_act_triggered();
    void modify_act_triggered();
    void deletOrdinary_act_triggered();
    void modifyOrdinary_act_triggered();

    void ordinarycustomContextMenuRequested(const QPoint &);
    void taskcustomContextMenuRequested(const QPoint &);

    void deleteMysqlProject(QString);
    void modifyMysqlProject();
    void createMysqlProject();
};

#endif // CHOOSEMAPUI_H
