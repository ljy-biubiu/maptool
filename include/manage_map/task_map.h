#ifndef TASK_MAP_H
#define TASK_MAP_H

#include <QObject>
#include <QFile>
#include <thread>
#include <qmap.h>
#include <unistd.h>
#include <QMessageBox>
#include <sys/types.h>
#include <future>   //std::future std::promise
#include <utility>  //std::ref
#include <dirent.h> //windows开发工具没有这个头文件

const std::string home_str_ = std::string(std::getenv("HOME"));

class ChooseMapUi;
#include "choosemapui.h"
class DialogWidgetTools;
#include "dialog_view/dialog_view.h"

class MainWindow;
#include "mainwindow.h"

enum Tar_N : int {
  Map = 0,       //滤波地图包名
  Vmap,          //原始地图包名
  RMap,          //高精度地图包名
  Data,          //原始数据包名
  Tar_Num
};

enum Loacl_F {
    TIME = 0,    //时间
    VERSION,     //版本
    INFO,        //信息
    SIGN,        //完成标志
    File_Num
};

class Task_map : public QWidget
{
    Q_OBJECT
public:
    explicit Task_map(ChooseMapUi *chooseMapUi_, MainWindow *mainWindow_, QWidget *parent = nullptr);
    std::tuple<bool,std::string,std::string> uploadDataing(const int &, QString);
    void SplitString(const std::string&, std::vector<std::string>&, const std::string &);
    bool checkoutSign(const int &);
    const int searchProjectRank(const QString &);
    //MainWindow* mainWindow() const;

    ChooseMapUi *chooseMapUi;

private:
    bool pullTaskDataList();
    bool pullProjectListName(const int &);
    bool getTaskProjectName();
    void readLocalData();
    bool haddleData();

    void doWorkDownloadTask(const std::string &, const std::string &, const int &rank);
    void checkoutLocalFile(QStringList &files, QString path);
    std::string divideData(const std::string &);
    std::vector<std::string> split(const std::string &str, const std::string &pattern);

    bool uploadSuppleData(QString &);

    ///////////////////////////////////////////UPLOAD
    // bool uploadDataing(const int &, QString);
    void mkDir(QString projectName = "NULL");
    std::tuple<std::string,std::string> packageNewData(const int &rank, QString path);
    void writeMapData(const QString &, const QString &);

    /////////////////////////////////////////transition to newMapTool
    //已完成数据
    const std::string cur_remote_data_dir{"/DATA_S1/"}; 
    //放弃or删除数据
    const std::string cur_remote_data_abandondir{"/DATA_S1/abandon/"};
    //补图数据存放
    const std::string cur_remote_data_suppledir{"/DATA_S1/suppled/"};
    //原数据
    const std::string cur_remote_data_olddir{"/DATA/"};

    /////////////////////////////////////////transition to newMapTool
    QList<QMap<QString, QString>> listTaskDatas_;
    QList<QMap<QString, QString>> originalData_;
    QList<QMap<QString, QString>> originalData_refersh_;
    QList<QString> projectVer_;
    QList<QString> projectName_;

    QString projectKey = "PROJECT";
    //--
    QString TASKkey = "TASK";
    QString LOCALTASKkey = "LOCALTASK";
    //--
    QString TASKtime = "TASKTIME";
    QString LOCALTASKtime = "LOCALTASKTIME";
    //--
    QString DATAsign = "DATASIGN";
    QString LOCALDATAsign = "LOCALDATASIGN";

    MainWindow *mainWindow;
    DialogWidgetTools *dialogtools_;

Q_SIGNALS:
    void sigRevMapData(bool, QList<QMap<QString, QString>>);
    void reLoadTaskSignal(int);
    void loadTaskData(QString);
    void requestLoadingMoive();
    void closeLoadingMoive();

public Q_SLOTS:
    void chooseTask(const int &);
    bool taskDataAbandon(const int &rank);
    bool taskDataComplete(const int &rank);
    bool taskDataDelet(const int &rank);
    bool taskDataModify(const int &rank);

};

#endif // TASK_MAP_H
