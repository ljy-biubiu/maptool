#ifndef UNOPTIMIZED_MAP_H
#define UNOPTIMIZED_MAP_H

#include <QObject>
#include <QFile>
#include <thread>
#include <qmap.h>
#include <QFile>
#include <QDir>
#include <QString>
#include <QMessageBox>
#include <unistd.h>
#include <sys/types.h>
#include <future>   //std::future std::promise
#include <utility>  //std::ref
#include <dirent.h> //windows开发工具没有这个头文件

const std::string homeStr = std::string(std::getenv("HOME"));

class ChooseMapUi;
#include "choosemapui.h"
class MainWindow;
#include "mainwindow.h"

enum Optimy_N : int {
  NamePOne = 0,       //名字part1
  NamePTwo,          //名字part2
  DateDay,          //包的日期
  TimeS,          //包具体时间
  Optimize_Num
};

enum Loacl_OF {
    Date_Day = 0,    //日期
    Data_TimeS,     //时间
    OF_Num
};

class Unoptimized_map : public QWidget
{
    Q_OBJECT
public:
    explicit Unoptimized_map(ChooseMapUi *chooseMapUi_, MainWindow *mainWindow_, QWidget *parent = nullptr);
    bool uploadOptimizing(const int &, QString);
    void SplitString(const std::string&, std::vector<std::string>&, const std::string &);
    const int searchProjectRank(const QString &);
    QList<QMap<QString, QString>> getOptimizeListData();
    // MainWindow* mainWindow() const;

    ChooseMapUi *chooseMapUi;

private:
    bool pullOptimizeDataList();
    bool getOptimizeProjectName();
    void readLocalData();
    bool haddleData();

    void doWorkDownloadOptimize(const std::string &, const std::string &, const int &rank);
    void checkoutLocalFile(QStringList &files, QString path);
    std::string divideData(const std::string &);
    std::vector<std::string> split(const std::string &str, const std::string &pattern);

    ///////////////////////////////////////////UPLOAD
    // bool uploadDataing(const int &, QString);
    void mkDir(QString projectName = "NULL");
    void optimizeVersionContent(const int &);
    void writeMapData(const QString &, const QString &);

    /////////////////////////////////////////transition to newMapTool
    const std::string cur_remote_optimize_dir{"/cti_data/"}; 

    /////////////////////////////////////////transition to newMapTool
    QList<QMap<QString, QString>> listOptimizedDatas_;
    QList<QMap<QString, QString>> originalData_;
    // QList<QMap<QString, QString>> originalData_refersh_;
    QList<QString> projectName_;

    QString projectKey = "PROJECT";
    //--
    QString OPTIMIZEKey = "OPTIMIZE";
    QString LOCALOPIMIZEKey = "LOCALOPTIMIZE";
    //--
    QString OPTIMIZEtime = "OPTIMIZETIME";
    QString LOCALOPTIMIZEtime = "LOCALOPTIMIZETIME";

Q_SIGNALS:
    void sigRevOptimizeData(bool, QList<QMap<QString, QString>>);
    void reLoadOptimizeSignal(int);
    void loadOptimizeData(QString);
    void loadMultipleOptimizeData(QList<QString>);
    void requestLoadingMoive();
    void closeLoadingMoive();

public Q_SLOTS:
    void chooseOptimize(const int &);
    void multipleChooseOptimize(const std::vector<int> &);
    bool optimizeDataAbandon(const int &rank);
    bool optimizeDataDelet(const int &rank);
    bool optimizeDataComplete(const int &rank);
    bool optimizeDataModify(const int &rank);
};

#endif // UNOPTIMIZED_MAP_H
