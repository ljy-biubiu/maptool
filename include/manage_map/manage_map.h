#ifndef MANAGE_MAP_H
#define MANAGE_MAP_H

#include <QObject>
#include <QFile>
#include <thread>
#include <qmap.h>
#include <unistd.h>
#include <sys/types.h>
#include <future>   //std::future std::promise
#include <utility>  //std::ref
#include <dirent.h> //windows开发工具没有这个头文件
#include <QApplication>
#include <QMessageBox>
#include <QInputDialog>
#include <QDir>

const std::string home_str = std::string(std::getenv("HOME"));

class ChooseMapUi;
#include "choosemapui.h"
class MainWindow;
#include "mainwindow.h"
class Task_map;
#include "task_map.h"
class MapWidget;
#include "map_widget/map_widget.h"
class Ftp_map;
#include "ftp_map.h"

class Choose_map : public QWidget
{
    Q_OBJECT
public:
    explicit Choose_map(Task_map *taskMap_, ChooseMapUi *chooseMapUi_, MainWindow *mainWindow_, MapWidget *mapWidget_, QWidget *parent = nullptr);
    QList<QString> getProjectName();
    QString getMapToolLocalVer();
    QString getMapToolRemoteVer();
    //MainWindow* mainWindow() const;

    ChooseMapUi *chooseMapUi;

private:
    bool pullFtpDatas();
    bool pullProjectName();
    void segmenteProjectData();
    bool pullOriginalData();
    void readLocalData();
    void segmenteDataRankLine();
    bool haddleData();
    void initToolVer();
    void checkMapAndVmapVersion();
    void getSizeFromFile();
    void mkDir(QString projectName = "NULL");

    void doWorkDownloadMap(const std::string &, const std::vector<std::string> &, const int &rank);
    void segmenteMapData(const std::vector<char> &);
    void segmenteVMapData(const std::vector<char> &);
    void checkoutLocalFile(QStringList &files, QString path);
    QString checkMapFileExit(QString data);
    QString checkVmapFileExit(QString data);
    std::string divideData(const std::string &);
    std::vector<std::string> split(const std::string &str, const std::string &pattern);
    std::string packageNewVmap(const int &rank, QString path);

    ///////////////////////////////////////////UPLOAD
    void uploadMaping(const int &, QString, std::promise<bool> &);
    void uploadRmaping(const int &, QString, std::promise<bool> &);
    bool uploadVmaping(const int &, QString);
    const bool checkoutMemory(const int &, QString);
    const bool checkoutRmapMemory(const int &, QString);
    std::string packageNewMap(const int &, QString);
    std::string packageNewRMap(const int &, QString);
    void writeMapData(const QString &, const QString &);
    const int searchProjectRank(const QString &);
    QString getLogContent(const QString &);

    ///////////////////////////////////////////create New Project
    void createNewProject(const QString &);
    QString splitNewProjectName(const QString &);
    bool mvNewPcdToDir(const QString &, const QString &);
    void createNewVersion(const QString &);
    bool judgeAndCreateVersion(const QString &, const QString &);
    void loadNewProjectData(const QString &);
    void createNewFtpDir(const QString &);
    bool packageAndUploadNewProject(const QString &);
    void doWorkcreateNewProject(const QString &pcdDir);

    void createNewProjectFromTask(const QString &);
    void doWorkcreateNewProjectFromTask(const QString &);

    /////////////////////////////////////////
    void createMaptoolVersion();
    void getLocalToolVersion();
    bool pullToolVersionList();
    void pullMaptoolNewestDeb(QString, std::promise<bool> &);
    void writeLocalVerFile();

    //////////////////////////////////////////////////////
    void getAllDataFromFtp();
    void putProjectInfoToMysql();
    void putSingleProjectToMysql(int rank, QString projectName);
    void modifyProjectInfoToMysql();
    void modifySingleProjectToMysql(int rank, QString projectName);
    void getProjectInfoFromMysql();
    void updateAndAddProject(QString project, MapData &);
    void modifyProject(int rank, QString projectName, MapData &);
    void initNewMapData(QString project, QString version, MapData &);
    void modifySingleFtpToMysql(int rank, QString projectName);

    /////////////////////////////////////////////////////
    const std::string cur_remote_map3_dir{"/map3/"};
    const std::string cur_remote_rmap_dir{"/RMAP/"};
    const std::string cur_remote_vmap3_dir{"/vmap/"};

    std::string remote_project_delete{"/PROJECT-DELET/"};

    /////////////////////////////////////////transition to newMapTool

    std::vector<std::string> mapMsgArrayTable;
    std::vector<std::string> vmapMsgArrayTable;
    QStringList mapFileExit;
    QStringList vmapFileExit;
    std::vector<char> mapData;
    std::vector<char> vmapData;
    QList<QMap<QString, QString>> listMapDatas;
    QMap<QString, QString> listToolDatas_;
    std::vector<MapData> vec_mapdatas_;
    QMap<QString, QString> newPackageNames_;
    QMap<QString, QString> newPackageVersions_;

    QList<QString> projectName_;
    QList<QMap<QString, QList<QString>>> originalData;
    QList<QMap<QString, QList<QString>>> localData;
    // std::string maptoolVerDeb_;
    QString maptoolLocalVer_;
    //embeddedmz::CFTPClient* FTPClient;
    //--
    QList<QString> projectFtpName_;
    QList<QMap<QString, QString>> listFtpMapDatas_;
    QList<QMap<QString, QList<QString>>> originalFtpData_;

    QString projectKey = "PROJECT";
    //--
    QString MAPkey = "MAP";
    QString VMAPkey = "VMAP";
    QString LOCALVMAPkey = "LOCALVMAP";
    QString LOCALMAPkey = "LOCALMAP";
    //--
    QString COMPAREVERTIONkey = "COMPAREVERTION";
    QString MEMORYkey = "MEMORY";
    //--
    QString VMAPtime = "VMAPTIME";
    QString MAPtime = "MAPTIME";
    QString LOCALVMAPtime = "LOCALVMAPTIME";
    QString LOCALMAPtime = "LOCALMAPTIME";
    //--
    QString RMAPkey = "RMAP";
    QString DATAkey = "DATA";
    QString LOACLRMAPkey = "LOACLRMAP";
    QString LOCALDATAkey = "LOCALDATA";
    //--
    QString RMEMORYkey = "RMEMORY";
    //--
    QString RMAPtime = "RMAPTIME";
    QString DATAtime = "DATATIME";
    QString LOACLRMAPtime = "LOACLRMAPTIME";
    QString LOCALDATAtime = "LOACLDATATIME";
    //--
    QString DATAsign = "DATASIGN";
    QString LOCALDATAsign = "LOACLDATASIGN";

    //--toolversion
    QString TOOLver = "TOOLVERSION";
    QString LOCALtoolver = "LOCALTOOLVER";
    QString PACKAGES = "PACKAGE";

    //--packageInfo
    QString MAPstr = "MAPSTR";
    QString RMAPstr = "RMAPSTR";
    QString VMAPstr = "VMAPSTR";
    QString DATAstr = "DATASTR";


    MainWindow *mainWindow;
    Task_map *taskMap;
    MapWidget *mapWidget;
    Ftp_map *ftpMap_;

Q_SIGNALS:
    void sigRevMapData(bool, QList<QMap<QString, QString>>);
    void reLoadMapSignal(int);
    void loadMapAndVmap(QString);
    void requestLogDialog();
    void requestLoadingMoive();
    void closeLoadingMoive();
    void refreshMapUi();
    void uploadDocking(QString);
    void initFtpData();

public Q_SLOTS:
    void chooseMap(const int &);
    void getPcdFile(QString);
    void uploadMapAndVmap(QString);
    void refreshVmapMapTime();
    void versionUpdate();
    bool mapDataDelete(const int &);
    void deleteMysqlProjectConfirm(QString);
    void modifyMysqlProjectConfirm();
    void createMysqlProjectConfirm();
};

#endif // CHOOSE_MAP_H
