#ifndef FTP_MAP_H
#define FTP_MAP_H

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

// const std::string home_str = std::string(std::getenv("HOME"));

class Choose_map;
#include "manage_map.h"
class MainWindow;
#include "mainwindow.h"

class Ftp_map : public QWidget
{
    Q_OBJECT
public:
    explicit Ftp_map(Choose_map *chooseMap_, QWidget *parent = nullptr);
    QList<QString> getProjectName();
    QList<QMap<QString, QList<QString>>> getOriginalData();
    QList<QMap<QString, QString>> getListMapData();

private:
    bool pullProjectName();
    void segmenteProjectData();
    bool pullOriginalData();

    std::string divideData(const std::string &);
    std::vector<std::string> split(const std::string &str, const std::string &pattern);
    void SplitString(const std::string& s, std::vector<std::string>& v, const std::string &c);

    QList<QMap<QString, QString>> listMapDatas_;

    QList<QString> projectName_;
    QList<QMap<QString, QList<QString>>> originalData_;

    QString projectKey = "PROJECT";
    //--
    QString MAPkey = "MAP";
    QString VMAPkey = "VMAP";

    //--
    QString VMAPtime = "VMAPTIME";
    QString MAPtime = "MAPTIME";

    //--
    QString RMAPkey = "RMAP";
    QString DATAkey = "DATA";

    //--
    QString RMAPtime = "RMAPTIME";
    QString DATAtime = "DATATIME";
 
    //--
    QString DATAsign = "DATASIGN";

    Choose_map *chooseMap;
    MainWindow *mainWindow;

public Q_SLOTS:
    bool pullFtpData();
};

#endif // FTP_MAP_H
