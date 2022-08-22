#include "ftp_map.h"
#include "updatelog.h"
#include "FTPClient.h"
#include "QDebug"
#include "qdir.h"
#include <QDateTime>
#include <QFileDialog>

Ftp_map::Ftp_map(Choose_map *chooseMap_, QWidget *parent) 
                    : QWidget(parent)
                    , chooseMap(chooseMap_)
{
}

bool Ftp_map::pullFtpData()
{
    const bool result_ = pullProjectName();
    if (!pullOriginalData())
        return false;
    segmenteProjectData();
    return result_;
}

///////////////////////////////////////////////////////////////////////

void Ftp_map::segmenteProjectData()
{
    qDebug() << "---------------------project data---------------------";
    listMapDatas_.clear();
    qDebug() << "---:" << originalData_.size() << " ---:" << projectName_.size();
    //将读的信息初步分割
    for (int i = 0; originalData_.size() > i; i++)
    {
        QString PROJECT = projectName_.at(i);
        qDebug() << "---:" << PROJECT;
        std::string MAPNAME = originalData_.at(i).find(projectName_.at(i)).value().at((int)Tar_N::Map).toStdString();
        std::string VMAPNAME = originalData_.at(i).find(projectName_.at(i)).value().at((int)Tar_N::Vmap).toStdString();
        std::string RMAPNAME = originalData_.at(i).find(projectName_.at(i)).value().at((int)Tar_N::RMap).toStdString();
        std::string DATANAME = originalData_.at(i).find(projectName_.at(i)).value().at((int)Tar_N::Data).toStdString();
        
        std::vector<std::string> vecMapData,vecMapVersion;
        SplitString(MAPNAME,vecMapData,"V");
        SplitString(vecMapData.back(),vecMapVersion,"_");

        std::vector<std::string> vecVmapData,vecVmapVersion;
        SplitString(VMAPNAME,vecVmapData,"V");
        SplitString(vecVmapData.back(),vecVmapVersion,"_");

        std::vector<std::string> vecRmapData,vecRmapVersion;
        SplitString(RMAPNAME,vecRmapData,"V");
        SplitString(vecRmapData.back(),vecRmapVersion,"_");

        std::vector<std::string> vecDataData,vecDataVersion;
        SplitString(DATANAME,vecDataData,"V");
        SplitString(vecDataData.back(),vecDataVersion,"_");

        QMap<QString, QString> atomMapData;

        atomMapData.insert("PROJECT", PROJECT);

        atomMapData.insert("MAP", "V" + QString::fromStdString(vecMapVersion.front()));

        atomMapData.insert("VMAP", "V" + QString::fromStdString(vecVmapVersion.front()));

        atomMapData.insert("MAPTIME", QString::fromStdString(MAPNAME.substr(MAPNAME.find('M'), 15)));

        atomMapData.insert("VMAPTIME", QString::fromStdString(VMAPNAME.substr(VMAPNAME.find('M'), 15)));
        //--
        atomMapData.insert("RMAP", "V" + QString::fromStdString(vecRmapVersion.front()));

        atomMapData.insert("DATA", "V" + QString::fromStdString(vecDataVersion.front()));

        atomMapData.insert("RMAPTIME", QString::fromStdString(RMAPNAME.substr(RMAPNAME.find('M'), 15)));

        atomMapData.insert("DATATIME", QString::fromStdString(DATANAME.substr(DATANAME.find('M'), 15)));

        atomMapData.insert("DATASIGN", QString::fromStdString(DATANAME.substr(DATANAME.find('S'), 2)));

        listMapDatas_.push_back(atomMapData);
    }
    qDebug() << "---------------------project data end---------------------";
}

bool Ftp_map::pullProjectName()
{   
    qDebug() << "enter---";
    projectName_.clear();
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");
    std::string projectPath = "/PROJECT/";

    std::vector<char> data;
    std::string map_;
    std::vector<std::string> splitDataArray;
    //int sizeSplitDataArray{0};
    const bool upload_result = FTPClient.DownloadFile(projectPath, data);

    //将读的信息初步分割  存储工程名称
    for (int i = 0; data.size() > i; i++)
    {
        if (data[i] == '\n')
        {
            projectName_.push_back(QString::fromStdString(split(divideData(map_), "Ｖ").front()));
            map_ = "";
        }
        else
        {
            map_ = map_ + data[i];
        }
    }

    //std::cout << "uploading..." << std::endl;
    if (upload_result) {
        qDebug() << "拉取FTP工程文件名成功";
    }
    FTPClient.CleanupSession();
    return upload_result;
}

bool Ftp_map::pullOriginalData()
{
    qDebug() << "enter---";
    originalData_.clear();
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    std::vector<char> data;
    std::string Data_;
    std::string dataPath;

    for (int i = 0; projectName_.size() > i; i++)
    {
        QMap<QString, QList<QString>> projectMess;
        QList<QString> mapVmapData;

        //MAP
        dataPath = "/PROJECT/" + projectName_.at(i).toStdString() + "/MAP/";
        if (!FTPClient.DownloadFile(dataPath, data))
        {
            qCritical() << "服务器缺失 " + QString::fromStdString(dataPath);
            return false;
        }
        if (data.size() == 0)
        {
            QString mapTargzFile = "NULL_V-.-.-_MNULL.tar.gz";
            mapVmapData.push_back(mapTargzFile);
        }
        else
        {
            for (int i = 0; data.size() > i; i++)
            {
                if (data[i] == '\n')
                {
                    mapVmapData.push_back(QString::fromStdString(divideData(Data_)));
                    Data_ = "";
                }
                else
                {
                    Data_ = Data_ + data[i];
                }
            }
            data.clear();
        }

        //VMAP
        dataPath = "/PROJECT/" + projectName_.at(i).toStdString() + "/VMAP/";
        if (!FTPClient.DownloadFile(dataPath, data))
        {
            qCritical() << "服务器缺失 " + QString::fromStdString(dataPath);
            return false;
        }
        //FTPClient.DownloadFile(dataPath, data);
        if (data.size() == 0)
        {
            QString vmapTargzFile = "NULL_V-.-.-_MNULL_vmap.tar.gz";
            mapVmapData.push_back(vmapTargzFile);
        }
        else
        {
            for (int i = 0; data.size() > i; i++)
            {
                if (data[i] == '\n')
                {
                    mapVmapData.push_back(QString::fromStdString(divideData(Data_)));
                    Data_ = "";
                }
                else
                {
                    Data_ = Data_ + data[i];
                }
            }
            data.clear();
        }

        //RMAP
        dataPath = "/PROJECT/" + projectName_.at(i).toStdString() + "/RMAP/";
        if (!FTPClient.DownloadFile(dataPath, data))
        {
            qCritical() << "服务器缺失 " + QString::fromStdString(dataPath);
            return false;
        }
        //FTPClient.DownloadFile(dataPath, data);
        if (data.size() == 0)
        {
            QString rmapTargzFile = "NULL_V-.-.-_MNULL_R.tar.gz";
            mapVmapData.push_back(rmapTargzFile);
        }
        else
        {
            for (int i = 0; data.size() > i; i++)
            {
                if (data[i] == '\n')
                {
                    mapVmapData.push_back(QString::fromStdString(divideData(Data_)));
                    Data_ = "";
                }
                else
                {
                    Data_ = Data_ + data[i];
                }
            }
            data.clear();
        }

        //DATA
        dataPath = "/PROJECT/" + projectName_.at(i).toStdString() + "/DATA/";
        if (!FTPClient.DownloadFile(dataPath, data))
        {
            qCritical() << "服务器缺失 " + QString::fromStdString(dataPath);
            return false;
        }
        //FTPClient.DownloadFile(dataPath, data);
        if (data.size() == 0)
        {
            QString dataTargzFile = "NULL_V-.-.-_MNULL_data_S0.tar.gz";
            mapVmapData.push_back(dataTargzFile);
        }
        else
        {
            for (int i = 0; data.size() > i; i++)
            {
                if (data[i] == '\n')
                {
                    mapVmapData.push_back(QString::fromStdString(divideData(Data_)));
                    Data_ = "";
                }
                else
                {
                    Data_ = Data_ + data[i];
                }
            }
            data.clear();
        }

        projectMess.insert(projectName_.at(i), mapVmapData);
        originalData_.push_back(projectMess);
    }

    FTPClient.CleanupSession();
    return true;
}


//空格分割字符
std::string Ftp_map::divideData(const std::string &data)
{
    std::istringstream iss(data);
    std::vector<std::string> strs{std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
    return strs.back();
}

//segmente map data by _

std::vector<std::string> Ftp_map::split(const std::string &str, const std::string &pattern)
{
    //const char* convert to char*
    char *strc = new char[std::strlen(str.c_str()) + 1];
    std::strcpy(strc, str.c_str());
    std::vector<std::string> resultVec;
    char *tmpStr = std::strtok(strc, pattern.c_str());
    while (tmpStr != NULL)
    {
        resultVec.push_back(std::string(tmpStr));
        tmpStr = std::strtok(NULL, pattern.c_str());
    }

    delete[] strc;
    return resultVec;
}
void Ftp_map::SplitString(const std::string& s, std::vector<std::string>& v, const std::string &c) {
  int length = s.length();
  if (s.size() > 0)
  {
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(std::string::npos != pos2){
      v.push_back(s.substr(pos1,pos2 - pos1));
      pos1 = pos2 + c.size();
      pos2 = s.find(c,pos1);
    }
    if(pos1 != s.length()){
      v.push_back(s.substr(pos1));
    }
  }
}

QList<QString> Ftp_map::getProjectName(){
    return projectName_;
}

QList<QMap<QString, QList<QString>>> Ftp_map::getOriginalData(){
    return originalData_;
}

QList<QMap<QString, QString>> Ftp_map::getListMapData(){
    return listMapDatas_;
}