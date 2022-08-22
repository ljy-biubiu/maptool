#include "unoptimized_map.h"
#include "updatelog.h"
#include "FTPClient.h"
#include "QDebug"
#include "qdir.h"
#include <QDateTime>
#include <QFileDialog>


Unoptimized_map::Unoptimized_map(ChooseMapUi *chooseMapUi_, MainWindow *mainWindow_, QWidget *parent) : QWidget(parent), chooseMapUi(chooseMapUi_)
{
    connect(this, SIGNAL(sigRevOptimizeData(bool, QList<QMap<QString, QString>>)),
            chooseMapUi, SLOT(getOptimizeDataList(bool, QList<QMap<QString, QString>>)));

    connect(chooseMapUi, SIGNAL(sendOptimizeButtonArrayData(int)), this, SLOT(chooseOptimize(int)));

    connect(chooseMapUi, SIGNAL(sendMultipleOptimizeButtonData(std::vector<int>)), this, SLOT(multipleChooseOptimize(std::vector<int>)));

    connect(chooseMapUi, SIGNAL(sendOptimizeButtonAbandon(int)), this, SLOT(optimizeDataAbandon(int)));

    connect(chooseMapUi, SIGNAL(sendOptimizeButtonDelet(int)), this, SLOT(optimizeDataDelet(int)));

    connect(chooseMapUi, SIGNAL(sendOptimizeButtonComplete(int)), this, SLOT(optimizeDataComplete(int)));

    connect(chooseMapUi, SIGNAL(sendOptimizeButtonModify(int)), this, SLOT(optimizeDataModify(int)));

    connect(this, SIGNAL(reLoadOptimizeSignal(int)), chooseMapUi, SLOT(reLoadOptimizeUiSlot(int)));

    connect(this, SIGNAL(requestLoadingMoive()), chooseMapUi, SLOT(loadingDatas()));

    connect(this, SIGNAL(closeLoadingMoive()), chooseMapUi, SLOT(EndloadingDatas()));

    if (haddleData())
    {
        Q_EMIT sigRevOptimizeData(true, listOptimizedDatas_);  
        qDebug() << "初始化optimize数据正常";
    }
    else
    {
        Q_EMIT sigRevOptimizeData(false, listOptimizedDatas_);
        qCritical() << "optimize无初始化数据";
    }
}

bool Unoptimized_map::haddleData()
{
    const bool result_ = pullOptimizeDataList();
    if (!getOptimizeProjectName())
        return false;
    mkDir();
    readLocalData();
    return result_;
}

///////////////////////////////////////////////////////////////////////

//create new project
///////////////////////////////////////////////////////////////////////
void Unoptimized_map::mkDir(QString projectName)
{
    std::string DIR = homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE";
    QString qdir = QString::fromStdString(DIR);
    QString initTime = "0";
    QDir dir(qdir);
    if(!dir.exists())
    {
        qDebug() << "路径不存在.";
        // return false;
    }
    //不存在当前目录，创建，可创建多级目录 //mkdir只能创建一级目录
    // bool ok = dir.mkpath(path_str);
    if (dir.mkpath(qdir))
    {
        qDebug() << "创建CTI_MAP_OPTIMIZE文件夹成功";
    } else {
        qCritical() << "创建CTI_MAP_OPTIMIZE失败";
    }

    // qDebug() << "------------" << projectName_.size();
    for (int i = 0; projectName_.size() > i; i++)
    {   
        // qDebug() << "tasking: " << projectName_.at(i);
        DIR = homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE/" + projectName_.at(i).toStdString();
        if (access(DIR.c_str(), 0))
        {
            qDebug() << "do create: " << projectName_.at(i);
            std::string cmd_str = "cd ~/maptool-candela/CTI_MAP_OPTIMIZE && mkdir " + 
                                  projectName_.at(i).toStdString() + "&& cd " + 
                                  projectName_.at(i).toStdString() + " && mkdir DATA && cd DATA && touch version";
            if (std::system(cmd_str.c_str()))
            {
                qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
            }
            else
            {
                qDebug() << "创建" + projectName_.at(i) + " data文件夹成功";
            }
            QString dataVersionPath = QString::fromStdString(DIR) + "/DATA/version";
            QString versionContent = initTime + "\n" + initTime;
            writeMapData(dataVersionPath, versionContent);
        }
    }

    if (projectName != "NULL")
    {
        DIR = homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE/" + projectName.toStdString();
        if (access(DIR.c_str(), 0))
        {
            std::string cmd_str = "cd ~/maptool-candela/CTI_MAP_OPTIMIZE && mkdir " + 
                                  projectName.toStdString() + " && cd " + 
                                  projectName.toStdString() + " && mkdir DATA && cd DATA && mkdir lslam-map && touch version";
            if (std::system(cmd_str.c_str()))
            {
                qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
            }
            else
            {
                qDebug() << "创建 " + projectName + " data文件夹成功";
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////// 下拉

bool Unoptimized_map::pullOptimizeDataList()
{
    originalData_.clear();
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020");
    std::string projectPath = "/cti_data/";

    std::vector<char> data;
    std::string map;
    //int sizeSplitDataArray{0};
    const bool upload_result = FTPClient.DownloadFile(projectPath, data);
    
    //将读的信息存储
    int index = 0;
    // std::cout << "num: " << data.size() << std::endl;
    for (int i = 0; data.size() > i; i++)
    {   
        std::vector<std::string> mapmsgs,bagnames;
        QMap<QString, QString> origin;
        if (data[i] == '\n')
        {
            QString optimyinfo = QString::fromStdString(divideData(map));
            //--
            SplitString(optimyinfo.toStdString(),mapmsgs,"-");
            std::string bagname = mapmsgs.front() + mapmsgs[1];
            // SplitString(bagname,bagnames,"-");
            QString optimyname = QString::fromStdString(bagname) + "-" + QString::number(index);
            //--
            origin.insert(optimyname,optimyinfo);
            // qDebug() << "optiminfo: " << optimyinfo;
            // qDebug() << "optimyname: " << optimyname;
            map = "";
            index = index+1;
            originalData_.push_back(origin);
        }
        else
        {
            map = map + data[i];
        }
    }

    //std::cout << "uploading..." << std::endl;
    if (upload_result)
    {
        qDebug() << "拉取ＦＴＰ工程文件名成功";
    }
    else
    {
        qCritical() << "拉取ＦＴＰ工程文件名失败";
    }
    FTPClient.CleanupSession();
    return upload_result;
}

bool Unoptimized_map::getOptimizeProjectName()
{
    projectName_.clear();
    listOptimizedDatas_.clear();
    std::vector<char> data;
    
    // QMap<QString,QString>::iterator it;
    // for ( it = originalData_.begin(); it != originalData_.end(); ++it ) 
    // std::cout << "num: " << originalData_.size() << std::endl;
    for ( int i{0}; i < originalData_.size(); i++ ) 
    {
        // qDebug() << "111111111111";
        QMap<QString, QString> origin = originalData_.at(i);
        QMap<QString, QString>::iterator it = origin.begin();
        std::string TASKPACKNAME = it.value().toStdString();
        //name
        QString projectName = it.key();
        // qDebug() << "taskprojectname = " << projectName;

        //info
        std::vector<std::string> origins,origin_signs;
        SplitString(TASKPACKNAME,origins,"-");

        QMap<QString, QString> taskData;

        taskData.insert("PROJECT", projectName);

        taskData.insert("OPTIMIZE", QString::fromStdString(origins.at(Optimy_N::DateDay)));

        taskData.insert("OPTIMIZETIME", QString::fromStdString(origins.at(Optimy_N::TimeS)));

        projectName_.push_back(projectName);
        listOptimizedDatas_.push_back(taskData);
    }
    if (listOptimizedDatas_.size() == 0)
    {
        return false;
    }
    return true;
}

//读取optimize的version
void Unoptimized_map::readLocalData()
{
    // qDebug() << "---------------------readlocal data---------------------";
    std::cout << "num: " << projectName_.size() << std::endl;
    for (int i{0}; projectName_.size() > i; i++)
    {
        // qDebug() << "---------------------i = " << i;
        QString projectName = projectName_.at(i);
        // qDebug() << "---------------------name = " << projectName;
        QString pathOptimizeVer = QString::fromStdString(home_str_) + "/maptool-candela/CTI_MAP_OPTIMIZE/" + projectName + "/DATA/" + "/version";

        QFile file(pathOptimizeVer);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            listOptimizedDatas_[i].insert("LOCALOPTIMIZE", "NULL");
        }
        else
        {
            QByteArray t = file.readAll();
            listOptimizedDatas_[i].insert("LOCALOPTIMIZETIME", QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_OF::Data_TimeS)));
            listOptimizedDatas_[i].insert("LOCALOPTIMIZE", QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_OF::Date_Day)));
        }
        file.close();
    }
    // qDebug() << "---------------------readlocal data end---------------------";
}
///////////////////////////////////////////////////////////////////////上传

bool Unoptimized_map::uploadOptimizing(const int &rank, QString path)
{
    optimizeVersionContent(rank);
    std::string remoteDataOriginalName = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::string localDataPackagePath = path.toStdString() + "/DATA/" + remoteDataOriginalName;
    std::string remoteDataPackagePath = "/cti_data_compelet/" + remoteDataOriginalName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020");

    const bool ifuploadfile = FTPClient.UploadFile(localDataPackagePath, remoteDataPackagePath);
    //上传到cti_data_compelet文件
    if (!ifuploadfile)
    {
        qCritical() << "上传 " + projectName_.at(rank) + " 文件失败";
    }
    else
    {
        qDebug() << "上传 " + projectName_.at(rank) + " 文件成功";
    }

    ///////////////////////////////////////////////////////////////////////临时使用 过度
    //删除cti_data内文件
    std::string cur_remoteDataPackageOldPath = cur_remote_optimize_dir + remoteDataOriginalName;
    const bool ifdeleteFile = FTPClient.RemoveFile(cur_remoteDataPackageOldPath);
    if (!ifdeleteFile)
    {
        qCritical() << "删除cti_data内 " << projectName_.at(rank) << " 文件失败";
    }
    else
    {
        qDebug() << "删除cti_data文件夹内lslam-map文件成功";
    }

    //////////////////////////////////////////////////////////////////////
    FTPClient.CleanupSession();
    //判断上传和移除
    return ifuploadfile && ifdeleteFile;
}

void Unoptimized_map::optimizeVersionContent(const int &rank)
{
    QString package_data = listOptimizedDatas_.at(rank).find("OPTIMIZE").value();
    QString package_time = listOptimizedDatas_.at(rank).find("OPTIMIZETIME").value();
    QString project = listOptimizedDatas_.at(rank).find(projectKey).value();

    QString dataVersionFile = QString::fromStdString(homeStr) + "/maptool-candela/CTI_MAP_OPTIMIZE/" + project + "/DATA/version";

    QString versionContent = package_data + "\n" + package_time;

    //写入文件
    writeMapData(dataVersionFile, versionContent);
}

const int Unoptimized_map::searchProjectRank(const QString &project)
{
    for (int i = 0; listOptimizedDatas_.size() > i; i++)
    {
        if (listOptimizedDatas_.at(i).find("PROJECT").value() == project)
        {
            return i;
        }
    }
    return -1;
}

void Unoptimized_map::writeMapData(const QString &path, const QString &content)
{
    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    file.write(content.toLatin1().data());
    file.close();
    qDebug() << "写入lslam-map Log文件: " << content;
}
//////////////////////////////////////////////////////////////////////////////选择卡片

void Unoptimized_map::chooseOptimize(const int &rank)
{
    qDebug() << "choose " << rank << "success";
    qDebug() << "list num: " <<  listOptimizedDatas_.size();
    if (listOptimizedDatas_.at(rank).find("OPTIMIZE").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZE").value() && 
        listOptimizedDatas_.at(rank).find("OPTIMIZETIME").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZETIME").value())
    {
        qDebug() << "enter load optimize";
        Q_EMIT loadOptimizeData(listOptimizedDatas_.at(rank).find("PROJECT").value());
        return;
    }
    std::vector<std::string> remoteFileName_type;
    std::string remoteProject = projectName_.at(rank).toStdString();
    std::string remoteFileInfo = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::cout << "downloading " << remoteProject << "... " << remoteFileInfo << "... " << rank << "..." << std::endl;
    // std::thread thDownloadMap(&Choose_map::doWorkDownloadMap, this, remoteProject, remoteMap, remoteVmap, rank);
    std::thread thDownloadMap(&Unoptimized_map::doWorkDownloadOptimize, this, remoteProject, remoteFileInfo, rank);
    thDownloadMap.detach();
    Q_EMIT requestLoadingMoive();
    //    qDebug()<<listMapDatas[rank][1];
}

void Unoptimized_map::multipleChooseOptimize(const std::vector<int> &listMultiple)
{
    qDebug() << "list num: " <<  listOptimizedDatas_.size();
    QList<QString> listProject;
    for (size_t i = 0; i < listMultiple.size(); i++)
    {
        int rank = listMultiple[i];
        //判断项目版本
        if (listOptimizedDatas_.at(rank).find("OPTIMIZE").value() != listOptimizedDatas_.at(rank).find("LOCALOPTIMIZE").value() && 
            listOptimizedDatas_.at(rank).find("OPTIMIZETIME").value() != listOptimizedDatas_.at(rank).find("LOCALOPTIMIZETIME").value())
        {
            std::vector<std::string> remoteFileName_type;
            std::string remoteProject = projectName_.at(rank).toStdString();
            std::string remoteFileInfo = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
            std::cout << "downloading " << remoteProject << "... " << remoteFileInfo << "... " << rank << "..." << std::endl;
            // std::thread thDownloadMap(&Choose_map::doWorkDownloadMap, this, remoteProject, remoteMap, remoteVmap, rank);
            std::thread thDownloadMap(&Unoptimized_map::doWorkDownloadOptimize, this, remoteProject, remoteFileInfo, rank);
            thDownloadMap.detach();
            Q_EMIT requestLoadingMoive();
        }
        listProject.push_back(listOptimizedDatas_.at(rank).find("PROJECT").value());
    }
    qDebug() << "enter load multiple optimize";
    Q_EMIT loadMultipleOptimizeData(listProject);
}

void Unoptimized_map::checkoutLocalFile(QStringList &files, QString path)
{
    QDir dir(path);
    QStringList nameFilters;
    nameFilters << "*.tar.gz";
    files = dir.entryList(nameFilters);
}

void Unoptimized_map::doWorkDownloadOptimize(const std::string &PATH_, const std::string &remoteFileInfo, const int &rank)
{
    std::string dataName = divideData(remoteFileInfo);

    //--
    qDebug() << "------: " << QString::fromStdString(PATH_) << "... " << QString::fromStdString(remoteFileInfo) << "... "  << rank << "...";
    mkDir(QString::fromStdString(PATH_));

    std::string localDataDir = homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE/" + PATH_ + "/DATA/";
    std::string localDataPath = localDataDir + dataName;

    std::string remoteDataDir = "/cti_data/";
    std::string remoteDataPath = remoteDataDir + dataName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });

    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020");

    //--
    const bool upload_result_data = FTPClient.DownloadFile(localDataPath, remoteDataPath);
    std::cout << "uploading..." << upload_result_data <<  std::endl;

    if (upload_result_data)
    {
        qDebug() << "download " + QString::fromStdString(dataName) + " success";
        std::string cmd_str = "cd " + localDataDir + " && unzip -o " + localDataPath;

        if (std::system(cmd_str.c_str()))
        {
            qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
        }
        else
        {
            qDebug() << "解压 " + QString::fromStdString(localDataPath) + " 文件夹成功";
        }
    }
    else
    {
        qCritical() << "download " + QString::fromStdString(dataName) + " fault";
    }

    FTPClient.CleanupSession();

    if (upload_result_data)
    {
        listOptimizedDatas_[rank][LOCALOPIMIZEKey] = listOptimizedDatas_.at(rank).find(OPTIMIZEKey).value();
        listOptimizedDatas_[rank][LOCALOPTIMIZEtime] = listOptimizedDatas_.at(rank).find(OPTIMIZEtime).value();

        QString currentDate = listOptimizedDatas_.at(rank).find(OPTIMIZEKey).value();
        QString currentTime = listOptimizedDatas_.at(rank).find(OPTIMIZEtime).value();
        QString dataVersionPath = QString::fromStdString(localDataDir) + "version";
        QString versionContent = currentDate + "\n" + currentTime;
        writeMapData(dataVersionPath, versionContent);
        Q_EMIT reLoadOptimizeSignal(rank);
    }

    Q_EMIT closeLoadingMoive();
}
///////////////////////////////////////////////////////////////////tool

//空格分割字符
std::string Unoptimized_map::divideData(const std::string &data)
{
    std::istringstream iss(data);
    std::vector<std::string> strs{std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
    return strs.back();
}
void Unoptimized_map::SplitString(const std::string& s, std::vector<std::string>& v, const std::string &c) {
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
//segmente map data by _
std::vector<std::string> Unoptimized_map::split(const std::string &str, const std::string &pattern)
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

QList<QMap<QString, QString>> Unoptimized_map::getOptimizeListData() {
    return listOptimizedDatas_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////任务管理
//放弃待优化任务
bool Unoptimized_map::optimizeDataAbandon(const int &rank) {
    qDebug() << "进入待优化任务放弃。";
    std::string projectname = projectName_.at(rank).toStdString();
    std::string path = homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE/" + projectname + "/DATA/";
    std::string remoteDataOptimizeName = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::string remoteDataPackagePath = "/cti_data_abandon/" + remoteDataOptimizeName;
    std::string cur_remoteDataPackageOldPath = cur_remote_optimize_dir + remoteDataOptimizeName;
    //本地进行移除
    std::string localDataPackagePath = path + remoteDataOptimizeName;
    std::string cmd_str = "cd " + path + " && rm -rf " + remoteDataOptimizeName;
    //确保本地数据存在，否则不可备份数据
    if (listOptimizedDatas_.at(rank).find("OPTIMIZE").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZE").value() && 
        listOptimizedDatas_.at(rank).find("OPTIMIZETIME").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZETIME").value())
    {
        qDebug() << "本地数据存在且版本相同";
        int reply = QMessageBox::question(this, tr("是否放弃此任务？"), tr("No：返回,  Yes：放弃."),
                                QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
                FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020");
                //ftp上进行上传备份
                const bool ifuploadfile = FTPClient.UploadFile(localDataPackagePath, remoteDataPackagePath);
                //ftp上移除数据
                const bool ifdeleteFile = FTPClient.RemoveFile(cur_remoteDataPackageOldPath);
                FTPClient.CleanupSession();
                //本地删除数据
                const bool iflocaldeleteFile = std::system(cmd_str.c_str());
                return ifuploadfile && ifdeleteFile;
            }
    }
    else
    {
        QMessageBox::question(this, tr("警告！"), tr("本地数据不存在/版本不同，不可放弃，请先确保本地数据与远端版本一致。"),
                                QMessageBox::Yes);
        return false;
    }
}
//删除待优化任务
bool Unoptimized_map::optimizeDataDelet(const int &rank) {
    qDebug() << "进入待优化任务删除。";
    std::string projectname = projectName_.at(rank).toStdString();
    std::string path = homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE/" + projectname + "/DATA/";
    std::string remoteDataOptimizeName = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::string remoteDataPackagePath = "/cti_data_abandon/" + remoteDataOptimizeName;
    std::string cur_remoteDataPackageOldPath = cur_remote_optimize_dir + remoteDataOptimizeName;
    //本地删除整个项目
    std::string localDataPackagePath = path + remoteDataOptimizeName;
    std::string cmd_str = "cd " + homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE" + " && rm -rf " + projectname;
    //确保本地数据存在，否则不可备份数据
    if (listOptimizedDatas_.at(rank).find("OPTIMIZE").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZE").value() && 
        listOptimizedDatas_.at(rank).find("OPTIMIZETIME").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZETIME").value())
    {
        qDebug() << "本地数据存在且版本相同";
        int reply = QMessageBox::question(this, tr("是否删除此任务？"), tr("No：返回,  Yes：删除."),
                                QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                //输入密码才可以删除
                bool isOK;
                QString text = QInputDialog::getText(NULL, "Input Dialog",
                                                   "Please input password",
                                                   QLineEdit::Password,
                                                   "",
                                                   &isOK);
                if (isOK)
                {
                    if (text != "MaptoolDelet")
                    {
                        QMessageBox::information(this, tr("提示"), tr("密码输入错误！不可删除该项目！"),
                                QMessageBox::Ok);
                        return false;
                    }
                }
                
                embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
                FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020");
                //ftp上进行上传备份
                const bool ifuploadfile = FTPClient.UploadFile(localDataPackagePath, remoteDataPackagePath);
                //ftp上移除数据
                const bool ifdeleteFile = FTPClient.RemoveFile(cur_remoteDataPackageOldPath);
                FTPClient.CleanupSession();
                //本地删除数据
                const bool iflocaldeleteFile = std::system(cmd_str.c_str());
                return ifuploadfile && ifdeleteFile;
            }
    }
    else
    {
        QMessageBox::question(this, tr("警告！"), tr("本地数据不存在/版本不同，不可放弃，请先确保本地数据与远端版本一致。"),
                                QMessageBox::Yes);
        return false;
    }
}
//完成待优化任务
bool Unoptimized_map::optimizeDataComplete(const int &rank) {
    qDebug() << "进入待优化任务完成。";
    std::string projectname = projectName_.at(rank).toStdString();
    std::string path = homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE/" + projectname + "/DATA/";
    std::string remoteDataOptimizeName = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::string remoteDataPackagePath = "/cti_data_compelet/" + remoteDataOptimizeName;
    std::string cur_remoteDataPackageOldPath = cur_remote_optimize_dir + remoteDataOptimizeName;
    //本地进行移除
    std::string localDataPackagePath = path + remoteDataOptimizeName;
    std::string cmd_str = "cd " + homeStr + "/maptool-candela/CTI_MAP_OPTIMIZE/" + " && rm -rf " + projectname;
    //确保本地数据存在，否则不可备份数据
    if (listOptimizedDatas_.at(rank).find("OPTIMIZE").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZE").value() && 
        listOptimizedDatas_.at(rank).find("OPTIMIZETIME").value() == listOptimizedDatas_.at(rank).find("LOCALOPTIMIZETIME").value())
    {
        qDebug() << "本地数据存在且版本相同";
        int reply = QMessageBox::question(this, tr("是否完成此任务？"), tr("No：返回,  Yes：完成."),
                                QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
                FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp001", "Ftp001@2020");
                //ftp上进行上传备份
                const bool ifuploadfile = FTPClient.UploadFile(localDataPackagePath, remoteDataPackagePath);
                //ftp上移除数据
                const bool ifdeleteFile = FTPClient.RemoveFile(cur_remoteDataPackageOldPath);
                FTPClient.CleanupSession();
                //本地删除数据
                const bool iflocaldeleteFile = std::system(cmd_str.c_str());
                return ifuploadfile && ifdeleteFile;
            }
    }
    else
    {
        QMessageBox::question(this, tr("警告！"), tr("本地数据不存在/版本不同，不可完成，请先确保本地数据与远端版本一致。"),
                                QMessageBox::Yes);
        return false;
    }
}
//修改待优化任务
bool Unoptimized_map::optimizeDataModify(const int &rank) {
    
}