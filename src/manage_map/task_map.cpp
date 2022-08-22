#include "task_map.h"
#include "updatelog.h"
#include "FTPClient.h"
#include "QDebug"
#include "qdir.h"
#include <QDateTime>
#include <QFileDialog>


Task_map::Task_map(ChooseMapUi *chooseMapUi_, MainWindow *mainWindow_, QWidget *parent) : QWidget(parent), mainWindow(mainWindow_), chooseMapUi(chooseMapUi_)
{
    connect(this, SIGNAL(sigRevMapData(bool, QList<QMap<QString, QString>>)),
            chooseMapUi, SLOT(getTaskDataList(bool, QList<QMap<QString, QString>>)));

    connect(chooseMapUi, SIGNAL(sendTaskButtonArrayData(int)), this, SLOT(chooseTask(int)));

    connect(chooseMapUi, SIGNAL(sendTaskButtonAbandon(int)), this, SLOT(taskDataAbandon(int)));

    connect(chooseMapUi, SIGNAL(sendTaskButtonDelet(int)), this, SLOT(taskDataDelet(int)));

    connect(chooseMapUi, SIGNAL(sendTaskButtonComplete(int)), this, SLOT(taskDataComplete(int)));

    connect(chooseMapUi, SIGNAL(sendTaskButtonModify(int)), this, SLOT(taskDataModify(int)));

    connect(this, SIGNAL(reLoadTaskSignal(int)), chooseMapUi, SLOT(reLoadTaskUiSlot(int)));

    connect(this, SIGNAL(requestLoadingMoive()), chooseMapUi, SLOT(loadingDatas()));

    connect(this, SIGNAL(closeLoadingMoive()), chooseMapUi, SLOT(EndloadingDatas()));

    if (haddleData())
    {
        Q_EMIT sigRevMapData(true, listTaskDatas_);  
        qDebug() << "初始化taskmap数据正常";
    }
    else
    {
        Q_EMIT sigRevMapData(false, listTaskDatas_);
        qCritical() << "taskmap无初始化数据";
    }
}

bool Task_map::haddleData()
{
    const bool result_ = pullTaskDataList();
    if (!getTaskProjectName())
        return false;
    readLocalData();
    return result_;
}

///////////////////////////////////////////////////////////////////////

//create new project
///////////////////////////////////////////////////////////////////////
void Task_map::mkDir(QString projectName)
{
    if (projectName != "NULL")
    {
        std::string DIR = home_str_ + "/maptool-candela/CTI_MAP_PROJECT/" + projectName.toStdString();
        if (access(DIR.c_str(), 0))
        {
            std::string cmd_str = "cd ~/maptool-candela/CTI_MAP_PROJECT && mkdir " + 
                                  projectName.toStdString() + " && cd " + 
                                  projectName.toStdString() + " && mkdir MAP && mkdir VMAP && mkdir RMAP && mkdir DATA && cd MAP && mkdir " + 
                                  projectName.toStdString() + " && cd .. && cd VMAP && mkdir " + projectName.toStdString();
            if (std::system(cmd_str.c_str()))
            {
                qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
            }
            else
            {
                qDebug() << "创建 " + projectName + " MAP/VMAP文件夹成功";
                std::string cmd_str2 = "cd ~/maptool-candela/CTI_MAP_PROJECT/" + projectName.toStdString() + "/RMAP && mkdir " + projectName.toStdString() +
                                       " && cd .. && cd DATA && mkdir " + projectName.toStdString();
                if (std::system(cmd_str2.c_str()))
                {
                    qCritical() << "执行 " + QString::fromStdString(cmd_str2) + " 失败";
                }
                else 
                {
                    qDebug() << "创建 " + projectName + " RMAP/DATA文件夹成功";
                }
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////

void Task_map::writeMapData(const QString &path, const QString &content)
{
    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    file.write(content.toLatin1().data());
    file.close();
    qDebug() << "写入新版DATA Log文件: " << content;
}

bool Task_map::uploadSuppleData(QString &project) {
  //上传优化后的补图文件数据
    QString suppleDir;
    QString suffix = "*.tar.gz";
    QString localSuppleDataDir = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + project + "/DATA/" + project;
    QStringList tarDirs;
    std::string localSuppleDataPath;
    std::string remoteSuppleDataPath;
    bool isHave = dialogtools_->getFileName(localSuppleDataDir,suffix,tarDirs);

    //上传
    if (isHave && tarDirs.size() != 0)
    {
        embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
        FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

        for (size_t i = 0; i < tarDirs.size(); i++)
        {
            suppleDir = tarDirs.at(i);
            qDebug() << "优化后补图数据名称：" << tarDirs.at(i);
        
            localSuppleDataPath = localSuppleDataDir.toStdString() + "/" + suppleDir.toStdString();
            remoteSuppleDataPath = cur_remote_data_suppledir + suppleDir.toStdString();
            std::cout << "本地路径： " << localSuppleDataPath << "  服务器路径： " << remoteSuppleDataPath;

            const bool ifuploadfile = FTPClient.UploadFile(localSuppleDataPath, remoteSuppleDataPath);

            if (ifuploadfile)
            {
                qDebug() << "优化后补图数据上传成功.";
                std::string cmd_str = "rm -rf " + localSuppleDataPath;
                //删除本地
                if (std::system(cmd_str.c_str()))
                    qCritical() << QString::fromStdString(cmd_str) << "　失败";
            }
            else {
                qDebug() << "优化后补图数据上传失败.";
            }
        }
        FTPClient.CleanupSession();
    }
    else { return false; }
}
std::tuple<bool,std::string,std::string> Task_map::uploadDataing(const int &rank, QString path)
{
    std::string localDataPackageName;
    std::string newVersionStr;
    std::tie(localDataPackageName, newVersionStr) = packageNewData(rank, path);
    std::string localDataPackagePath = path.toStdString() + "/DATA/" + localDataPackageName;
    std::string remoteDataPackageDir = "/PROJECT/" + projectName_.at(rank).toStdString() + "/DATA/";
    std::string remoteDataPackagePath = remoteDataPackageDir + localDataPackageName;
    std::string remoteDataOriginalName = originalData_refersh_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::string remoteDataOriginalPath = remoteDataPackageDir + remoteDataOriginalName;

    //关键帧数据名
    QString dataVersion = listTaskDatas_.at(rank).find(TASKkey).value();
    std::string keyframsVer = split(dataVersion.toStdString(), "V").back();
    std::vector<std::string> vec_data = split(dataVersion.toStdString(), ".");
    int numbFirst = std::stoi(vec_data[2]);
    int numbSecond = std::stoi(vec_data[1]);
    int numbThird = std::stoi(split(vec_data[0], "V").back());
    if (numbThird > 1)
    {
        numbThird = numbThird-1;
    }
    QString oldFramesVer = QString::number(numbThird) + "." + QString::number(numbSecond) + "." + QString::number(numbFirst);
    std::string localFramesSavePath = path.toStdString() + "/DATA/" + projectName_.at(rank).toStdString() + "/" + keyframsVer;
    
    //--
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    //删除旧的数据包
    pullProjectListName(rank);
    for (size_t i = 0; i < projectVer_.size(); i++)
    {
        std::string remoteOldPackageDir = remoteDataPackageDir + projectVer_[i].toStdString();
        bool deleteOldFlag = FTPClient.RemoveFile(remoteOldPackageDir);
        if (deleteOldFlag)
        {
            //删除project S1
            qDebug() << "删除 " + projectName_.at(rank) + " 旧文件成功";
        } 
    }
    
    const bool ifuploadfile = FTPClient.UploadFile(localDataPackagePath, remoteDataPackagePath);
    bool ifdeletefile_falg;
    //data数据不会为null
    if (remoteDataOriginalName != "NULL_V-.-.-_MNULL_data_S0.tar.gz")
    {
        //移除的是project S0
        ifdeletefile_falg = FTPClient.RemoveFile(remoteDataOriginalPath);
    } else {
        ifdeletefile_falg = true;
    }
    const bool ifdeleteFile = ifdeletefile_falg;

    if (!ifuploadfile)
    {
        qCritical() << "上传 " + projectName_.at(rank) + " DATA文件失败";
    }
    else
    {
        qDebug() << "上传 " + projectName_.at(rank) + " DATA文件成功";
    }
    if (!ifdeleteFile)
    {
        qCritical() << "删除服务器 " + projectName_.at(rank) + " DATA文件失败";
    }
    else
    {
        qDebug() << "删除服务器 " + projectName_.at(rank) + " DATA文件成功";
    }

    ///////////////////////////////////////////////////////////////////////临时使用 过度 上传到data_s1
    std::string cur_remoteDataPackagePath = cur_remote_data_dir + projectName_.at(rank).toStdString() + "/" + localDataPackageName;
    std::string cur_remoteDataPackageOldPath = cur_remote_data_olddir + remoteDataOriginalName;

    if (!FTPClient.UploadFile(localDataPackagePath, cur_remoteDataPackagePath))
    {
        qCritical() << "上传 " << projectName_.at(rank) << " 到新的data文件夹失败";
    }
    else
    {
        qDebug() << "上传data到新的S_1文件夹成功";
    }

    if (!FTPClient.RemoveFile(cur_remoteDataPackageOldPath))
    {
        qCritical() << "删除data文件夹内 " << projectName_.at(rank) << " 文件失败";
    }
    else
    {
        qDebug() << "删除Data文件夹内data文件成功";
    }
    //////////////////////////////////////////////////////////////////////上传关键帧
    std::string cur_remoteFramesSavePath = "/map_mapping/" + projectName_.at(rank).toStdString() + "/" + keyframsVer;
    std::string cur_remoteFramesSaveOldPath = "/map_mapping/" + projectName_.at(rank).toStdString() + "/" + oldFramesVer.toStdString();

    if (!FTPClient.RemoveFile(cur_remoteFramesSaveOldPath))
    {
        qCritical() << "删除旧的关键帧失败";
    }
    else
    {
        qDebug() << "删除旧的关键帧成功";
    }
    if (!FTPClient.UploadFile(localFramesSavePath, cur_remoteFramesSavePath))
    {
        qCritical() << "上传关键帧失败";
    }
    else
    {
        qDebug() << "上传关键帧成功";
    }

    FTPClient.CleanupSession();
    //仅判断上传
    return std::make_tuple(ifuploadfile,localDataPackageName,newVersionStr);
}

std::tuple<std::string,std::string> Task_map::packageNewData(const int &rank, QString path)
{
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");
    QString project = listTaskDatas_.at(rank).find(projectKey).value();
    QString dataVersion = listTaskDatas_.at(rank).find(TASKkey).value();
  //////////////////////////////////////////////////////////////////////删除vmap
    QString vmapDir = QString::fromStdString(home_str_) + "/maptool-candela/CTI_MAP_PROJECT/" + project + "/DATA/" + project + "/vmap";
    dialogtools_->delDir(vmapDir);
  //////////////////////////////////////////////////////////////////////上传优化后补图数据
    uploadSuppleData(project);
  //////////////////////////////////////////////////////////////////////打包
    QString newDataVersion;
    QString dataVersionFile;
    QString PackageName;
    QString versionContent;
    QString log = "update";
    QString sign = "1";
    std::string localDataDir;
    std::vector<std::string> vec_data = split(dataVersion.toStdString(), ".");
    int numbFirst = std::stoi(vec_data[2]);
    int numbSecond = std::stoi(vec_data[1]);
    int numbThird = std::stoi(split(vec_data[0], "V").back());

    localDataDir = home_str_ + "/maptool-candela/CTI_MAP_PROJECT/" + project.toStdString() + "/DATA/";
    dataVersionFile = QString::fromStdString(localDataDir) + project + "/version_data";

    //path = path +"/MAP/"+ project + ".pcd";

    // numbFirst = 0;
    // numbSecond = 0;
    // numbThird = numbThird + 1;
    newDataVersion = QString::number(numbThird) + "." + QString::number(numbSecond) + "." + QString::number(numbFirst);

    PackageName = project + "_V" + newDataVersion + "_M" + current_date + "_data_S1.tar.gz";
    versionContent = current_date + "\n" + newDataVersion + "\n" + log + "\n" + sign;
    std::string versionStr = "V" + newDataVersion.toStdString();

    //写入文件
    writeMapData(dataVersionFile, versionContent);

    std::string cmd_str = "cd " + localDataDir + " && tar zcvf " + PackageName.toStdString() + " " + project.toStdString();

    if (std::system(cmd_str.c_str()))
    {
        qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
    }

    return std::make_tuple(PackageName.toStdString(),versionStr);
}

bool Task_map::pullTaskDataList()
{
    originalData_.clear();
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");
    std::string projectPath = "/DATA/";

    std::vector<char> data;
    std::string map;
    //int sizeSplitDataArray{0};
    const bool upload_result = FTPClient.DownloadFile(projectPath, data);
    
    //将读的信息存储，获取S0的任务列表
    // std::cout << "num: " << data.size() << std::endl;
    for (int i = 0; data.size() > i; i++)
    {   
        std::vector<std::string> mapmsgs;
        QMap<QString, QString> origin;
        if (data[i] == '\n')
        {
            QString taskinfo = QString::fromStdString(divideData(map));
            SplitString(taskinfo.toStdString(),mapmsgs,"_V");
            QString taskname = QString::fromStdString(mapmsgs.front());
            origin.insert(taskname,taskinfo);
            // qDebug() << "task： " << taskinfo;
            // qDebug() << "task： " << taskname;
            map = "";
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
//获取project的包，便于管理删除
bool Task_map::pullProjectListName(const int &rank) {
    projectVer_.clear();
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");
    std::string projectPath = "/PROJECT/" + projectName_.at(rank).toStdString() + "/DATA/";

    std::vector<char> OLDdata;
    std::string packageName;
    const bool uploadFlag = FTPClient.DownloadFile(projectPath, OLDdata);
    for (size_t i = 0; i < OLDdata.size(); i++)
    {
        if (OLDdata[i] == '\n')
        {
            QString packageinfo = QString::fromStdString(divideData(packageName));
            projectVer_.push_back(packageinfo);
            packageName = "";
        }
        else 
        {
            packageName = packageName + OLDdata[i];
        }
    }
    if (uploadFlag)
    {
        qDebug() << "拉取ＦＴＰ工程文件名成功";
    }
    FTPClient.CleanupSession();
    return uploadFlag;
}

//读取task的version
void Task_map::readLocalData()
{
    // qDebug() << "---------------------readlocal data---------------------";
    QList<QMap<QString, QString>> subListMapData;
    QMap<QString, QString> atomTaskData;
    std::cout << "num: " << projectName_.size() << std::endl;
    for (int i{0}; projectName_.size() > i; i++)
    {
        // qDebug() << "---------------------i = " << i;
        QString projectName = projectName_.at(i);
        // qDebug() << "---------------------name = " << projectName;
        QString pathTaskVer = QString::fromStdString(home_str_) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName + "/DATA/" + projectName + "/version_data";

        QFile file(pathTaskVer);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            listTaskDatas_[i].insert("LOCALTASK", "NULL");
        }
        else
        {
            QByteArray t = file.readAll();
            listTaskDatas_[i].insert("LOCALTASKTIME", "M" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::TIME)));
            listTaskDatas_[i].insert("LOCALTASK", "V" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::VERSION)));
            listTaskDatas_[i].insert("LOCALDATASIGN", "S" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::SIGN)));
        }
        file.close();
    }
    // qDebug() << "---------------------readlocal data end---------------------";
}

bool Task_map::getTaskProjectName()
{
    projectName_.clear();
    listTaskDatas_.clear();
    originalData_refersh_.clear();
    std::vector<char> data;
    
    // QMap<QString,QString>::iterator it;
    // for ( it = originalData_.begin(); it != originalData_.end(); ++it ) 
    // std::cout << "num: " << originalData_.size() << std::endl;
    for ( int i{0}; i < originalData_.size(); i++ ) 
    {
        // qDebug() << "111111111111";
        QMap<QString, QString> origin = originalData_.at(i);
        QMap<QString, QString>::iterator it = origin.begin();
        // for ( it = origin.begin(); it != origin.end(); ++it ) {
            std::string TASKPACKNAME = it.value().toStdString();
            // std::cout << "taskname = " <<  i << std::endl;
            // std::cout << "taskname = " <<  TASKPACKNAME << std::endl;
            //name
            QString projectName = it.key();
            // qDebug() << "taskprojectname = " << projectName;

            //sign
            std::vector<std::string> origins,origin_signs;
            SplitString(TASKPACKNAME,origins,"_V");
            SplitString(origins.back(),origin_signs,"_");
            QStringList signs = QString::fromStdString(origin_signs.back()).split(".");
            QString projectSign = signs.front();
            // qDebug() << "taskprojectsign = " << projectSign;

            QMap<QString, QString> taskData;
            if (projectSign == "S0")
            {
                taskData.insert("PROJECT", projectName);

                taskData.insert("TASK", QString::fromStdString(TASKPACKNAME.substr(TASKPACKNAME.find('V'), 6)));

                taskData.insert("TASKTIME", QString::fromStdString(TASKPACKNAME.substr(TASKPACKNAME.find('M'), 15)));

                taskData.insert("DATASIGN", QString::fromStdString(TASKPACKNAME.substr(TASKPACKNAME.find('S'), 2)));

                projectName_.push_back(projectName);
                listTaskDatas_.push_back(taskData);
                originalData_refersh_.push_back(origin);
            }
        // }
    }
    if (listTaskDatas_.size() == 0)
    {
        return false;
    }
    return true;
}

bool Task_map::checkoutSign(const int &rank) {
    // const int rank = searchProjectRank(project);
    if (rank < 0)
    {
        return false;
    }
    QString project = listTaskDatas_.at(rank).find(projectKey).value();
    if (listTaskDatas_.at(rank).find("DATASIGN").value() == "S0")
    {
        return true;
    }
}

const int Task_map::searchProjectRank(const QString &project)
{
    for (int i = 0; listTaskDatas_.size() > i; i++)
    {
        if (listTaskDatas_.at(i).find("PROJECT").value() == project)
        {
            return i;
        }
    }
    return -1;
}

void Task_map::chooseTask(const int &rank)
{
    qDebug() << "choose " << rank << "success";
    qDebug() << "list num: " <<  listTaskDatas_.size();
    if (listTaskDatas_.at(rank).find("TASK").value() == listTaskDatas_.at(rank).find("LOCALTASK").value() && 
        listTaskDatas_.at(rank).find("DATASIGN").value() == listTaskDatas_.at(rank).find("LOCALDATASIGN").value())
    {
        qDebug() << "enter load task";
        Q_EMIT loadTaskData(listTaskDatas_.at(rank).find("PROJECT").value());
        return;
    }
    std::vector<std::string> remoteFileName_type;
    std::string remoteProject = projectName_.at(rank).toStdString();
    std::string remoteFileInfo =  originalData_refersh_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::cout << "downloading " << remoteProject << "... " << remoteFileInfo << "... " << rank << "..." << std::endl;
    // std::thread thDownloadMap(&Choose_map::doWorkDownloadMap, this, remoteProject, remoteMap, remoteVmap, rank);
    std::thread thDownloadMap(&Task_map::doWorkDownloadTask, this, remoteProject, remoteFileInfo, rank);
    thDownloadMap.detach();
    Q_EMIT requestLoadingMoive();
    // qDebug()<<listMapDatas[rank][1];
}

void Task_map::checkoutLocalFile(QStringList &files, QString path)
{
    QDir dir(path);
    QStringList nameFilters;
    nameFilters << "*.tar.gz";
    files = dir.entryList(nameFilters);
}

void Task_map::doWorkDownloadTask(const std::string &PATH_, const std::string &remoteFileInfo, const int &rank)
{
    std::string dataName = divideData(remoteFileInfo);

    //--
    qDebug() << "------: " << QString::fromStdString(PATH_) << "... " << QString::fromStdString(remoteFileInfo) << "... "  << rank << "...";
    mkDir(QString::fromStdString(PATH_));

    std::string localDataDir = home_str_ + "/maptool-candela/CTI_MAP_PROJECT/" + PATH_ + "/DATA/";
    std::string localDataPath = localDataDir + dataName;

    std::string remoteDataDir = "/DATA/";
    std::string remoteDataPath = remoteDataDir + dataName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });

    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    //--
    const bool upload_result_data = FTPClient.DownloadFile(localDataPath, remoteDataPath);
    std::cout << "uploading..." << upload_result_data <<  std::endl;

    if (upload_result_data)
    {
        qDebug() << "download " + QString::fromStdString(dataName) + " success";
        //--删除旧的文件夹
        std::string delete_str = "cd " + localDataDir + " && rm -rf " + PATH_;
        if (std::system(delete_str.c_str()))
        {
            qCritical() << "执行 " + QString::fromStdString(delete_str) + " 失败";
        }
        else
        {
            qDebug() << "删除 " + QString::fromStdString(PATH_) + " 文件夹成功";
        }
        
        //--解压
        std::string cmd_str = "cd " + localDataDir + " && tar -zxvf " + localDataPath;

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
        listTaskDatas_[rank][LOCALTASKkey] = listTaskDatas_.at(rank).find(TASKkey).value();
        listTaskDatas_[rank][LOCALDATAsign] = listTaskDatas_.at(rank).find(DATAsign).value();
        Q_EMIT reLoadTaskSignal(rank);
    }
    //--修改本地版本文件
    QString prodate = listTaskDatas_.at(rank).find(TASKtime).value();
    QString dataVersion = listTaskDatas_.at(rank).find(TASKkey).value();
    QString prosign = listTaskDatas_.at(rank).find(DATAsign).value();
    // int numbVersion = std::stoi(split(dataVersion.toStdString(), "V").back());
    std::string current_date = split(prodate.toStdString(), "M").back();
    QString log = "pull";
    std::string sign = split(prosign.toStdString(), "S").back();

    // std::string localDataDir = home_str_ + "/maptool-candela/CTI_MAP_PROJECT/" + project.toStdString() + "/DATA/";
    QString dataVersionFile = QString::fromStdString(localDataDir) + QString::fromStdString(PATH_) + "/version_data";
    QString newDataVersion = QString::fromStdString(split(dataVersion.toStdString(), "V").back());
    QString versionContent = QString::fromStdString(current_date) + "\n" + newDataVersion + "\n" + log + "\n" + QString::fromStdString(sign);

    //写入文件
    writeMapData(dataVersionFile, versionContent);

    Q_EMIT closeLoadingMoive();
}

//空格分割字符
std::string Task_map::divideData(const std::string &data)
{
    std::istringstream iss(data);
    std::vector<std::string> strs{std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
    return strs.back();
}
void Task_map::SplitString(const std::string& s, std::vector<std::string>& v, const std::string &c) {
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

std::vector<std::string> Task_map::split(const std::string &str, const std::string &pattern)
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////任务管理
//放弃待绘图任务
bool Task_map::taskDataAbandon(const int &rank) {
    std::string projectname = projectName_.at(rank).toStdString();
    std::string path = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + projectname + "/DATA/";
    std::string remoteDataTaskName = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::string remoteDataPackagePath = cur_remote_data_abandondir + remoteDataTaskName;
    std::string cur_remoteDataPackageOldPath = cur_remote_data_olddir + remoteDataTaskName;
    //本地进行移除
    std::string localDataPackagePath = path + remoteDataTaskName;
    std::string cmd_str = "cd " + path + " && rm -rf " + remoteDataTaskName;
    //确保本地数据存在，否则不可备份数据
    if (listTaskDatas_.at(rank).find("TASK").value() == listTaskDatas_.at(rank).find("LOCALTASK").value() && 
        listTaskDatas_.at(rank).find("DATASIGN").value() == listTaskDatas_.at(rank).find("LOCALDATASIGN").value())
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
//删除待绘图任务
bool Task_map::taskDataDelet(const int &rank) {
    qDebug() << "进入待绘图任务删除。";
    std::string projectname = projectName_.at(rank).toStdString();
    std::string path = homeStr + "/maptool-candela/CTI_MAP_PROJECT/" + projectname + "/DATA/";
    std::string remoteDataOptimizeName = originalData_.at(rank).find(projectName_.at(rank)).value().toStdString();
    std::string remoteDataPackagePath = cur_remote_data_abandondir + remoteDataOptimizeName;
    std::string cur_remoteDataPackageOldPath = cur_remote_data_olddir + remoteDataOptimizeName;
    //本地删除整个项目
    std::string localDataPackagePath = path + remoteDataOptimizeName;
    std::string cmd_str = "cd " + homeStr + "/maptool-candela/CTI_MAP_PROJECT" + " && rm -rf " + projectname;
    //确保本地数据存在，否则不可备份数据
    if (listTaskDatas_.at(rank).find("TASK").value() == listTaskDatas_.at(rank).find("LOCALTASK").value() && 
        listTaskDatas_.at(rank).find("DATASIGN").value() == listTaskDatas_.at(rank).find("LOCALDATASIGN").value())
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
//完成待绘图任务
bool Task_map::taskDataComplete(const int &rank) {
    qDebug() << "进入待绘图任务完成。";
    QMessageBox::information(this, tr("提示"), tr("待绘图任务不可主动完成，请上传所有数据后，任务将会自动完成。"),
                                QMessageBox::Ok);
}
//修改待绘图任务
bool Task_map::taskDataModify(const int &rank) {
    
}