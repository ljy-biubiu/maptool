#include "manage_map.h"
#include "updatelog.h"
#include "FTPClient.h"
#include "QDebug"
#include "qdir.h"
#include <QDateTime>
#include <QFileDialog>

Choose_map::Choose_map(Task_map *taskMap_, ChooseMapUi *chooseMapUi_, MainWindow *mainWindow_, MapWidget *mapWidget_, QWidget *parent) 
            : QWidget(parent)
            , mainWindow(mainWindow_)
            , chooseMapUi(chooseMapUi_)
            , taskMap(taskMap_)
            , mapWidget(mapWidget_)
{
    ftpMap_ = new Ftp_map(this);

    connect(this, SIGNAL(sigRevMapData(bool, QList<QMap<QString, QString>>)),
            chooseMapUi, SLOT(getMapDataTotal(bool, QList<QMap<QString, QString>>)));

    connect(chooseMapUi, SIGNAL(sendButtonArrayData(int)), this, SLOT(chooseMap(int)));

    connect(this, SIGNAL(reLoadMapSignal(int)), chooseMapUi, SLOT(reLoadMapUiSlot(int)));

    connect(this, SIGNAL(requestLogDialog()), chooseMapUi_, SLOT(createDialog()));

    connect(chooseMapUi, SIGNAL(createNewProjectSwitch(QString)), this, SLOT(getPcdFile(QString)));

    connect(chooseMapUi, SIGNAL(uploadProjectNameSwitch(QString)), this, SLOT(uploadMapAndVmap(QString)));

    connect(chooseMapUi, SIGNAL(refreshChooseUi()), this, SLOT(refreshVmapMapTime()));

    connect(this, SIGNAL(requestLoadingMoive()), chooseMapUi, SLOT(loadingDatas()));

    connect(this, SIGNAL(closeLoadingMoive()), chooseMapUi, SLOT(EndloadingDatas()));

    connect(this, SIGNAL(uploadDocking(QString)), mainWindow, SLOT(pushNewestDockInfo(QString)));

    connect(chooseMapUi, SIGNAL(sendMapDataButtonDelet(int)), this, SLOT(mapDataDelete(int)));

    connect(chooseMapUi, SIGNAL(sentDeleteMysqlProject(QString)), this, SLOT(deleteMysqlProjectConfirm(QString)));

    connect(chooseMapUi, SIGNAL(sentModifyMysql()), this, SLOT(modifyMysqlProjectConfirm()));
    
    connect(chooseMapUi, SIGNAL(sentCreateMysql()), this, SLOT(createMysqlProjectConfirm()));

    connect(this, SIGNAL(initFtpData()), ftpMap_, SLOT(pullFtpData()));

    // vec_mapdatas_.clear();
    if (haddleData())
    {
        Q_EMIT sigRevMapData(true, listMapDatas);
        qDebug() << "初始化mapmanage数据正常";
    }
    else
    {
        Q_EMIT sigRevMapData(false, listMapDatas);
        qCritical() << "初始化mapmanage数据异常";
    }
    initToolVer();
}

bool Choose_map::haddleData()
{
    getProjectInfoFromMysql();
    const bool result_ = pullProjectName();
    mkDir();
    if (!pullOriginalData())
        return false;
    segmenteProjectData();
    readLocalData();
    checkMapAndVmapVersion();
    getSizeFromFile();
    return result_;
}

void Choose_map::initToolVer() {
    //--检查版本文件，创建
    createMaptoolVersion();
    //--获取远端最新的版本
    if (!pullToolVersionList()) {
        qCritical() << "服务器工具版本有误，获取名称失败，请检查后重试！";
        // return;
    }
    //--获取本地版本信息
    getLocalToolVersion();
}

///////////////////////////////////////////////////////////////////////

//create new project
///////////////////////////////////////////////////////////////////////
void Choose_map::createNewProject(const QString &pcdDir)
{
    std::thread thCreateNewProject(&Choose_map::doWorkcreateNewProject, this, pcdDir);
    thCreateNewProject.detach();
    Q_EMIT requestLoadingMoive();
    Q_EMIT refreshMapUi();
}

void Choose_map::createNewProjectFromTask(const QString &pcdName)
{
    std::thread thCreateNewProject(&Choose_map::doWorkcreateNewProjectFromTask, this, pcdName);
    thCreateNewProject.detach();
    Q_EMIT requestLoadingMoive();
    // Q_EMIT refreshMapUi();
}

void Choose_map::doWorkcreateNewProject(const QString &pcdDir)
{
    QString pcdName;
    pcdName = QString::fromStdString(split(pcdDir.toStdString(), "/").back());

    QString projectName = splitNewProjectName(pcdName);
    mkDir(projectName);
    if (!mvNewPcdToDir(pcdDir, projectName))
    {
        Q_EMIT closeLoadingMoive();
        return;
    }
    createNewVersion(projectName);
    loadNewProjectData(projectName);
    createNewFtpDir(projectName);
    packageAndUploadNewProject(projectName);
    Q_EMIT closeLoadingMoive();
}

void Choose_map::doWorkcreateNewProjectFromTask(const QString &pcdName)
{
    QString projectName = pcdName;
    createNewVersion(projectName);
    loadNewProjectData(projectName);
    createNewFtpDir(projectName);
    MapData mapdata;
    initNewMapData(projectName,"V-.-.-",mapdata);
    mapWidget->mapDataIncreaseFromMysql(mapdata);
    vec_mapdatas_.push_back(mapdata);
    // packageAndUploadNewProject(projectName);
    Q_EMIT closeLoadingMoive();
}

bool Choose_map::packageAndUploadNewProject(const QString &projectName)
{
    std::string dirPath = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + projectName.toStdString();
    std::string mapDirPath = dirPath + "/MAP/";
    std::string vmapDirPath = dirPath + "/VMAP/";
    std::string rmapDirPath = dirPath + "/RMAP/";
    std::string dataDirPath = dirPath + "/DATA/";
    QString signCompelet = "0"; 

    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");

    QString mapPackageName = projectName + "_V0.0.0_M" + current_date + ".tar.gz";
    QString vmapPackageName = projectName + "_V0.0.0_M" + current_date + "_vmap.tar.gz";
    QString rmapPackageName = projectName + "_V0.0.0_M" + current_date + "_R.tar.gz";
    QString dataPackageName = projectName + "_V0.0.0_M" + current_date + "_S" + signCompelet + ".tar.gz";

    std::string localMapPackagePath = mapDirPath + mapPackageName.toStdString();
    std::string localVmapPackagePath = vmapDirPath + vmapPackageName.toStdString();
    std::string localRMapPackagePath = rmapDirPath + mapPackageName.toStdString();
    std::string localDataPackagePath = dataDirPath + dataPackageName.toStdString();

    std::string remoteMapPackagePath = "/PROJECT/" + projectName.toStdString() + "/MAP/" + mapPackageName.toStdString();
    std::string remoteVmapPackagePath = "/PROJECT/" + projectName.toStdString() + "/VMAP/" + vmapPackageName.toStdString();
    std::string remoteRMapPackagePath = "/PROJECT/" + projectName.toStdString() + "/RMAP/" + rmapPackageName.toStdString();
    std::string remoteDataPackagePath = "/PROJECT/" + projectName.toStdString() + "/DATA/" + dataPackageName.toStdString();

    MapData mapdata;
    mapdata.projectName = projectName.toStdString();
    //--data
    mapdata.dataVersion = "V0.0.0";
    mapdata.dataStr = "/PROJECT/" + projectName.toStdString() + "/DATA/"
                        + dataPackageName.toStdString();
    //--map
    mapdata.mapVersion = "V0.0.0";
    mapdata.mapStr = "/PROJECT/" + projectName.toStdString() + "/MAP/" 
                        + mapPackageName.toStdString();
    //--rmap
    mapdata.rmapVersion = "V0.0.0";
    mapdata.rmapStr = "/PROJECT/" + projectName.toStdString() + "/RMAP/"
                        + rmapPackageName.toStdString();
    //--vmap
    mapdata.vmapVersion = "V0.0.0";
    mapdata.vmapStr = "/PROJECT/" + projectName.toStdString() + "/VMAP/"
                        + vmapPackageName.toStdString();
    mapWidget->mapDataIncreaseFromMysql(mapdata);
    vec_mapdatas_.push_back(mapdata);

    //--
    std::string cmd_str;

    cmd_str = "cd " + mapDirPath + " && tar zcvf " + mapPackageName.toStdString() + " " + projectName.toStdString();
    if (std::system(cmd_str.c_str()))
        return false;

    cmd_str = "cd " + vmapDirPath + " && tar zcvf " + vmapPackageName.toStdString() + " " + projectName.toStdString();
    if (std::system(cmd_str.c_str()))
        return false;

    cmd_str = "cd " + rmapDirPath + " && tar zcvf " + rmapPackageName.toStdString() + " " + projectName.toStdString();
    if (std::system(cmd_str.c_str()))
        return false;

    cmd_str = "cd " + dataDirPath + " && tar zcvf " + dataPackageName.toStdString() + " " + projectName.toStdString();
    if (std::system(cmd_str.c_str()))
        return false;

    //--
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    if (!FTPClient.UploadFile(localMapPackagePath, remoteMapPackagePath))
    {
        qCritical() << "上传 " << projectName << " MAP文件失败";
    }
    else
    {
        qDebug() << "上传MAP文件成功";
    }

    if (!FTPClient.UploadFile(localVmapPackagePath, remoteVmapPackagePath))
    {
        qCritical() << "上传 " << projectName << " VMAP文件失败";
    }
    else
    {
        qDebug() << "上传VMAP文件成功";
    }

    if (!FTPClient.UploadFile(localRMapPackagePath, remoteRMapPackagePath))
    {
        qCritical() << "上传 " << projectName << " RMAP文件失败";
    }
    else
    {
        qDebug() << "上传RMAP文件成功";
    }

    if (!FTPClient.UploadFile(localDataPackagePath, remoteDataPackagePath))
    {
        qCritical() << "上传 " << projectName << " DATA文件失败";
    }
    else
    {
        qDebug() << "上传DATA文件成功";
    }

    FTPClient.CleanupSession();
}

void Choose_map::createNewFtpDir(const QString &projectName)
{
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");
    std::string remoteProjectPath = "/PROJECT/" + projectName.toStdString();
    std::string remoteMapPath = "/PROJECT/" + projectName.toStdString() + "/MAP/";
    std::string remoteVmapPath = "/PROJECT/" + projectName.toStdString() + "/VMAP/";
    std::string remoteRMapPath = "/PROJECT/" + projectName.toStdString() + "/RMAP/";
    std::string remoteDataPath = "/PROJECT/" + projectName.toStdString() + "/DATA/";
    std::string remoteDataCompletePath = "/DATA_S1/" + projectName.toStdString();
    std::string remoteFramesSavePath = "/map_mapping/" + projectName.toStdString();

    if (FTPClient.CreateDir(remoteProjectPath))
    {
        qDebug() << "ＦＴＰ服务器创建" + projectName + "文件成功";
    }
    else
    {
        qCritical() << "ＦＴＰ服务器创建" + projectName + "文件失败";
    }
    if (FTPClient.CreateDir(remoteMapPath))
    {
        qDebug() << "ＦＴＰ服务器创建" + projectName + "MAP文件成功";
    }
    else
    {
        qCritical() << "ＦＴＰ服务器创建" + projectName + "MAP文件失败(有时候成功也会报失败)";
    }
    if (FTPClient.CreateDir(remoteVmapPath))
    {
        qDebug() << "ＦＴＰ服务器创建" + projectName + "VMAP文件成功";
    }
    else
    {
        qCritical() << "ＦＴＰ服务器创建" + projectName + "VMAP文件失败(有时候成功也会报失败)";
    }
    if (FTPClient.CreateDir(remoteRMapPath))
    {
        qDebug() << "ＦＴＰ服务器创建" + projectName + "RMAP文件成功";
    }
    else
    {
        qCritical() << "ＦＴＰ服务器创建" + projectName + "RMAP文件失败(有时候成功也会报失败)";
    }
    if (FTPClient.CreateDir(remoteDataPath))
    {
        qDebug() << "ＦＴＰ服务器创建" + projectName + "DATA文件成功";
    }
    else
    {
        qCritical() << "ＦＴＰ服务器创建" + projectName + "DATA文件失败(有时候成功也会报失败)";
    }
  //ftp上创建文件夹
    if (FTPClient.CreateDir(remoteDataCompletePath))
    {
        qDebug() << "ＦＴＰ服务器创建" + QString::fromStdString(remoteDataCompletePath) + "文件成功";
    }
    if (FTPClient.CreateDir(remoteFramesSavePath))
    {
        qDebug() << "ＦＴＰ服务器创建" + QString::fromStdString(remoteDataCompletePath) + "文件成功";
    }
    FTPClient.CleanupSession();
}

void Choose_map::loadNewProjectData(const QString &projectName)
{
    //    QList<QMap<QString,QString>> subListMapData;
    QMap<QString, QString> atomMapData_;

    atomMapData_.insert(projectKey, projectName);
    atomMapData_.insert(MAPkey, "v0.0.0");
    atomMapData_.insert(VMAPkey, "v0.0.0");
    atomMapData_.insert(LOCALMAPkey, "v0.0.0");
    atomMapData_.insert(LOCALVMAPkey, "v0.0.0");
    atomMapData_.insert(RMAPkey, "v0.0.0");
    atomMapData_.insert(DATAkey, "v0.0.0");
    atomMapData_.insert(LOACLRMAPkey, "v0.0.0");
    atomMapData_.insert(LOCALDATAkey, "v1.0.0");

    atomMapData_.insert(COMPAREVERTIONkey, "TURE");
    atomMapData_.insert(MEMORYkey, "0");
    atomMapData_.insert(RMEMORYkey, "0");

    listMapDatas.push_back(atomMapData_);
}

bool Choose_map::judgeAndCreateVersion(const QString &file, const QString &fileStr) { 
    QDir dir(file);
    if (dir.exists()) {
        QFileInfo fileINfo(fileStr);
        if (!fileINfo.isFile()) {
            std::cout << "create version file." << std::endl;
            std::string cmd_str = "touch " + fileStr.toStdString();
            if (!std::system(cmd_str.c_str())) {
                QDateTime current_date_time = QDateTime::currentDateTime();
                QString current_date = current_date_time.toString("yyyyMMddhhmmss");
                QString version = "0.0.0";
                QString versionContent = current_date + "\n" + version + "\n" + "new" + "\n" + "0";
                writeMapData(fileStr, versionContent);
            }
        }
        return true;
    } else {
        bool ok = dir.mkpath(file);
        if (ok) {
            std::string cmd_str = "touch " + fileStr.toStdString();
            if (!std::system(cmd_str.c_str())) {
                QDateTime current_date_time = QDateTime::currentDateTime();
                QString current_date = current_date_time.toString("yyyyMMddhhmmss");
                QString version = "0.0.0";
                QString versionContent = current_date + "\n" + version + "\n" + "new" + "\n" + "0";
                writeMapData(fileStr, versionContent);
            }   
        }
        return ok;
    }   
}

void Choose_map::createNewVersion(const QString &projectName)
{
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");

    QString version = "0.0.0";
    QString log = "init";

    QString mapDir = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName + "/MAP/" + projectName;
    QString vmapDir = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName + "/VMAP/" + projectName;
    QString rmapDir = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName + "/RMAP/" + projectName;
    QString dataDir = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName + "/DATA/" + projectName;
    QString pcdPath = mapDir + "/" + projectName + ".pcd";
    QString rpcdPath = rmapDir + "/" + projectName + "_r.pcd";
    QString mapVersionPath = mapDir + "/version_map3";
    QString vmapVersionPath = vmapDir + "/version_vmap";
    QString rmapVersionPath = rmapDir + "/version_map3";
    QString dataVersionPath = dataDir + "/version_data";

    QFileInfo fileInfo(pcdPath);
    //QString fileSize = QString::number(fileInfo.size());
    QString fileSize = "0"; //init for upload file
    // QFileInfo fileInfo(rpcdPath);
    //QString fileSize = QString::number(fileInfo.size());
    QString fileSize_r = "0"; //init for upload file
    QString taskComplete = "0";

    std::string cmd_str = "touch " + mapDir.toStdString() + "/version_map3 ";
    if (std::system(cmd_str.c_str()))
        qCritical() << QString::fromStdString(cmd_str) << "　失败";

    std::string cmd_str2 = "touch " + vmapDir.toStdString() + "/version_vmap ";
    if (std::system(cmd_str2.c_str()))
        qCritical() << QString::fromStdString(cmd_str2) << "　失败";

    std::string cmd_str3 = "touch " + rmapDir.toStdString() + "/version_map3 ";
    if (std::system(cmd_str3.c_str()))
        qCritical() << QString::fromStdString(cmd_str3) << "　失败";
    
    QFileInfo FileInfo(dataVersionPath);
    if (!FileInfo.isFile())
    {
        std::string cmd_str4 = "touch " + dataDir.toStdString() + "/version_data ";
        if (std::system(cmd_str4.c_str()))
            qCritical() << QString::fromStdString(cmd_str4) << "　失败";

        QString dataContent = current_date + "\n" + version + "\n" + log + "\n" + taskComplete;
        writeMapData(dataVersionPath, dataContent);
    }

    QString mapContent = current_date + "\n" + version + "\n" + log + "\n" + fileSize;
    writeMapData(mapVersionPath, mapContent);

    QString vmapContent = current_date + "\n" + version + "\n" + log;
    writeMapData(vmapVersionPath, vmapContent);

    QString rmapContent = current_date + "\n" + version + "\n" + log + "\n" + fileSize_r;
    writeMapData(rmapVersionPath, rmapContent);
    
}

bool Choose_map::mvNewPcdToDir(const QString &Dir, const QString &projectName)
{
    QString mapPath = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName + "/MAP/" + projectName;

    std::string cmd_str = "mv " + Dir.toStdString() + " " + mapPath.toStdString();
    if (std::system(cmd_str.c_str()))
    {
        qCritical() << "移动" + projectName + ".pcd到" + mapPath + "失败";
        return false;
    }
    else
    {
        qCritical() << "移动" + projectName + ".pcd到" + mapPath + "成功";
        return true;
    }
}

QString Choose_map::splitNewProjectName(const QString &projectName)
{
    // QString projectName = projectName;
    return QString::fromStdString(split(projectName.toStdString(), ".").front());
}

void Choose_map::getPcdFile(QString pcdName)
{
    createNewProject(pcdName);
}

void Choose_map::refreshVmapMapTime()
{
    Q_EMIT refreshMapUi();
}

///////////////////////////////////////////////////////////////////////

const bool Choose_map::checkoutMemory(const int &rank, QString path)
{
    qDebug() << "lis num: " << listMapDatas.size();
    QString project = listMapDatas.at(rank).find(projectKey).value();
    qDebug() << "lis pro: " << project;
    QString m_path = path + "/MAP/" + project + "/" + project + ".pcd";
    QFileInfo fileInfo(m_path);
    qDebug() << "lis memory: " << listMapDatas.at(rank).find("MEMORY").value();
    qDebug() << "size: " << QString::number(fileInfo.size());
    if (QString::number(fileInfo.size()) == listMapDatas.at(rank).find("MEMORY").value())
    {
        qDebug() << "Map文件并未修改";
        return false;
    }
    else
    {
        qDebug() << "Map文件已修改";
        return true;
    }   
}

const bool Choose_map::checkoutRmapMemory(const int &rank, QString path) {
    qDebug() << "lis num: " << listMapDatas.size();
    QString project = listMapDatas.at(rank).find(projectKey).value();
    qDebug() << "lis pro: " << project;
    QString r_path = path + "/RMAP/" + project + "/" + project + "_r.pcd";
    QFileInfo RfileInfo(r_path);
    qDebug() << "lis memory: " << listMapDatas.at(rank).find("RMEMORY").value();
    qDebug() << "size: " << QString::number(RfileInfo.size());
    if (QString::number(RfileInfo.size()) == listMapDatas.at(rank).find("RMEMORY").value())
    {
        qDebug() << "RMap文件并未修改";
        return false;
    }
    else
    {
        qDebug() << "RMap文件已修改";
        return true;
    }
}

QString Choose_map::getLogContent(const QString &type)
{
    if (type == "MAP")
    {
        qDebug() << "MapLog: " << chooseMapUi->updatelog->getMapLog();
        return chooseMapUi->updatelog->getMapLog();
    }
    else
    {
        qDebug() << "VmapLog: " << chooseMapUi->updatelog->getVmapLog();
        return chooseMapUi->updatelog->getVmapLog();
    }
}

std::string Choose_map::packageNewMap(const int &rank, QString path)
{
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");
    QString project = listMapDatas.at(rank).find(projectKey).value();
    QString mapVersion = listMapDatas.at(rank).find(MAPkey).value();
    QString newMapVersion;
    QString mapVersionFile;
    QString PackageName;
    QString versionContent;
    QString log;
    std::string localMapDir;
    std::vector<std::string> vec_data = split(mapVersion.toStdString(), ".");
    path = path + "/MAP/" + project + "/" + project + ".pcd";

    log = getLogContent("MAP");

    QFileInfo fileInfo(path);

    for (int i = 0; 100 > i; i++)  //地图最多更新100个版本
    {
        if (vec_data.front() == "V-")
        {
            newMapVersion = "1.0.0";
            break;
        }
        else if (vec_data.front() == "V" + std::to_string(i))
        {
            newMapVersion = QString::number(i + 1) + ".0.0";
            break;
        }
    }

    PackageName = project + "_V" + newMapVersion + "_M" + current_date + ".tar.gz";
    localMapDir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + project.toStdString() + "/MAP/";
    mapVersionFile = QString::fromStdString(localMapDir) + project + "/version_map3";
    versionContent = current_date + "\n" + newMapVersion + "\n" + log + "\n" + QString::number(fileInfo.size());
    QString versionStr = "V" + newMapVersion;
    newPackageVersions_.insert(MAPkey,versionStr);
    //    //写入文件
    writeMapData(mapVersionFile, versionContent);
    std::string cmd_str = "cd " + localMapDir + " && tar zcvf " + PackageName.toStdString() + " " + project.toStdString();

    if (std::system(cmd_str.c_str()))
    {
        qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
    }

    return PackageName.toStdString();
}

void Choose_map::writeMapData(const QString &path, const QString &content)
{
    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    file.write(content.toLatin1().data());
    file.close();
    // qDebug() << "写入新版MAP/VMAP Log文件: " << content;
}

const int Choose_map::searchProjectRank(const QString &project)
{
    for (int i = 0; listMapDatas.size() > i; i++)
    {
        if (listMapDatas.at(i).find("PROJECT").value() == project)
        {
            return i;
        }
    }
    return -1;
}

bool Choose_map::uploadVmaping(const int &rank, QString path)
{
    qDebug() << "--------";
    std::string localVmapPackageName = packageNewVmap(rank, path);
    newPackageNames_.insert(VMAPstr,QString::fromStdString(localVmapPackageName));
    std::string localVmapPackagePath = path.toStdString() + "/VMAP/" + localVmapPackageName;
    std::string remoteVmapPackageDir = "/PROJECT/" + projectName_.at(rank).toStdString() + "/VMAP/";
    std::string remoteVmapPackagePath = remoteVmapPackageDir + localVmapPackageName;
    std::string remoteVmapOriginalName = originalData.at(rank).find(projectName_.at(rank)).value().at(1).toStdString();
    std::string remoteVmapOriginalPath = remoteVmapPackageDir + remoteVmapOriginalName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    bool ifdeletefile_falg;
    const bool ifuploadfile = FTPClient.UploadFile(localVmapPackagePath, remoteVmapPackagePath);
    //若vmap数据为null 不删除服务器数据
    if (remoteVmapOriginalName != "NULL_V-.-.-_MNULL_vmap.tar.gz")
    {
        ifdeletefile_falg = FTPClient.RemoveFile(remoteVmapOriginalPath);
    } else {
        ifdeletefile_falg = true;
    }
    const bool ifdeleteFile = ifdeletefile_falg;

    if (!ifuploadfile)
    {
        qCritical() << "上传 " + projectName_.at(rank) + " VＭＡＰ文件失败";
    }
    else
    {
        qDebug() << "上传 " + projectName_.at(rank) + " VＭＡＰ文件成功";
    }
    if (!ifdeleteFile)
    {
        qCritical() << "删除服务器 " + projectName_.at(rank) + " VＭＡＰ文件失败";
    }
    else
    {
        qDebug() << "删除服务器 " + projectName_.at(rank) + " VＭＡＰ文件成功";
    }

    ///////////////////////////////////////////////////////////////////////临时使用 过度 上传到map3 map

    std::string cur_remoteVmapPackagePath = cur_remote_vmap3_dir + localVmapPackageName;
    std::string cur_remoteVmapPackageOldPath = cur_remote_vmap3_dir + remoteVmapOriginalName;

    if (!FTPClient.UploadFile(localVmapPackagePath, cur_remoteVmapPackagePath))
    {
        qCritical() << "上传 " << projectName_.at(rank) << " 到老的vmap文件夹失败";
    }
    else
    {
        qDebug() << "上传VMAP到老的vmap文件夹成功";
    }

    if (!FTPClient.RemoveFile(cur_remoteVmapPackageOldPath))
    {
        qCritical() << "删除vmap文件夹内 " << projectName_.at(rank) << " 文件失败";
    }
    else
    {
        qDebug() << "删除VMAP文件夹内vmap文件成功";
    }

    //////////////////////////////////////////////////////////////////////
    FTPClient.CleanupSession();

    return ifuploadfile && ifdeleteFile;
}

void Choose_map::uploadMaping(const int &rank, QString path, std::promise<bool> &promiseObj)
{
    std::string localMapPackageName = packageNewMap(rank, path);
    newPackageNames_.insert(MAPstr,QString::fromStdString(localMapPackageName));
    std::string localMapPackagePath = path.toStdString() + "/MAP/" + localMapPackageName;
    std::string remoteMapPackageDir = "/PROJECT/" + projectName_.at(rank).toStdString() + "/MAP/";
    std::string remoteMapPackagePath = remoteMapPackageDir + localMapPackageName;
    std::string remoteMapOriginalName = originalData.at(rank).find(projectName_.at(rank)).value().at((int)Tar_N::Map).toStdString();
    std::string remoteMapOriginalPath = remoteMapPackageDir + remoteMapOriginalName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    std::cout << localMapPackagePath << std::endl;
    std::cout << remoteMapPackagePath << std::endl;

    bool ifdeletefile_falg;
    const bool ifuploadfile = FTPClient.UploadFile(localMapPackagePath, remoteMapPackagePath);
    if (remoteMapOriginalName != "NULL_V-.-.-_MNULL.tar.gz")
    {
        ifdeletefile_falg = FTPClient.RemoveFile(remoteMapOriginalPath);
    } else {
        ifdeletefile_falg = true;
    }
    
    const bool ifdeletefile = ifdeletefile_falg;

    if (!ifuploadfile)
    {
        qCritical() << "上传 " + projectName_.at(rank) + " ＭＡＰ文件失败";
    }
    else
    {
        qDebug() << "上传 " + projectName_.at(rank) + " ＭＡＰ文件成功";
    }
    if (!ifdeletefile)
    {
        qCritical() << "删除服务器 " + projectName_.at(rank) + " ＭＡＰ文件失败";
    }
    else
    {
        qDebug() << "删除服务器 " + projectName_.at(rank) + " ＭＡＰ文件成功";
    }

    ///////////////////////////////////////////////////////////////////////临时使用 过度 上传到map3 map

    std::string cur_remoteMapPackagePath = cur_remote_map3_dir + localMapPackageName;
    std::string cur_remotemapPackageOldPath = cur_remote_map3_dir + remoteMapOriginalName;

    if (!FTPClient.UploadFile(localMapPackagePath, cur_remoteMapPackagePath))
    {
        qCritical() << "上传 " << projectName_.at(rank) << " 到老的map3文件夹失败";
    }
    else
    {
        qDebug() << "上传MAP到老的map文件夹成功";
    }

    if (!FTPClient.RemoveFile(cur_remotemapPackageOldPath))
    {
        qCritical() << "删除map3文件夹内 " << projectName_.at(rank) << " 文件失败";
    }
    else
    {
        qDebug() << "删除map3文件夹内map文件成功";
    }

    //////////////////////////////////////////////////////////////////////

    FTPClient.CleanupSession();

    Q_EMIT closeLoadingMoive();
    promiseObj.set_value(ifuploadfile && ifdeletefile);
}

void Choose_map::uploadRmaping(const int &rank, QString path, std::promise<bool> &promiseObj)
{
    std::string localRMapPackageName = packageNewRMap(rank, path);
    newPackageNames_.insert(RMAPstr,QString::fromStdString(localRMapPackageName));
    std::string localRMapPackagePath = path.toStdString() + "/RMAP/" + localRMapPackageName;
    std::string remoteRMapPackageDir = "/PROJECT/" + projectName_.at(rank).toStdString() + "/RMAP/";
    std::string remoteRMapPackagePath = remoteRMapPackageDir + localRMapPackageName;
    std::string remoteRMapOriginalName = originalData.at(rank).find(projectName_.at(rank)).value().at((int)Tar_N::RMap).toStdString();
    std::string remoteRMapOriginalPath = remoteRMapPackageDir + remoteRMapOriginalName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    std::cout << localRMapPackagePath << std::endl;
    std::cout << remoteRMapPackagePath << std::endl;

    const bool ifuploadfile = FTPClient.UploadFile(localRMapPackagePath, remoteRMapPackagePath);
    bool ifdeletefile_falg;
    if (remoteRMapOriginalName != "NULL_V-.-.-_MNULL_R.tar.gz")
    {
        ifdeletefile_falg = FTPClient.RemoveFile(remoteRMapOriginalPath);
    } else {
        ifdeletefile_falg = true;
    }
    const bool ifdeletefile = ifdeletefile_falg;

    if (!ifuploadfile)
    {
        qCritical() << "上传 " + projectName_.at(rank) + " RMAP文件失败";
    }
    else
    {
        qDebug() << "上传 " + projectName_.at(rank) + " RMAP文件成功";
    }
    if (!ifdeletefile)
    {
        qCritical() << "删除服务器 " + projectName_.at(rank) + " RMAP文件失败";
    }
    else
    {
        qDebug() << "删除服务器 " + projectName_.at(rank) + " RMAP文件成功";
    }

    ///////////////////////////////////////////////////////////////////////临时使用 过度 上传到map3 map

    std::string cur_remoteMapPackagePath = cur_remote_rmap_dir + localRMapPackageName;
    std::string cur_remotemapPackageOldPath = cur_remote_rmap_dir + remoteRMapOriginalName;

    if (!FTPClient.UploadFile(localRMapPackagePath, cur_remoteMapPackagePath))
    {
        qCritical() << "上传 " << projectName_.at(rank) << " 到老的rmap文件夹失败";
    }
    else
    {
        qDebug() << "上传rMAP到老的rmap文件夹成功";
    }

    if (!FTPClient.RemoveFile(cur_remotemapPackageOldPath))
    {
        qCritical() << "删除老的rmap文件夹内 " << projectName_.at(rank) << " 文件失败";
    }
    else
    {
        qDebug() << "删除rMAP文件夹内rmap文件成功";
    }

    //////////////////////////////////////////////////////////////////////

    FTPClient.CleanupSession();

    Q_EMIT closeLoadingMoive();
    promiseObj.set_value(ifuploadfile && ifdeletefile);
}

std::string Choose_map::packageNewRMap(const int &rank, QString path)
{
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");
    QString project = listMapDatas.at(rank).find(projectKey).value();
    QString rmapVersion = listMapDatas.at(rank).find(RMAPkey).value();
    QString newRmapVersion;
    QString rmapVersionFile;
    QString PackageName;
    QString versionContent;
    QString log = "upgrade";
    std::string localRmapDir;
    std::vector<std::string> vec_data = split(rmapVersion.toStdString(), ".");
    path = path + "/RMAP/" + project + "/" + project + "_r.pcd";

    QFileInfo fileInfo(path);

    for (int i = 0; 100 > i; i++)  //地图最多更新100个版本
    {
        if (vec_data.front() == "V-")
        {
            newRmapVersion = "1.0.0";
            break;
        }
        if (vec_data.front() == "V" + std::to_string(i))
        {
            newRmapVersion = QString::number(i + 1) + ".0.0";
            break;
        }
    }

    PackageName = project + "_V" + newRmapVersion + "_M" + current_date + "_R.tar.gz";
    localRmapDir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + project.toStdString() + "/RMAP/";
    rmapVersionFile = QString::fromStdString(localRmapDir) + project + "/version_map3";
    versionContent = current_date + "\n" + newRmapVersion + "\n" + log + "\n" + QString::number(fileInfo.size());
    QString versionStr = "V" + newRmapVersion;
    newPackageVersions_.insert(RMAPkey,versionStr);
    //    //写入文件
    writeMapData(rmapVersionFile, versionContent);
    std::string cmd_str = "cd " + localRmapDir + " && tar zcvf " + PackageName.toStdString() + " " + project.toStdString();

    if (std::system(cmd_str.c_str()))
    {
        qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
    }

    return PackageName.toStdString();
}

void Choose_map::uploadMapAndVmap(QString project)
{
    //    QString project;
    newPackageVersions_.clear();
    newPackageNames_.clear();
    QString path;
    bool mapUploadStatu;
    bool vmapUploadStatu;
    bool dataUploadStatu;
    bool rmapUploadStatu;
    std::string dataNewVersion;
    std::string dataNewPackageName;
    path = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + project;
    QString optimize_path = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_OPTIMIZE/" + project;

    //判断是否是任务栏新建项目
    const int num = searchProjectRank(project);
    if (num < 0)
    {
        //新项目 创建新的项目文件
        qDebug() << "新项目";
        createNewProjectFromTask(project);
        projectName_.push_back(project);
        //重新获取数据信息
        pullOriginalData();
        segmenteProjectData();
        readLocalData();
        getSizeFromFile();
    }
    const int rank = searchProjectRank(project);
    
    Q_EMIT requestLogDialog();
    //上传map
    if (checkoutMemory(rank, path))
    {
        std::promise<bool> promiseObj;
        std::future<bool> futureObj = promiseObj.get_future();
        std::thread thDownloadMap(&Choose_map::uploadMaping, this, rank, path, std::ref(promiseObj));
        Q_EMIT requestLoadingMoive();
        mapUploadStatu = futureObj.get();
        thDownloadMap.detach();
    } else {
        mapUploadStatu = true;
    }
    //上传rmap
    if (checkoutRmapMemory(rank, path))
    {
        std::promise<bool> promiseObj;
        std::future<bool> futureObj = promiseObj.get_future();
        std::thread thDownloadMap(&Choose_map::uploadRmaping, this, rank, path, std::ref(promiseObj));
        Q_EMIT requestLoadingMoive();
        rmapUploadStatu = futureObj.get();
        thDownloadMap.detach();
    } else {
        rmapUploadStatu = true;
    }
    //上传vmap
    vmapUploadStatu = uploadVmaping(rank, path);
    //上传更新dock桩点
    Q_EMIT uploadDocking(project);
    //上传data
    const int rank_data = taskMap->searchProjectRank(project);
    if (taskMap->checkoutSign(rank_data))
    {
        std::tie(dataUploadStatu, dataNewPackageName, dataNewVersion) = taskMap->uploadDataing(rank_data, path);
        newPackageVersions_.insert(DATAkey,QString::fromStdString(dataNewVersion));
        newPackageNames_.insert(DATAstr,QString::fromStdString(dataNewPackageName));
    } else {
        dataUploadStatu = true;
    }
    //--
    // if (num < 0) {
    //     std::cout << "新项目->数据库" << std::endl;
    //     putSingleProjectToMysql(rank, project);
    // } else {
    //     std::cout << "修改->数据库" << std::endl;
        modifySingleProjectToMysql(rank, project);
    // }
    
    if (mapUploadStatu && vmapUploadStatu && rmapUploadStatu && dataUploadStatu)
    {
        QMessageBox::information(NULL, "提示", "文件已经上传成功！", QMessageBox::Yes);
        qDebug() << project << " 工程文件已经上传成功！";      
        Q_EMIT refreshMapUi();
    }
    else
    {
        QMessageBox::information(NULL, "提示", "文件上传失败！请联系相关开发人员", QMessageBox::Yes);
        qCritical() << project << " 工程文件上传失败！请联系相关开发人员";
    }
}

std::string Choose_map::packageNewVmap(const int &rank, QString path)
{
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyyMMddhhmmss");

    QString project = listMapDatas.at(rank).find(projectKey).value();
    QString vmapVersion = listMapDatas.at(rank).find(VMAPkey).value();
    QString newVmapVersion;
    QString vmapVersionFile;
    QString PackageName;
    QString versionContent;
    QString log;
    std::string localMapDir;

    std::vector<std::string> vec_data = split(vmapVersion.toStdString(), ".");
    std::cout << "vmapversion: " << vmapVersion.toStdString() << std::endl;
    QString numbFirst = QString::fromStdString(vec_data[2]);
    QString numbSecond = QString::fromStdString(vec_data[1]); //std::stoi()
    QString numbThird = QString::fromStdString(split(vec_data[0], "V").back());


    log = getLogContent("VMAP");

    localMapDir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + project.toStdString() + "/VMAP/";
    vmapVersionFile = QString::fromStdString(localMapDir) + project + "/version_vmap";

    //path = path +"/MAP/"+ project + ".pcd";
    if (checkoutMemory(rank, path) || numbThird == "-")
    {
        numbFirst = QString::number(0);
        numbSecond = QString::number(0);
        if (numbThird == "-")
        {
            numbThird = QString::number(1);
        }
        else
        {
            numbThird = QString::number(numbThird.toInt() + 1);
        }
        newVmapVersion = numbThird + "." + numbSecond + "." + numbFirst;
    }
    else
    {
        if (numbFirst != "9")
        {
            numbFirst = QString::number(numbFirst.toInt() + 1);
            newVmapVersion = numbThird + "." + numbSecond + "." + numbFirst;
        }
        else
        {
            numbFirst = QString::number(0);
            numbSecond = QString::number(numbSecond.toInt() + 1);
            newVmapVersion = numbThird + "." + numbSecond + "." + numbFirst;
        }
    }

    PackageName = project + "_V" + newVmapVersion + "_M" + current_date + "_vmap.tar.gz";
    versionContent = current_date + "\n" + newVmapVersion + "\n" + log;
    QString versionStr = "V" + newVmapVersion;
    newPackageVersions_.insert(VMAPkey,versionStr);
    //写入文件
    writeMapData(vmapVersionFile, versionContent);

    std::string cmd_str = "cd " + localMapDir + " && tar zcvf " + PackageName.toStdString() + " " + project.toStdString();

    if (std::system(cmd_str.c_str()))
    {
        qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
    }

    return PackageName.toStdString();
}

void Choose_map::getSizeFromFile()
{   
    // qDebug() << "---------------size compared started------------";
    static bool flag{false};
    if (flag == false)
    {
        flag = true;
        for (int i = 0; projectName_.size() > i; i++)
        {
            QString path;
            path = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/MAP/" + projectName_.at(i) + "/version_map3";
            QFile file2(path);
            if (file2.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QByteArray t = file2.readAll();
                std::vector<std::string> spliter = split(QString(t).toStdString(), "\n");
                listMapDatas[i].insert(MEMORYkey, QString::fromStdString(spliter.back()));
            }
            else
            {
                QMap<QString, QString> atomVmapData_;
                listMapDatas[i].insert(MEMORYkey, "NULL");
            }
            // qDebug() << "get pcd memory: " << listMapDatas[i].find(MEMORYkey).value();

            QString r_path = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/RMAP/" + projectName_.at(i) + "/version_map3";
            QFile file3(r_path);
            if (file3.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QByteArray t = file3.readAll();
                std::vector<std::string> spliter = split(QString(t).toStdString(), "\n");
                listMapDatas[i].insert(RMEMORYkey, QString::fromStdString(spliter.back()));
            }
            else
            {
                QMap<QString, QString> atomVmapData_;
                listMapDatas[i].insert(RMEMORYkey, "NULL");
            }
            // qDebug() << "get r-pcd memory: " << listMapDatas[i].find(RMEMORYkey).value();
        }
    }
    else
    {
        for (int i = 0; projectName_.size() > i; i++)
        {
            QString path;
            path = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/MAP/" + projectName_.at(i) + "/version_map3";
            QFile file2(path);
            if (file2.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QByteArray t = file2.readAll();
                std::vector<std::string> spliter = split(QString(t).toStdString(), "\n");
                listMapDatas[i][MEMORYkey] = QString::fromStdString(spliter.back());
            }

            QString r_path = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/RMAP/" + projectName_.at(i) + "/version_map3";
            QFile file3(r_path);
            if (file3.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QByteArray t = file3.readAll();
                std::vector<std::string> spliter = split(QString(t).toStdString(), "\n");
                listMapDatas[i][RMEMORYkey] = QString::fromStdString(spliter.back());
            }
        }
    }
    // qDebug() << "---------------size compared end------------";
}

void Choose_map::checkMapAndVmapVersion()
{
    QString MAPkey = "MAP";
    QString VMAPkey = "VMAP";
    for (int i = 0; listMapDatas.size() > i; i++)
    {
        std::vector<std::string> vecMapVer = split(listMapDatas[i].find(MAPkey).value().toStdString(), ".");
        std::vector<std::string> vecVMapVer = split(listMapDatas[i].find(VMAPkey).value().toStdString(), ".");
        if (vecMapVer[0] == vecVMapVer[0])
        {
            listMapDatas[i].insert(COMPAREVERTIONkey, "TURE");
        }
        else
        {
            listMapDatas[i].insert(COMPAREVERTIONkey, "FALSE");
        }
    }
}

void Choose_map::mkDir(QString projectName)
{
    std::string DIR;
    DIR = home_str + "/maptool-candela/CTI_MAP_PROJECT";
    if (access(DIR.c_str(), 0))
    {
        std::string cmd_str = "cd ~/maptool-candela/ && mkdir CTI_MAP_PROJECT";
        if (std::system(cmd_str.c_str()))
        {
            qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
        }
        else
        {
            qDebug() << "创建CTI_MAP_PROJECT文件夹成功";
        }
    }

    for (int i = 0; projectName_.size() > i; i++)
    {

        DIR = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i).toStdString();
        if (access(DIR.c_str(), 0))
        {
            std::string cmd_str = "cd ~/maptool-candela/CTI_MAP_PROJECT && mkdir " + 
                                  projectName_.at(i).toStdString() + "&& cd " + 
                                  projectName_.at(i).toStdString() + " && mkdir MAP && mkdir VMAP && mkdir RMAP && mkdir DATA";
            if (std::system(cmd_str.c_str()))
            {
                qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
            }
            else
            {
                qDebug() << "创建" + projectName_.at(i) + " MAP/VMAP文件夹成功";
            }
        }
        else 
        {
            std::string rmap_dir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i).toStdString() + "/RMAP";
            if (access(rmap_dir.c_str(), 0))
            {
                std::string cmd_str = "cd ~/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i).toStdString() + " && mkdir RMAP";
                if (std::system(cmd_str.c_str()))
                {
                    qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
                }
                else
                {
                    qDebug() << "创建" + projectName_.at(i) + " RMAP文件夹成功";
                }
            }

            std::string data_dir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i).toStdString() + "/DATA";
            if (access(data_dir.c_str(), 0))
            {
                std::string cmd_str = "cd ~/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i).toStdString() + " && mkdir DATA";
                if (std::system(cmd_str.c_str()))
                {
                    qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
                }
                else
                {
                    qDebug() << "创建" + projectName_.at(i) + " DATA文件夹成功";
                }
            }
        }
    }

    if (projectName != "NULL")
    {
        DIR = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + projectName.toStdString();
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

void Choose_map::segmenteProjectData()
{
    // qDebug() << "---------------------project data---------------------";
    listMapDatas.clear();
    //将读的信息初步分割
    // std::cout << " size: " << projectName_.size() << " origin: " << originalData.size() << std::endl;
    for (int i = 0; originalData.size() > i; i++)
    {
        QString PROJECT = projectName_.at(i);
        std::string MAPNAME = originalData.at(i).find(projectName_.at(i)).value().at((int)Tar_N::Map).toStdString();
        std::string VMAPNAME = originalData.at(i).find(projectName_.at(i)).value().at((int)Tar_N::Vmap).toStdString();
        std::string RMAPNAME = originalData.at(i).find(projectName_.at(i)).value().at((int)Tar_N::RMap).toStdString();
        std::string DATANAME = originalData.at(i).find(projectName_.at(i)).value().at((int)Tar_N::Data).toStdString();
        
        std::vector<std::string> vecMapData,vecMapVersion;
        taskMap->SplitString(MAPNAME,vecMapData,"V");
        taskMap->SplitString(vecMapData.back(),vecMapVersion,"_");

        std::vector<std::string> vecVmapData,vecVmapVersion;
        taskMap->SplitString(VMAPNAME,vecVmapData,"V");
        taskMap->SplitString(vecVmapData.back(),vecVmapVersion,"_");

        std::vector<std::string> vecRmapData,vecRmapVersion;
        taskMap->SplitString(RMAPNAME,vecRmapData,"V");
        taskMap->SplitString(vecRmapData.back(),vecRmapVersion,"_");

        std::vector<std::string> vecDataData,vecDataVersion;
        taskMap->SplitString(DATANAME,vecDataData,"V");
        taskMap->SplitString(vecDataData.back(),vecDataVersion,"_");

        QMap<QString, QString> atomMapData_;

        atomMapData_.insert("PROJECT", PROJECT);

        atomMapData_.insert("MAP", "V" + QString::fromStdString(vecMapVersion.front()));

        atomMapData_.insert("VMAP", "V" + QString::fromStdString(vecVmapVersion.front()));

        atomMapData_.insert("MAPTIME", QString::fromStdString(MAPNAME.substr(MAPNAME.find('M'), 15)));

        atomMapData_.insert("VMAPTIME", QString::fromStdString(VMAPNAME.substr(VMAPNAME.find('M'), 15)));
        //--
        atomMapData_.insert("RMAP", "V" + QString::fromStdString(vecRmapVersion.front()));

        atomMapData_.insert("DATA", "V" + QString::fromStdString(vecDataVersion.front()));

        atomMapData_.insert("RMAPTIME", QString::fromStdString(RMAPNAME.substr(RMAPNAME.find('M'), 15)));

        atomMapData_.insert("DATATIME", QString::fromStdString(DATANAME.substr(DATANAME.find('M'), 15)));

        atomMapData_.insert("DATASIGN", QString::fromStdString(DATANAME.substr(DATANAME.find('S'), 2)));

        listMapDatas.push_back(atomMapData_);
    }
    // qDebug() << "---------------------project data end---------------------";
}

bool Choose_map::pullProjectName()
{
    //数据库获取名称
    bool upload_result = false;
    projectName_.clear();
    for (size_t i = 0; i < vec_mapdatas_.size(); i++) {
        projectName_.push_back(QString::fromStdString(vec_mapdatas_.at(i).projectName));
    }
    if (projectName_.size() != 0)
        upload_result = true;
    
    std::cout << "项目名状态：" << upload_result << "  项目名个数：" << projectName_.size() << std::endl;
    return upload_result;
}

//读取map跟map的ｖersion
void Choose_map::readLocalData()
{
    // qDebug() << "---------------------map readlocal data---------------------";
    QList<QMap<QString, QString>> subListMapData;
    QMap<QString, QString> atomMapData_;
    // std::cout << "size: " << projectName_.size() << " origin: " << originalData.size() << " vec: " << vec_mapdatas_.size() << 
    //              " list: " << listMapDatas.size() << std::endl;
    for (int i = 0; projectName_.size() > i; i++)
    {
        // qDebug() << "---------------------map i = " << i;
        QString pathMap = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/MAP/" + projectName_.at(i);
        QString pathMapVer = pathMap + "/version_map3";
        judgeAndCreateVersion(pathMap,pathMapVer);
        QFile file(pathMapVer);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            listMapDatas[i].insert("LOCALMAP", "NULL");
        }
        else
        {
            QByteArray t = file.readAll();
            listMapDatas[i].insert("LOCALMAPTIME", "M" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::TIME)));
            listMapDatas[i].insert("LOCALMAP", "V" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::VERSION)));
        }
        file.close();

        QString pathVmap = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/VMAP/" + projectName_.at(i);
        QString pathVmapVer = pathVmap + "/version_vmap";
        judgeAndCreateVersion(pathVmap,pathVmapVer);
        QFile file2(pathVmapVer);
        if (!file2.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            listMapDatas[i].insert("LOCALVMAP", "NULL");
        }
        else
        {
            QByteArray t = file2.readAll();
            listMapDatas[i].insert("LOCALVMAPTIME", "M" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::TIME)));
            listMapDatas[i].insert("LOCALVMAP", "V" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::VERSION)));
        }
        file2.close();

        QString pathRmap = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/RMAP/" + projectName_.at(i);
        QString pathRmapVer = pathRmap + "/version_map3";
        judgeAndCreateVersion(pathRmap,pathRmapVer);
        QFile file3(pathRmapVer);
        if (!file3.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            listMapDatas[i].insert("LOACLRMAP", "NULL");
        }
        else
        {
            QByteArray t = file3.readAll();
            listMapDatas[i].insert("LOACLRMAPTIME", "M" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::TIME)));
            listMapDatas[i].insert("LOACLRMAP", "V" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::VERSION)));
        }
        file3.close();

        QString pathData = QString::fromStdString(home_str) + "/maptool-candela/CTI_MAP_PROJECT/" + projectName_.at(i) + "/DATA/" + projectName_.at(i);
        QString pathDataVer = pathData + "/version_data";
        judgeAndCreateVersion(pathData,pathDataVer);
        QFile file4(pathDataVer);
        if (!file4.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            listMapDatas[i].insert("LOCALDATA", "NULL");
        }
        else
        {
            QByteArray t = file4.readAll();
            listMapDatas[i].insert("LOACLDATATIME", "M" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::TIME)));
            listMapDatas[i].insert("LOCALDATA", "V" + QString::fromStdString(split(QString(t).toStdString(), "\n").at(Loacl_F::VERSION)));
            listMapDatas[i].insert("LOACLDATASIGN", "S" + QString::fromStdString(split(QString(t).toStdString(), "\n").back()));
        }
        file4.close();
    }
    // qDebug() << "---------------------readlocal data end---------------------";
}

bool Choose_map::pullOriginalData()
{
    originalData.clear();
    for (int i = 0; projectName_.size() > i; i++)
    {
        QMap<QString, QList<QString>> projectMess;
        QList<QString> mapVmapData;
        for (auto mapdata : vec_mapdatas_)
        {
            if (mapdata.projectName == projectName_.at(i).toStdString())
            {
                //MAP
                if (mapdata.mapStr == "") {
                    qCritical() << "数据库缺失map信息： " + QString::fromStdString(mapdata.projectName);
                } else {
                    std::vector<std::string> vecMapData;
                    taskMap->SplitString(mapdata.mapStr,vecMapData,"/");
                    mapVmapData.push_back(QString::fromStdString(vecMapData.back()));
                }
                //VMAP
                if (mapdata.vmapStr == "") {
                    qCritical() << "数据库缺失Vmap信息： " + QString::fromStdString(mapdata.projectName);
                } else {
                    std::vector<std::string> vecMapData;
                    taskMap->SplitString(mapdata.vmapStr,vecMapData,"/");
                    mapVmapData.push_back(QString::fromStdString(vecMapData.back()));
                }
                //RMAP
                if (mapdata.rmapStr == "") {
                    qCritical() << "数据库缺失RMAP信息： " + QString::fromStdString(mapdata.projectName);
                } else {
                    std::vector<std::string> vecMapData;
                    taskMap->SplitString(mapdata.rmapStr,vecMapData,"/");
                    mapVmapData.push_back(QString::fromStdString(vecMapData.back()));
                }
                //DATA
                if (mapdata.dataStr == "") {
                    qCritical() << "数据库缺失DATA信息： " + QString::fromStdString(mapdata.projectName);
                } else {
                    std::vector<std::string> vecMapData;
                    taskMap->SplitString(mapdata.dataStr,vecMapData,"/");
                    mapVmapData.push_back(QString::fromStdString(vecMapData.back()));
                }
                projectMess.insert(projectName_.at(i), mapVmapData);
                originalData.push_back(projectMess);
                break;
            }
        }
    }
    std::cout << "原始数据: " << originalData.size() << std::endl;
    return true;
}

void Choose_map::chooseMap(const int &rank)
{
    qDebug() << "choose map: " << rank;
    if (listMapDatas.at(rank).find("MAP").value() == listMapDatas.at(rank).find("LOCALMAP").value() && 
        listMapDatas.at(rank).find("VMAP").value() == listMapDatas.at(rank).find("LOCALVMAP").value())
    {
        Q_EMIT loadMapAndVmap(listMapDatas.at(rank).find("PROJECT").value());
        return;
    }

    std::vector<std::string> remoteFileName_type;
    std::string remoteProject = projectName_.at(rank).toStdString();
    remoteFileName_type.push_back(originalData.at(rank).find(projectName_.at(rank)).value().at((int)Tar_N::Map).toStdString());
    remoteFileName_type.push_back(originalData.at(rank).find(projectName_.at(rank)).value().at((int)Tar_N::Vmap).toStdString());
    remoteFileName_type.push_back(originalData.at(rank).find(projectName_.at(rank)).value().at((int)Tar_N::RMap).toStdString());
    remoteFileName_type.push_back(originalData.at(rank).find(projectName_.at(rank)).value().at((int)Tar_N::Data).toStdString());

    // std::thread thDownloadMap(&Choose_map::doWorkDownloadMap, this, remoteProject, remoteMap, remoteVmap, rank);
    std::thread thDownloadMap(&Choose_map::doWorkDownloadMap, this, remoteProject, remoteFileName_type, rank);
    thDownloadMap.detach();
    Q_EMIT requestLoadingMoive();
    //    qDebug()<<listMapDatas[rank][1];
}

void Choose_map::checkoutLocalFile(QStringList &files, QString path)
{
    //    QString path = "/home/ljy/test1";
    QDir dir(path);
    QStringList nameFilters;
    nameFilters << "*.tar.gz";
    files = dir.entryList(nameFilters);
}

void Choose_map::doWorkDownloadMap(const std::string &PATH_, const std::vector<std::string> &remoteFileName_type, const int &rank)
{
    std::string mapName = divideData(remoteFileName_type.at((int)Tar_N::Map));
    std::string vmapName = divideData(remoteFileName_type.at((int)Tar_N::Vmap));
    std::string rmapName = divideData(remoteFileName_type.at((int)Tar_N::RMap));
    std::string dataName = divideData(remoteFileName_type.at((int)Tar_N::Data));

    //--
    std::string localMapDir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + PATH_ + "/MAP/";
    std::string localMapPath = localMapDir + mapName;

    std::string remoteMapDir = "/PROJECT/" + PATH_ + "/MAP/";
    std::string remoteMapPath = remoteMapDir + mapName;
    //--
    std::string localVmapDir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + PATH_ + "/VMAP/";
    std::string localVmapPath = localVmapDir + vmapName;

    std::string remoteVmapDir = "/PROJECT/" + PATH_ + "/VMAP/";
    std::string remoteVmapPath = remoteVmapDir + vmapName;
    //--
    std::string localRMapDir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + PATH_ + "/RMAP/";
    std::string localRMapPath = localRMapDir + rmapName;

    std::string remoteRMapDir = "/PROJECT/" + PATH_ + "/RMAP/";
    std::string remoteRMapPath = remoteRMapDir + rmapName;
    //--
    std::string localDataDir = home_str + "/maptool-candela/CTI_MAP_PROJECT/" + PATH_ + "/DATA/";
    std::string localDataPath = localDataDir + dataName;

    std::string remoteDataDir = "/PROJECT/" + PATH_ + "/DATA/";
    std::string remoteDataPath = remoteDataDir + dataName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });

    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    //--
    const bool upload_result_map = FTPClient.DownloadFile(localMapPath, remoteMapPath);
    std::cout << "uploading..." << std::endl;

    if (upload_result_map)
    {
        qDebug() << "download " + QString::fromStdString(mapName) + " success";
        std::string cmd_str = "cd " + localMapDir + " && tar -zxvf " + localMapPath;

        if (std::system(cmd_str.c_str()))
        {
            qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
        }
        else
        {
            qDebug() << "解压 " + QString::fromStdString(localMapPath) + " 文件夹成功";
        }
    }
    else
    {
        qCritical() << "download " + QString::fromStdString(mapName) + " fault";
    }
    //--
    const bool upload_result_vmap = FTPClient.DownloadFile(localVmapPath, remoteVmapPath);
    std::cout << "uploading..." << std::endl;

    if (upload_result_vmap)
    {
        qDebug() << "download " + QString::fromStdString(vmapName) + " success";
        std::string cmd_str = "cd " + localVmapDir + " && tar -zxvf " + localVmapPath;
        if (std::system(cmd_str.c_str()))
        {
            qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
        }
        else
        {
            qDebug() << "解压 " + QString::fromStdString(localVmapPath) + " 文件夹成功";
        }
    }
    else
    {
        qCritical() << "download " + QString::fromStdString(vmapName) + " fault";
    }
    //--
    const bool upload_result_rmap = FTPClient.DownloadFile(localRMapPath, remoteRMapPath);
    std::cout << "uploading..." << std::endl;

    if (upload_result_rmap)
    {
        qDebug() << "download " + QString::fromStdString(rmapName) + " success";
        std::string cmd_str = "cd " + localRMapDir + " && tar -zxvf " + localRMapPath;

        if (std::system(cmd_str.c_str()))
        {
            qCritical() << "执行 " + QString::fromStdString(cmd_str) + " 失败";
        }
        else
        {
            qDebug() << "解压 " + QString::fromStdString(localRMapPath) + " 文件夹成功";
        }
    }
    else
    {
        qCritical() << "download " + QString::fromStdString(rmapName) + " fault";
    }
    //--
    const bool upload_result_data = FTPClient.DownloadFile(localDataPath, remoteDataPath);
    std::cout << "uploading..." << std::endl;

    if (upload_result_data)
    {
        qDebug() << "download " + QString::fromStdString(dataName) + " success";
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

    if (upload_result_rmap && upload_result_data)
    {
        listMapDatas[rank][LOACLRMAPkey] = listMapDatas.at(rank).find(RMAPkey).value();
        listMapDatas[rank][LOCALDATAkey] = listMapDatas.at(rank).find(DATAkey).value();
    }
    if (upload_result_map && upload_result_vmap)
    {
        listMapDatas[rank][LOCALMAPkey] = listMapDatas.at(rank).find(MAPkey).value();
        listMapDatas[rank][LOCALVMAPkey] = listMapDatas.at(rank).find(VMAPkey).value();
        Q_EMIT reLoadMapSignal(rank);
    }

    getSizeFromFile();

    Q_EMIT closeLoadingMoive();
    //    //reloadMapUi
    //    if(haddleData())
    //    {
    //        Q_EMIT sigRevMapData(true,listMapDatas);
    //    }
}

//空格分割字符
std::string Choose_map::divideData(const std::string &data)
{
    std::istringstream iss(data);
    std::vector<std::string> strs{std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
    return strs.back();
}

//segmente map data by _

std::vector<std::string> Choose_map::split(const std::string &str, const std::string &pattern)
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

void Choose_map::segmenteVMapData(const std::vector<char> &data)
{
    std::string vmap_;
    std::string vmapName;
    std::vector<std::string> vecVMapData;

    //将读的信息初步分割
    for (int i = 0; data.size() > i; i++)
    {
        if (data[i] == '\n')
        {
            vmapMsgArrayTable.push_back(vmap_);
            vecVMapData = split(divideData(vmap_), "_");
            std::string VMapDataFinal;
            for (int numb_ = vecVMapData.size(); numb_ != 1; numb_--)
            {
                VMapDataFinal = vecVMapData[numb_ - 2] + "_" + VMapDataFinal;
            }
            VMapDataFinal = "VMap : " + VMapDataFinal;

            QMap<QString, QString> atomVmapData_;
            QString isExitFile = checkVmapFileExit(QString::fromStdString(divideData(vmap_)));

            atomVmapData_.insert("VMAP", QString::fromStdString(VMapDataFinal));
            atomVmapData_.insert("VMAPExitFlag", isExitFile);

            listMapDatas.push_back(atomVmapData_);
            vmap_ = "";
        }
        else
        {
            vmap_ = vmap_ + data[i];
        }
    }
}

QString Choose_map::checkMapFileExit(QString data)
{
    for (int i = 0; mapFileExit.size() > i; i++)
    {
        if (mapFileExit.at(i) == data)
        {
            return "true";
        }
    }
    return "false";
}

QString Choose_map::checkVmapFileExit(QString data)
{
    for (int i = 0; vmapFileExit.size() > i; i++)
    {
        if (vmapFileExit.at(i) == data)
        {
            return "true";
        }
    }
    return "false";
}

QList<QString> Choose_map::getProjectName(){
    return projectName_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//--创建工具版本文件
void Choose_map::createMaptoolVersion() {
    QString txtFileDir = qApp->applicationDirPath() + "/toolVersion";
    QFileInfo fileInfo(txtFileDir);
    if (!fileInfo.isFile())
    {
        std::string str = "cd " + qApp->applicationDirPath().toStdString() + " && touch toolVersion";
        if (std::system(str.c_str()))
        {
            qDebug() << "创建version文件失败";
        } else {
            qDebug() << "创建version文件成功";
        }
    }
}

//--获取当前本地版本信息
void Choose_map::getLocalToolVersion() {
    maptoolLocalVer_ = "";
    QString txtFileDir = qApp->applicationDirPath() + "/toolVersion";
    qDebug() << "app version : " << txtFileDir;
    QFile file(txtFileDir);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        listToolDatas_.insert("LOCALTOOLVER", "NULL");
    }
    else
    {
        QString line;
        QTextStream in(&file);  //用文件构造流
        line = in.readLine();//读取一行放到字符串里
        qDebug() << "本地版本：" << line;
        maptoolLocalVer_ = line;
        if(!line.isNull())//字符串有内容
        {
            listToolDatas_.insert("LOCALTOOLVER", line);
        } else {
            listToolDatas_.insert("LOCALTOOLVER", "NULL");
        }
    }
    file.close();
}

QString Choose_map::getMapToolLocalVer() {
    qDebug() << "localver: " << maptoolLocalVer_;
    return maptoolLocalVer_;
}

QString Choose_map::getMapToolRemoteVer() {
    qDebug() << "remotever: " << listToolDatas_.find("TOOLVERSION").value();
    return listToolDatas_.find("TOOLVERSION").value();
}

//--获取远端最新版本名
bool Choose_map::pullToolVersionList()
{
    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });
    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");
    std::string projectPath = "/maptool-run/";

    std::string strlist;
    std::string map;
    std::vector<std::string> vermsgs;
    //int sizeSplitDataArray{0};
    const bool upload_result = FTPClient.List(projectPath, strlist, true);
    if (strlist != "")
    {
        std::cout << "远端信息：" << strlist << std::endl;
        taskMap->SplitString(strlist,vermsgs,"_");
        // maptoolVerDeb_ = vermsgs[1];
        listToolDatas_.insert("PACKAGE", QString::fromStdString(strlist));
        listToolDatas_.insert("TOOLVERSION", QString::fromStdString(vermsgs[1]));
    } else {
        listToolDatas_.insert("PACKAGE", "NULL");
        listToolDatas_.insert("TOOLVERSION", "NULL");
    }
    
    //std::cout << "uploading..." << std::endl;
    if (upload_result)
    {
        qDebug() << "拉取maptool工程文件名成功";
    }
    else
    {
        qCritical() << "拉取maptool工程文件名失败";
    }
    FTPClient.CleanupSession();
    return upload_result;
}

//--拉取最新版本至本地
void Choose_map::pullMaptoolNewestDeb(QString password, std::promise<bool> &promiseObj) //qstring & error
{
    std::string verDebName = divideData(listToolDatas_.find("PACKAGE").value().toStdString());
    std::string localDebDir = home_str + "/maptool-candela/" + verDebName;
    std::string remoteDebDir = "/maptool-run/" + verDebName;

    embeddedmz::CFTPClient FTPClient([](const std::string &strLogMsg)
                                     { std::cout << strLogMsg << std::endl; });

    FTPClient.InitSession("ftp.ctirobot.com", 21, "ftp002", "Ftp002@20201013");

    //--
    const bool upload_result_data = FTPClient.DownloadFile(localDebDir, remoteDebDir);
    if (upload_result_data)
    {
        std::string dpkgstr = "echo \'" + password.toStdString() + "\' | sudo -S dpkg -i " + localDebDir;
        std::cout << "安装指令： " << dpkgstr << std::endl;
        const bool dpkg_result_data = std::system(dpkgstr.c_str());
        if (dpkg_result_data)
        {
            qCritical() << "安装 " + QString::fromStdString(localDebDir) + " 失败";
        }
        else {
            qCritical() << "安装 " + QString::fromStdString(localDebDir) + " 成功";
        }
        promiseObj.set_value(upload_result_data && !dpkg_result_data);
        return;
    }
    promiseObj.set_value(false);
}

//--更新本地版本信息
void Choose_map::writeLocalVerFile() {
    QString txtFileDir = qApp->applicationDirPath() + "/toolVersion";
    QString currentVer = listToolDatas_.find(TOOLver).value();
    writeMapData(txtFileDir,currentVer);
}

//--版本更新
void Choose_map::versionUpdate() {
    //--对比版本差异
    if (listToolDatas_.find("TOOLVERSION").value() != listToolDatas_.find("LOCALTOOLVER").value())
    {
        int reply = QMessageBox::question(this, tr("检查到最新版本，是否更新？"), tr("No：放弃,  Yes：更新."),
                                QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::Yes) {
                //用户输入密码
                bool isOK;
                QString text = QInputDialog::getText(NULL, "Input Dialog",
                                                   "请输入用户密码",
                                                   QLineEdit::Password,
                                                   "",
                                                   &isOK);
                //--拉取最新版本文件（home） 执行更新
                if (isOK) {
                    std::promise<bool> promiseObj;
                    std::future<bool> futureObj = promiseObj.get_future();
                    std::thread thDownloadDeb(&Choose_map::pullMaptoolNewestDeb, this, text, std::ref(promiseObj));
                    bool pull_result = futureObj.get();
                    thDownloadDeb.detach();
                    // const bool pull_result = pullMaptoolNewestDeb(text);
                    // createMaptoolVersion();
                    //更新本地版本细信息
                    if (pull_result) {
                        QMessageBox::question(this, tr("成功"), tr("绘图工具版本已更新成功，请重启工具生效。"),
                                QMessageBox::Yes);
                        writeLocalVerFile();
                    } else {
                        QMessageBox::question(this, tr("成功"), tr("版本更新失败！"),
                                QMessageBox::Yes);
                    } 
                }
                //--更新本地版本信息
            }
    } else {
        //不需要更新
        int reply = QMessageBox::information(this, tr("提示"), tr("工具已是最新版本，不需更新！"),
                                QMessageBox::Ok);
        return;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//--获取ftp最新数据
void Choose_map::getAllDataFromFtp() {
    if (projectFtpName_.size() == 0 || listFtpMapDatas_.size() == 0 || originalFtpData_.size() == 0) {
        Q_EMIT initFtpData();
        projectFtpName_ = ftpMap_->getProjectName();
        listFtpMapDatas_ = ftpMap_->getListMapData();
        originalFtpData_ = ftpMap_->getOriginalData();
    } else {
        return;
    } 
}
//--批量上传数据（全部ftp->数据库）
void Choose_map::putProjectInfoToMysql() {
    std::cout << "数据上传数据库" << std::endl;
    getAllDataFromFtp();
    // vec_mapdatas_.clear();
    for (size_t rank = 0; rank < projectFtpName_.size(); rank++) {
        putSingleProjectToMysql(rank,projectFtpName_.at(rank));
    }
}
//--单个添加数据(ftp->数据库)
void Choose_map::putSingleProjectToMysql(int rank, QString projectName) {
    MapData mapdata;
    modifyProject(rank,projectName,mapdata);
    mapWidget->mapDataIncreaseFromMysql(mapdata);
    vec_mapdatas_.push_back(mapdata);
}
//--批量修改（全部 仅数据库）---
void Choose_map::modifyProjectInfoToMysql() {
    for (size_t rank = 0; rank < projectName_.size(); rank++) {
        modifySingleProjectToMysql(rank, projectName_.at(rank)); 
    }
}
//--单个修改(仅数据库)
void Choose_map::modifySingleProjectToMysql(int rank, QString projectName) {
    bool isExit = false;
    for (auto mapdata : vec_mapdatas_)
    {
        if (mapdata.projectName == projectName.toStdString()) { //存在
            std::cout << "修改->数据库" << std::endl;
            updateAndAddProject(projectName, mapdata);
            std::cout << "修改->数据库->完成->上传" << std::endl;
            mapWidget->mapDataUpdateFromMysql(mapdata);
            isExit = true;
            break;
        }
    }
}
//--单个修改(ftp->数据库)
void Choose_map::modifySingleFtpToMysql(int rank, QString projectName) {
    bool isExit = false;
    for (auto mapdata : vec_mapdatas_)
    {
        if (mapdata.projectName == projectName.toStdString()) { //存在
            std::cout << "修改->数据库" << std::endl;
            modifyProject(rank, projectName, mapdata);
            mapWidget->mapDataUpdateFromMysql(mapdata);
            isExit = true;
            break;
        }
    }
    if (!isExit) { //不存在新增
        std::cout << "新项目->数据库" << std::endl;
        putSingleProjectToMysql(rank,projectName);
    }
}
//--获取所有项目数据
void Choose_map::getProjectInfoFromMysql() {
    vec_mapdatas_.clear();
    mapWidget->mapDataGetFromMysql(vec_mapdatas_);
    std::cout << "项目数：" << vec_mapdatas_.size() << std::endl;
}
//初始化新建新的数据
void Choose_map::initNewMapData(QString project, QString version, MapData &mapdata) {
    mapdata.projectName = project.toStdString();
    //--data
    mapdata.dataVersion = version.toStdString();
    mapdata.dataStr = "/PROJECT/" + project.toStdString() + "/DATA/"
                        + "NULL_V-.-.-_MNULL_data_S0.tar.gz";
    //--map
    mapdata.mapVersion = version.toStdString();
    mapdata.mapStr = "/PROJECT/" + project.toStdString() + "/MAP/" 
                        + "NULL_V-.-.-_MNULL.tar.gz";
    //--rmap
    mapdata.rmapVersion = version.toStdString();
    mapdata.rmapStr = "/PROJECT/" + project.toStdString() + "/RMAP/"
                        + "NULL_V-.-.-_MNULL_R.tar.gz";
    //--vmap
    mapdata.vmapVersion = version.toStdString();
    mapdata.vmapStr = "/PROJECT/" + project.toStdString() + "/VMAP/"
                        + "NULL_V-.-.-_MNULL_vmap.tar.gz";
    std::cout << "新项目->数据库" << std::endl;
}
//上传修改(数据库)
void Choose_map::updateAndAddProject(QString project, MapData &mapdata) {
    mapdata.projectName = project.toStdString();
    QMap<QString, QString>::iterator itr;
    //--修改version
    for (itr = newPackageVersions_.begin(); itr != newPackageVersions_.end(); itr++) {
        qDebug() << "新版本信息: " << itr.key() << " " << itr.value();
        if (itr.key() == MAPkey) {
            mapdata.mapVersion = itr.value().toStdString();
        } else if (itr.key() == RMAPkey) {
            mapdata.rmapVersion = itr.value().toStdString();
        } else if (itr.key() == VMAPkey) {
            mapdata.vmapVersion = itr.value().toStdString();
        } else if (itr.key() == DATAkey) {
            mapdata.dataVersion = itr.value().toStdString();
        }
    }
    //--修改路径
    QMap<QString, QString>::iterator ite;
    for (ite = newPackageNames_.begin(); ite != newPackageNames_.end(); ite++) {
        qDebug() << "新版本信息: " << ite.key() << " " << ite.value();
        if (ite.key() == MAPstr) {
            mapdata.mapStr = "/PROJECT/" + project.toStdString() + "/MAP/"
                            + ite.value().toStdString();
        } else if (ite.key() == RMAPstr) {
            mapdata.rmapStr = "/PROJECT/" + project.toStdString() + "/RMAP/"
                            + ite.value().toStdString();
        } else if (ite.key() == VMAPstr) {
            mapdata.vmapStr = "/PROJECT/" + project.toStdString() + "/VMAP/"
                            + ite.value().toStdString();
        } else if (ite.key() == DATAstr) {
            mapdata.dataStr = "/PROJECT/" + project.toStdString() + "/DATA/"
                            + ite.value().toStdString();
        }
    }
    qDebug() << "修改完成. ";
}
//首次添加数据库(ftp->数据库)
void Choose_map::modifyProject(int rank, QString projectName, MapData &mapdata) {
    // MapData mapdata;
    mapdata.projectName = projectName.toStdString();
    //--data
    mapdata.dataVersion = listFtpMapDatas_.at(rank).find(DATAkey).value().toStdString();
    mapdata.dataStr = "/PROJECT/" + projectName.toStdString() + "/DATA/"
                        + originalFtpData_.at(rank).find(projectName).value().at((int)Tar_N::Data).toStdString();
    //--map
    mapdata.mapVersion = listFtpMapDatas_.at(rank).find(MAPkey).value().toStdString();
    mapdata.mapStr = "/PROJECT/" + projectName.toStdString() + "/MAP/" 
                        + originalFtpData_.at(rank).find(projectName).value().at((int)Tar_N::Map).toStdString();
    //--rmap
    mapdata.rmapVersion = listFtpMapDatas_.at(rank).find(RMAPkey).value().toStdString();
    mapdata.rmapStr = "/PROJECT/" + projectName.toStdString() + "/RMAP/"
                        + originalFtpData_.at(rank).find(projectName).value().at((int)Tar_N::RMap).toStdString();
    //--vmap
    mapdata.vmapVersion = listFtpMapDatas_.at(rank).find(VMAPkey).value().toStdString();
    mapdata.vmapStr = "/PROJECT/" + projectName.toStdString() + "/VMAP/"
                        + originalFtpData_.at(rank).find(projectName).value().at((int)Tar_N::Vmap).toStdString();
}
//删除数据库内数据
void Choose_map::deleteMysqlProjectConfirm(QString projectName) {
    getAllDataFromFtp();
    qDebug() << "delete: " << projectName;
    int reply = QMessageBox::question(this, tr("是否删除数据库内此项目？"), tr("No：否,  Yes：是."),
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
            if (text != "MaptoolMysqlDelet")
            {
                QMessageBox::information(this, tr("提示"), tr("密码输入错误！不可删除该数据库数据！"),
                        QMessageBox::Ok);
                return;
            }
        }
    } else { return; }

    if (projectName == "全部") {
        for (size_t i = 0; i < vec_mapdatas_.size(); i++) {
            mapWidget->mapDataDeleteFromMysql(vec_mapdatas_.at(i));
        }
    } else {
        for (size_t i = 0; i < vec_mapdatas_.size(); i++) {
            if (vec_mapdatas_.at(i).projectName == projectName.toStdString()) {
                mapWidget->mapDataDeleteFromMysql(vec_mapdatas_.at(i));
                break;
            }
        }
    }
    Q_EMIT refreshMapUi();
}
//修改全部ftp->数据库
void Choose_map::modifyMysqlProjectConfirm() {
    qDebug() << "修改全部.";
    getAllDataFromFtp();
    for (int rank = 0; rank < projectFtpName_.size(); rank++) {
        modifySingleFtpToMysql(rank, projectFtpName_.at(rank));
    }
    Q_EMIT refreshMapUi();
}
//重新创建全部ftp->数据库
void Choose_map::createMysqlProjectConfirm() {
    int reply = QMessageBox::question(this, tr("是否重新创建全部项目？"), tr("No：否,  Yes：是."),
                                QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        //输入密码才可以创建
        bool isOK;
        QString text = QInputDialog::getText(NULL, "Input Dialog",
                                            "Please input password",
                                            QLineEdit::Password,
                                            "",
                                            &isOK);
        if (isOK)
        {
            if (text != "MaptoolMysqlCreate")
            {
                QMessageBox::information(this, tr("提示"), tr("密码输入错误！不可创建数据库数据！"),
                        QMessageBox::Ok);
                return;
            }
        }
    } else { return; }
    putProjectInfoToMysql();
    Q_EMIT refreshMapUi();
}

bool Choose_map::mapDataDelete(const int &rank) {
    qDebug() << "进入项目园区删除。";
    std::string projectname = projectName_.at(rank).toStdString();
    // std::string path = homeStr + "/maptool-candela/CTI_MAP_PROJECT/" + projectname + "/DATA/";
    // std::string remoteDataPackageName = originalData.at(rank).find(projectName_.at(rank)).value().toStdString();
    // std::string remoteDataPackagePath = remote_project_delete + remoteDataPackageName;
    std::string cur_remoteDataPackageOldPath = "/PROJECT/" + projectname;
    //本地删除整个项目
    // std::string localDataPackagePath = path + remoteDataPackageName;
    std::string cmd_str = "cd " + homeStr + "/maptool-candela/CTI_MAP_PROJECT" + " && rm -rf " + projectname;
    //确保本地数据存在，否则不可备份数据
    if (listMapDatas.at(rank).find("MAP").value() == listMapDatas.at(rank).find("LOCALMAP").value() && 
        listMapDatas.at(rank).find("VMAP").value() == listMapDatas.at(rank).find("LOCALVMAP").value())
    {
        qDebug() << "本地数据存在且版本相同";
        int reply = QMessageBox::question(this, tr("若删除则项目信息不可追回,请问是否删除此项目？"), tr("No：返回,  Yes：删除."),
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
                // const bool ifuploadfile = FTPClient.UploadFile(localDataPackagePath, remoteDataPackagePath);
                //ftp上移除数据
                const bool ifdeleteFile = FTPClient.RemoveDir(cur_remoteDataPackageOldPath);
                FTPClient.CleanupSession();
                //本地删除数据
                const bool iflocaldeleteFile = std::system(cmd_str.c_str());
                //同时删除数据库内数据
                deleteMysqlProjectConfirm(QString::fromStdString(projectname));
                return ifdeleteFile ;
            }
    }
    else
    {
        QMessageBox::question(this, tr("警告！"), tr("本地数据不存在/版本不同，不可放弃，请先确保本地数据与远端版本一致。"),
                                QMessageBox::Yes);
        return false;
    }
}