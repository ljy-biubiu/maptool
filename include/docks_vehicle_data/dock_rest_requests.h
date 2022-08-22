#ifndef DOCK_REST_REQUESTS_H
#define DOCK_REST_REQUESTS_H

#include "docks_tool/request_curl.hpp"
#include "docks_tool/docks_curl.hpp"
#include "docks_tool/area_curl.hpp"
#include "vehicle_mysql_info/Maptool.h"
// #include "hdl_graph_slam/keyframe_data.hpp"
#include <boost/optional.hpp>
#include "map_data/pointcloudmap.h"
#include "vmap_data/datatype.h"

#include "geo/geo.h"

#include <Eigen/Dense>
#include <memory>
#include <QIODevice>
// #include <QMessageBox>
#include <QObject>
#include <QDebug>
#include <QFile>
#include <opencv2/opencv.hpp>
#include <QString>
#include <QStringList>
#include <iomanip>
#include <unistd.h>

typedef pcl::PointXYZI PointT;

struct KeyFrame
{
    std::string cloud_file;
    boost::optional<Eigen::Isometry3d> estimate;
    long long int stamp;
    long node_id;
};

class DocksViewRequests
{
public:
    DocksViewRequests();
    ~DocksViewRequests();

public:
    void setDockRequestDomain(std::string domain);
    std::vector<MapDockInfo> pullMapDockInfo(std::string &projectName, double lat, double lon);
    std::string getBlockId();
    std::vector<MapInfo> pullMapVehiclaMysqlInfo(QString projectname,std::set<std::string> &robotsid);
    std::set<std::string> getRobotId(QString projectname);
    bool uploadById(std::vector<std::pair<std::string, std::vector<double>>> uploadDockInfos);
    //--
    void closeMysql();
    void initMysql();
    // void load_graph(const std::string& mappaths);
    bool isNum(std::string &str);
    bool get_keyframes(const std::string& directory, 
                       std::deque<KeyFrame> &key_frames);
    bool load_keyframe(const std::string &folder, KeyFrame &key_frame);
    void add_keyframes(std::deque<KeyFrame> &key_frames, KeyFrame &key_frame);

    //--
    void get_QuaterAndVector(const QString& directory, 
                             boost::optional<Eigen::Vector3d> &ZERO_GNSS);
    bool save_pointcloud(const std::string &filename,
                         std::deque<KeyFrame> &key_frames,
                         boost::optional<Eigen::Vector3d> &map_zero_gnss,
                         bool downsample_flag,
                         double downsample_resolution, 
                         bool saveCompressedPCD /* = true*/);
    bool writeKeyFramesFiles(QString &save_path, std::vector<Way_Point_> &keyframes_poses);
    void readKeyFramesFiles(QString &file_Path, std::vector<Way_Point_> &keyframes_poses, int &num);
    // std::vector<std::string> spaceSplit(std::string const &input);

    //--
    void setPlatformRequestDomain(std::string domain);
    void setPlatformRequesauthorizationToken(std::string authorizationToken);
    std::string pullParkToken(std::string &user, std::string &password, std::string &keyDir);
    QList<QMap<QString, QString>> pullByToken(std::string authorizationToken);
    void post_initia_dock(std::string &map_id,
                          std::string authorizationToken,
                          std::vector<double> &initial_DockInfo);
    void postUploadAllData(std::string authorizationToken, std::string uploadData);
    void deleteAllData(std::string authorizationToken, std::string dataType, std::string parkId);
    void odinaryAllData(std::string authorizationToken, std::string uploadData, std::string parkId);
    QList<QMap<int, QString>> getSignalDataIdFromParkId(std::string authorizationToken, 
                                                        std::string parkId, 
                                                        std::string dataType);
    void deletSignalDataFromId(std::string authorizationToken, std::string id);

    //--
    double getDirection(Eigen::Quaterniond &pose_q);
    Way_Point_ getDeviationFromAxis(double dire, Way_Point_ &pose, 
                                                   double offset);
    Way_Point_ getDeviationFromDeviationAxis(double dire, Way_Point_ &pose, 
                                                   double offset);
    Way_Point_ getDeviationFromQuadrant(double dire, Way_Point_ &pose, 
                                                   double offset);
    double getDeviationFromQuadrant(double dire);
    Way_Point_ setPoseFromDeviation(double direction, Way_Point_ &pose,
                                    double offset);
    std::vector<Way_Point_> setPoseFromDeviation(double direction, 
                                                 std::vector<Way_Point_> &poses, 
                                                 double offset);
    
    //--
    void mapDataGetFromMysql(std::vector<MapData> &vec_mapdata);
    void mapDataUpdateFromMysql(MapData &mapdata);
    void mapDataIncreaseFromMysql(MapData &mapdata);
    void mapDataDeleteFromMysql(MapData &mapdata);

private:
    Maptool *mapinfo_mt_;
    RequestDocks dockrequest_;
    RequestToken tokenrequest_;
    PostDock postdock_;
    DealAllData postarea_;
    RequestPark parkrequest_;
    UploadDocks dockupload_;
    DealSignalData signaldata_;
};

#endif // DOCK_REST_REQUESTS_H
