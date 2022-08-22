#include "dock_rest_requests.h"

DocksViewRequests::DocksViewRequests() {
    mapinfo_mt_ = new Maptool();
}

DocksViewRequests::~DocksViewRequests() {}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//设置环境
void DocksViewRequests::setDockRequestDomain(std::string domain) {
    dockrequest_.setRequestDomain(domain);
}
//桩点下拉
std::vector<MapDockInfo> DocksViewRequests::pullMapDockInfo(std::string &projectName, 
                                                            double lat, double lon) {

    // RequestDocks dockrequest;
    // input use topic /map_init_pose  (msg.pose.position.x, msg.pose.position.y)
    dockrequest_.setGNSSZero(lat, lon);
    std::cout << "经纬度信息+++++++++++++++: " << lat << ", " << lon << std::endl; 
    std::vector<MapDockInfo> Res_docks = dockrequest_.pullByName(projectName); //项目名获取/id获取

    std::string mapFileDirectory = projectName;
    // nh.param<std::string>("/CTI_RUN_ENV", mapFileDirectory, "");
    std::cout << "/CTI_RUN_ENV: '" << mapFileDirectory << "'" << std::endl;

    Request request;
    std::deque<RelocationSite> siteDeque;
    siteDeque = request.pullByName(mapFileDirectory);
    for (auto i = siteDeque.begin(); i < siteDeque.end(); i++) {
        RelocationSite relocationSite = *i;
        relocationSite.print();
    }

    return Res_docks;
}
std::string DocksViewRequests::getBlockId() {
    // RequestDocks dockrequest;
    return dockrequest_.turnBackBlockId();
}
//发布最新桩点信息
bool DocksViewRequests::uploadById(std::vector<std::pair<std::string, std::vector<double>>> uploadDockInfos) {
    return dockupload_.uploadById(uploadDockInfos);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//车子信息-数据库
std::set<std::string> DocksViewRequests::getRobotId(QString projectname) {
    std::string mapname ;
    //限制园区长度为20
    if (projectname.length() > 20)
    {
        mapname = projectname.toStdString().substr(0,20);
        std::cout << "切割后园区名： " << mapname << std::endl;
    } else {
        mapname = projectname.toStdString();
    }
    
    std::set<std::string> robots_id;
    mapinfo_mt_->getRobotId(mapname,robots_id);
    return robots_id;
}
std::vector<MapInfo> DocksViewRequests::pullMapVehiclaMysqlInfo(QString projectname,std::set<std::string> &robotsid) {
    std::string msg_id = projectname.toStdString();
    // std::string nowtime = systime.toStdString();

    std::cout << "传参--------------： " << msg_id << std::endl;
    std::vector<MapInfo> vec; // mapinfo结构数组  存储园区车辆信息
    mapinfo_mt_->getMapinfo(msg_id.c_str(), robotsid, vec);  // 查询并得到园区车辆信息
    return vec;
}

/*maptool工具开机连接数据库
    --获取所有项目信息
    --新增项目
    --项目内容修改
*/
void DocksViewRequests::mapDataGetFromMysql(std::vector<MapData> &vec_mapdata) {    //获取
    mapinfo_mt_->getMapData(vec_mapdata);
}
void DocksViewRequests::mapDataUpdateFromMysql(MapData &mapdata) {  //修改
    std::cout << "开始修改上传" << std::endl;
    mapinfo_mt_->updateMapData(mapdata);
}
void DocksViewRequests::mapDataIncreaseFromMysql(MapData &mapdata) {    //新增
    mapinfo_mt_->savaMapData(mapdata);
}
void DocksViewRequests::mapDataDeleteFromMysql(MapData &mapdata) {    //删除
    mapinfo_mt_->deleteMapData(mapdata);
}

void DocksViewRequests::closeMysql() {
    mapinfo_mt_->close_mysql();  //关闭数据库连接
    delete mapinfo_mt_;
}

void DocksViewRequests::initMysql() {
    mapinfo_mt_->init_mysql(); // 初始化连接数据库
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//自动调整点高度
/*
-----获取原始数据,加载,读取
-----解析原始数据,获得坐标方向
-----判断数据坐标，返回所有帧的坐标值
*/
bool DocksViewRequests::isNum(std::string &str)
{
    std::stringstream sin(str);
    double d;
    char c;
    if (!(sin >> d) || sin >> c){
        return false;
    }
    return true;
}
bool DocksViewRequests::get_keyframes(const std::string& directory, 
                                      std::deque<KeyFrame> &key_frames)
{
    key_frames.clear();
    //遍历directory目录下的所有文件夹
    boost::filesystem::path path(directory);
    boost::filesystem::directory_iterator end;
    for (boost::filesystem::directory_iterator it(path); it != end; it++)
    {
        if (boost::filesystem::is_directory(*it))
        {
            std::string folder_str = it->path().string();
            std::string folder_name_str = it->path().filename().string();
            //文件夹名称个数不为6 或 根目录 跳过
            if (folder_name_str.size() != 6 || \
                !isNum(folder_name_str) || \
                folder_name_str.find(".") != std::string::npos)
            {
              continue;
            }
            KeyFrame kf;
            if(load_keyframe(folder_str, kf))
            {
                add_keyframes(key_frames, kf);
            }
        }
    }
    return key_frames.empty() ? false : true;
}
bool DocksViewRequests::load_keyframe(const std::string &folder, KeyFrame &key_frame) {
    long node_id = -1;
    long long int val_nanosecs;
    boost::optional<Eigen::Isometry3d> estimate;
    std::ifstream ifs(folder + "/data");
    if (!ifs) {
        return false;
    }
    while (!ifs.eof())
    {
        std::string token;
        ifs >> token;
        //时间戳
        if (token == "stamp")
        {
            long long int tmp_val;
            ifs >> val_nanosecs;
            val_nanosecs *= 1000000000;
            ifs >> tmp_val;
            val_nanosecs += tmp_val;
            key_frame.stamp = val_nanosecs;
        }
        //方向，坐标
        else if (token == "estimate")
        {
            Eigen::Matrix4d mat;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    ifs >> mat(i, j);
                }
            }
            estimate = Eigen::Isometry3d::Identity();
            estimate->linear() = mat.block<3, 3>(0, 0);
            estimate->translation() = mat.block<3, 1>(0, 3);
            key_frame.estimate = estimate;
        }
        else if (token == "id")
        {
            ifs >> node_id;
            key_frame.node_id = node_id;
        }
    }
    if (node_id < 0) {
        std::cerr << "invalid node id!" << std::endl;
        return false;
    }
    //cloud路径
    key_frame.cloud_file = folder + "/cloud.pcd";
    return true;
}
void DocksViewRequests::add_keyframes(std::deque<KeyFrame> &key_frames, KeyFrame &key_frame)
{
    size_t size = key_frames.size();
    if(size == 0){
        key_frames.push_back(key_frame);
        return;
    }
    unsigned int start = 0;
    unsigned int end   = size-1;
    unsigned int index = 0;
    //--二分法快速搜索--
    while(start <= end)
    {
        unsigned int mid = (start + end)/2;
        //--时间排序
        if(key_frame.stamp > key_frames[mid].stamp){
            start = mid+1;
        }
        else{
            end = mid-1;
        }
        //--
        if(key_frame.stamp <= key_frames[start].stamp){
            index = start;
            break;
        }
        if(key_frame.stamp >= key_frames[end].stamp){
            index = end + 1;
            break;
        }
    }
    key_frames.insert(key_frames.begin() + index, key_frame);
}
//处理cloud数据,保存
void DocksViewRequests::get_QuaterAndVector(const QString& directory, 
                                            boost::optional<Eigen::Vector3d> &ZERO_GNSS) {
    std::string input_gnss_filename = directory.toStdString() + "/zero_gnss_pose";
    std::cout << "gnss file: " << input_gnss_filename << std::endl;

    ZERO_GNSS = boost::optional<Eigen::Vector3d>();
    bool map_load_zero_gnss_;
    std::ifstream ifs(input_gnss_filename);
    if (ifs) {
        while (!ifs.eof()) {
            std::string token;
            ifs >> token;
            if (token == "zero_gnss") {
                Eigen::Vector3d gnss;
                ifs >> gnss.x() >> gnss.y() >> gnss.z();
                ZERO_GNSS = gnss;
            }
        }
    }
    if (ZERO_GNSS) {
        map_load_zero_gnss_ = true;
        std::cout << "zero gnss:  " << std::setprecision(15) << ZERO_GNSS->x() << " " << ZERO_GNSS->y()
                            << " " << ZERO_GNSS->z() << std::endl;
    }
    else {
        map_load_zero_gnss_ = false;
    }
}
bool DocksViewRequests::save_pointcloud(const std::string &filename,
                                        std::deque<KeyFrame> &key_frames,
                                        boost::optional<Eigen::Vector3d> &map_zero_gnss,
                                        bool downsample_flag,
                                        double downsample_resolution, 
                                        bool saveCompressedPCD /* = true*/) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto &keyframe : key_frames) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(keyframe.cloud_file, *transformed);
        // pcl::transformPointCloud(*keyframe.second->cloud, *transformed,
        //                          keyframe.second->node->estimate().cast<float>());
        pcl::transformPointCloud(*transformed, *transformed,
                                 keyframe.estimate->cast<float>());
        std::copy(transformed->begin(), transformed->end(),
                  std::back_inserter(accumulated->points));
    }

    accumulated->is_dense = false;
    accumulated->width = accumulated->size();
    accumulated->height = 1;

    if (downsample_flag) {
      std::cout << "过滤点云,过滤值: " << downsample_resolution << std::endl;
      pcl::octree::OctreePointCloud<PointT> octree(
          downsample_resolution); // for localization use default 0.2 !!!
      octree.setInputCloud(accumulated);
      octree.addPointsFromInputCloud();
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
      octree.getOccupiedVoxelCenters(filtered->points);
      {
          pcl::octree::OctreePointCloudSearch<PointT> octree_intense(
              downsample_resolution); // for localization use default 0.2 !!!
          octree_intense.setInputCloud(accumulated);
          octree_intense.addPointsFromInputCloud();
          for (size_t i = 0, cnt = filtered->points.size(); i < cnt; i++) {
              std::vector<int> pointIdxVec;
              float intense_ave = 0.f;
              if (octree_intense.voxelSearch(filtered->points[i], pointIdxVec)) {
                  float add_cnt = pointIdxVec.size();
                  for (std::size_t j = 0; j < pointIdxVec.size(); ++j) {
                      intense_ave += accumulated->points[pointIdxVec[j]].intensity;
                  }
                  if (pointIdxVec.size() > 0) {
                      intense_ave /= add_cnt;
                      filtered->points[i].intensity = intense_ave;
                  }
              }
          }
      }
      accumulated = filtered;
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*accumulated, pcl_pc2);
    Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    if (map_zero_gnss) {
        origin.x() = int(map_zero_gnss->x() * 1e3); //
        origin.y() = int(map_zero_gnss->y() * 1e3);
        origin.z() = int(map_zero_gnss->z() * 1e3);
        orientation.w() = 1.0;
        orientation.x() = map_zero_gnss->x() * 1e3 - origin.x();
        orientation.y() = map_zero_gnss->y() * 1e3 - origin.y();
        orientation.z() = map_zero_gnss->z() * 1e3 - origin.z();
    }

    if (!saveCompressedPCD) {
        pcl::io::savePCDFile(filename, pcl_pc2, origin, orientation, true);
        return true;
    }
    pcl::PCDWriter pcd_writer;
    std::cout << "savepcd-----------------: " << filename << std::endl;
    if (pcd_writer.writeBinaryCompressed(filename, pcl_pc2, origin, orientation) == 0) { return true; }
    else
        return false;
}

/*保存原始数据点云帧数据
 */
bool DocksViewRequests::writeKeyFramesFiles(QString &save_path, std::vector<Way_Point_> &keyframes_poses) {
    if (save_path[save_path.length() - 1] != '/') save_path = save_path + "/";
    QString path_str;
    //#pragma omp parallel for
    if (keyframes_poses.size() != 0) {
        path_str = save_path + "keyframes.csv";
        qDebug() << path_str ;
    } else {
        qDebug() << "原始数据为空，不可保存." ;
        // QMessageBox::question(this, QObject::tr("提示"), QObject::tr("原始数据为空，不可保存."),
        //                 QMessageBox::Yes);
        return false;
    }
    
    if (path_str.isEmpty()) {
        return false;
    }
    QFile file;
    file.setFileName(path_str);
    int keyframe_num;
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    //---------数据格式----------
    for (size_t i = 0; i < keyframes_poses.size(); i++) {
        keyframe_num = i;
        Way_Point_ wp = keyframes_poses[i];
        QString str =
            QString::number(keyframe_num) + "," +
            QString::number(wp.point.x, 'f', 4) + "," +
            QString::number(wp.point.y, 'f', 4) + "," +
            QString::number(wp.point.z, 'f', 4) + "," +
            QString::number(wp.point.quate_x, 'f', 4) + "," +
            QString::number(wp.point.quate_y, 'f', 4) + "," +
            QString::number(wp.point.quate_z, 'f', 4) + "\n";
        file.write(str.toUtf8());
    }
    //----------------------------------------------
    file.close();
    return true;
}

/*读取原始数据帧数据
导入keyframes_poses中
*/
void DocksViewRequests::readKeyFramesFiles(QString &file_Path, std::vector<Way_Point_> &keyframes_poses, int &num) {
    QString fileName = file_Path;
    qDebug() << "file_Path = " << file_Path;

    QFile file(file_Path);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&file);
    while (!in.atEnd()) {
      QString line = in.readLine();
      QStringList sl = line.split(",");
      if (sl.count() >= 4) {
        Way_Point_ wp;
        // qDebug() << "----read start----";
        num = atoi(sl.at(0).toStdString().c_str());
        wp.point.x = atof(sl.at(1).toStdString().c_str());
        wp.point.y = atof(sl.at(2).toStdString().c_str());
        wp.point.z = atof(sl.at(3).toStdString().c_str());
        if (sl.count() > 5)
        {
            wp.point.quate_x = atof(sl.at(4).toStdString().c_str());
            wp.point.quate_y = atof(sl.at(5).toStdString().c_str());
            wp.point.quate_z = atof(sl.at(6).toStdString().c_str());
        }
        keyframes_poses.push_back(wp);
        //--
        // std::cout << "data: --------------" << estimate(0,0) << " " << estimate(1,0) << " " << estimate(2,0) << " " << std::endl;
        // qDebug() << "----read end----";
      }
    }
    file.close();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//平台环境设置
void DocksViewRequests::setPlatformRequestDomain(std::string domain) {
    tokenrequest_.setRequestDomain(domain);
    parkrequest_.setRequestDomain(domain);
    postdock_.setRequestDomain(domain);
    postarea_.setRequestDomain(domain);
    signaldata_.setRequestDomain(domain);
}

void DocksViewRequests::setPlatformRequesauthorizationToken(std::string authorizationToken) {
    tokenrequest_.setAuthorizationToken(authorizationToken);
}
//平台link token
std::string DocksViewRequests::pullParkToken(std::string &user, 
                                             std::string &password,
                                             std::string &keyDir) {

    // RequestDocks dockrequest;
    // input use topic /map_init_pose  (msg.pose.position.x, msg.pose.position.y)
    std::string linkToken = tokenrequest_.pullByUserPassword(user,password,keyDir); //用户名密码获取token
    return linkToken;
}
QList<QMap<QString, QString>> DocksViewRequests::pullByToken(std::string authorizationToken) {
    QList<QMap<QString, QString>> listParksIdName = parkrequest_.pullByToken(authorizationToken); //token获取预发布
    return listParksIdName;
}
//--初始点位发送
void DocksViewRequests::post_initia_dock(std::string &map_id,
                                         std::string authorizationToken,
                                         std::vector<double> &initial_DockInfo) {
    postdock_.postToMap(map_id, initial_DockInfo, authorizationToken); // TODO 运行环境待改
}
//--地图数据处理(批量)
void DocksViewRequests::postUploadAllData(std::string authorizationToken, std::string uploadData) { //--批量创建
    postarea_.postUploadAllData(authorizationToken, uploadData);
}
void DocksViewRequests::deleteAllData(std::string authorizationToken, std::string dataType, std::string parkId) { //--批量删除
    postarea_.deleteAllData(authorizationToken, dataType, parkId);
}
void DocksViewRequests::odinaryAllData(std::string authorizationToken, std::string uploadData, std::string parkId) { //--批量修改
    postarea_.odinaryAllData(authorizationToken, uploadData, parkId);
}
//--获取单区域数据id
QList<QMap<int, QString>> DocksViewRequests::getSignalDataIdFromParkId(std::string authorizationToken, 
                                                                       std::string parkId, 
                                                                       std::string dataType) {
    signaldata_.getByParkId(authorizationToken,parkId,dataType);
}
//--删除单区域数据
void DocksViewRequests::deletSignalDataFromId(std::string authorizationToken, std::string id) {
    signaldata_.deletSignalDataFromId(authorizationToken, id);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//偏移&边界线方向
//根据四元素获取弧度 角度（方向）
double DocksViewRequests::getDirection(Eigen::Quaterniond &pose_q) {
    double dire;
    double yaw_deg;

    Eigen::Quaterniond orient(pose_q.w(), pose_q.x(),
                              pose_q.y(), pose_q.z());
    double siny_cosp = +2.0 * (pose_q.w() * pose_q.z() + pose_q.x() * pose_q.y());
    double cosy_cosp = +1.0 - 2.0 * (pose_q.y() * pose_q.y() + pose_q.z() * pose_q.z());
    //获取弧度
    double yaw = atan2(siny_cosp, cosy_cosp);
    //获取角度
    yaw_deg = ((-yaw + M_PI * 0.5) < 0) ?
               (-yaw + 2.5 * M_PI) * 180. / M_PI :
               (-yaw + M_PI * 0.5) * 180. / M_PI;
      while ((yaw_deg >= 180) || (yaw_deg < -180)) {
          if (yaw_deg >= 180)
              yaw_deg -= 360;
          else if (yaw_deg < -180)
              yaw_deg += 360;
      }
    dire = yaw_deg;  // yaw * 180 / M_PI ;
    std::cout << "direction: " << dire << " yaw: " << yaw  << std::endl;
    return dire;
}
double DocksViewRequests::getDeviationFromQuadrant(double dire) {
    double dir;
    if (dire > 10 && dire < 80) {
        dir = dire;
    }
    else if (dire > 100 && dire < 170) {
        dir = 180 - dire;
    }
    else if (dire > -170 && dire < -80) {
        dir = -180 - dire;
    }
    else if (dire > -80 && dire < -10) {
        dir = dire;
    }
    return dir;  
}
//坐标轴方向,根据该方向直接移动
Way_Point_ DocksViewRequests::getDeviationFromDeviationAxis(double dire, Way_Point_ &pose, 
                                                   double offset)
{
    Way_Point_ target_pose;
    if (dire >= -10 && dire <= 10) { //0--x不变 y减小
        target_pose.point.x = pose.point.x;
        target_pose.point.y = pose.point.y - offset;
        target_pose.point.z = pose.point.z;
    }
    else if (dire >= -100 && dire <= -80) { //-90--y不变 x减小
        target_pose.point.x = pose.point.x - offset;
        target_pose.point.y = pose.point.y;
        target_pose.point.z = pose.point.z;
    }
    else if ((dire <= -170 && dire >= -180) || (dire >= 170 && dire <= 180)) { //180--x不变 y增大
        target_pose.point.x = pose.point.x;
        target_pose.point.y = pose.point.y + offset;
        target_pose.point.z = pose.point.z;
    }
    else if (dire <= 100 && dire >= 80) { //90--y不变 x增大
        target_pose.point.x = pose.point.x + offset;
        target_pose.point.y = pose.point.y;
        target_pose.point.z = pose.point.z;
    }
    return target_pose;
}
//坐标轴方向,顺时针旋转90度后移动
Way_Point_ DocksViewRequests::getDeviationFromAxis(double dire, Way_Point_ &pose, 
                                                   double offset)
{
    Way_Point_ target_pose;
    if (dire >= -10 && dire <= 10) { //90--y不变 x增大
        std::cout << "11" << std::endl;
        target_pose.point.x = pose.point.x + offset;
        target_pose.point.y = pose.point.y;
        target_pose.point.z = pose.point.z;
    }
    else if (dire >= -100 && dire <= -80) { //0--x不变 y减小
        std::cout << "22" << std::endl;
        target_pose.point.x = pose.point.x;
        target_pose.point.y = pose.point.y - offset;
        target_pose.point.z = pose.point.z;
    }
    else if ((dire <= -170 && dire >= -180) || (dire >= 170 && dire <= 180)) { //-90--y不变 x减小
        std::cout << "33" << std::endl;
        target_pose.point.x = pose.point.x - offset;
        target_pose.point.y = pose.point.y;
        target_pose.point.z = pose.point.z;
    }
    else if (dire <= 100 && dire >= 80) { //180--x不变 y增大
        std::cout << "44" << std::endl;
        target_pose.point.x = pose.point.x;
        target_pose.point.y = pose.point.y + offset;
        target_pose.point.z = pose.point.z;
    }
    return target_pose;
}
/*边界线方向
  --根据绘制方向逆时针旋转90度（+90）-> 偏移方向
  --0~90 90~180 -180~-90 -90~0
  --偏移1m  0->x 90->y 180->x -90->y
--*/
Way_Point_ DocksViewRequests::getDeviationFromQuadrant(double dire, Way_Point_ &pose, 
                                                   double offset) {
    double dev;
    double dir;
    Way_Point_ target_pose;
    double diff_x;
    double diff_y;
    if ((dire >= -10 && dire <= 10) || (dire >= -100 && dire <= -80) ||
        (dire <= -170 && dire >= -180) || (dire >= 170 && dire <= 180) ||
        (dire <= 100 && dire >= 80)) {
        target_pose = getDeviationFromAxis(dire,pose,offset);
    }
    else if (dire > 10 && dire < 80) {   //|| (dire > -180 && dire <= -90)
        std::cout << "1" << std::endl;
        dev = dire + 90; // 90~180 || -90~0
        dir = dire;
        // diff_x = offset * sin(fabs(dir));
        // diff_y = offset * cos(fabs(dir));
        diff_x = offset * sin(dir);
        diff_y = offset * cos(dir);
        
        target_pose.point.x = diff_x + pose.point.x;
        target_pose.point.y = diff_y + pose.point.y;
        target_pose.point.z = pose.point.z;
    }
    else if (dire > 100 && dire < 170) {
        std::cout << "2" << std::endl;
        dev = -180 + (dire - 90); //-90~-180
        dir = -180 - dev;
        diff_x = offset * cos(dir);
        diff_y = offset * sin(dir);
        
        target_pose.point.x = diff_x + pose.point.x;
        target_pose.point.y = diff_y + pose.point.y;
        target_pose.point.z = pose.point.z;
    }
    else if (dire > -170 && dire < -100) {
        std::cout << "3" << std::endl;
        dev = dire + 90; //-90~0
        dir = 90 + (-180-(dev));
        diff_x = offset * cos(dir);
        diff_y = offset * sin(dir);
        
        target_pose.point.x = pose.point.x + diff_x;
        target_pose.point.y = pose.point.y + diff_y;
        target_pose.point.z = pose.point.z;
    }
    else if (dire > -80 && dire < -10) {
        std::cout << "4" << std::endl;
        dev = 90 + dire;
        dir = dire;
        diff_x = offset * sin(dir);
        diff_y = offset * cos(dir);
        
        target_pose.point.x = pose.point.x + diff_x;
        target_pose.point.y = pose.point.y + diff_y;
        target_pose.point.z = pose.point.z;
    }
    std::cout << "偏移角度：" << dev << std::endl;
    std::cout << "夹角：" << dir << std::endl;
    // std::cout << "pose：" << pose.point.x << " " << pose.point.y << " " << pose.point.z << std::endl;
    // std::cout << "targetpose：" << target_pose.point.x << " " << target_pose.point.y << " " << target_pose.point.z << std::endl;

    return target_pose;
}
Way_Point_ DocksViewRequests::setPoseFromDeviation(double direction, Way_Point_ &pose, 
                                                   double offset) {
    //判断dire所在象限
    Way_Point_ target_pose = getDeviationFromQuadrant(direction,pose,offset);
    return target_pose;
}
std::vector<Way_Point_> DocksViewRequests::setPoseFromDeviation(double direction, 
                                                                std::vector<Way_Point_> &poses, 
                                                                double offset) {
    //判断dire所在象限
    double dev_dire = getDeviationFromQuadrant(direction);
    std::vector<Way_Point_> target_poses;
    std::cout << "夹角" << dev_dire << std::endl;
    // //坐标偏移
    for (size_t i = 0; i < poses.size(); i++)
    {
        Way_Point_ target_pose;
        if ((direction >= -10 && direction <= 10) || (direction >= -100 && direction <= -80) ||
            (direction <= -170 && direction >= -180) || (direction >= 170 && direction <= 180) ||
            (direction <= 100 && direction >= 80)) {
            target_pose = getDeviationFromDeviationAxis(direction,poses[i],offset);
        }
        else {
            double diff_x = offset * cos(dev_dire);
            double diff_y = offset * sin(dev_dire);
            
            target_pose.point.x = diff_x + poses[i].point.x;
            target_pose.point.y = diff_y + poses[i].point.y;
            target_pose.point.z = poses[i].point.z;
        }
        target_poses.push_back(target_pose);
    }
    return target_poses;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////