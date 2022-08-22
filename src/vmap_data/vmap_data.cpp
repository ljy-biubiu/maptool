#include "vmap_data/vmap_data.h"
#include <stdio.h>
#include <stack>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

VmapData::VmapData() { 
  clearAll(); 
  }
VmapData::~VmapData() {}

/*清除所有类型的数据*/
void VmapData::clearAll() {
  //#pragma omp parallel for
  for (int i = 0; i < Vm_T::Type_Num; i++) {
    if (!vmap_[i].empty()) {
      for (auto p : vmap_[i]) {
        if (!p.wp.empty()) {
          p.wp.clear();
        }
      }
      vmap_[i].clear();
    }
  }
}

/*依次读入所有类型地图文件------经纬度转坐标（坐标为空）
  每个文件每行中的数据到vmap_
*/
void VmapData::readVmapFiles(QStringList &fileNames) {
  purge_arrows_.clear();
  for (int i = 0; i < fileNames.size(); ++i) {
    QString fileName = fileNames.at(i);
    qDebug() << "fileName = " << fileName;

    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&file);
    while (!in.atEnd()) {
      QString line = in.readLine();
      QStringList sl = line.split(",");
      if (sl.count() >= 12) {
        VectorMap_ vectormap;
        Way_Point_ wp;

        vectormap.type_id = atoi(sl.at(0).toStdString().c_str());
        vectormap.type_num_id = atoi(sl.at(1).toStdString().c_str());
        wp.point.x = atof(sl.at(2).toStdString().c_str());
        wp.point.y = atof(sl.at(3).toStdString().c_str());
        wp.point.z = atof(sl.at(4).toStdString().c_str());
        wp.point.quate_x = atof(sl.at(5).toStdString().c_str());
        wp.point.quate_y = atof(sl.at(6).toStdString().c_str());
        wp.point.quate_z = atof(sl.at(7).toStdString().c_str());
        wp.point.quate_w = atof(sl.at(8).toStdString().c_str());
        wp.satfix.latitude = atof(sl.at(9).toStdString().c_str());
        wp.satfix.longitude = atof(sl.at(10).toStdString().c_str());
        wp.satfix.altitude = atof(sl.at(11).toStdString().c_str());

        map_projection_project(&origin_pos_, wp.satfix.latitude,
                               wp.satfix.longitude, &wp.point.y, &wp.point.x);
 
        wp.property = atoi(sl.at(12).toStdString().c_str());
        wp.ltype = atoi(sl.at(13).toStdString().c_str());
        wp.limit_vel = atof(sl.at(14).toStdString().c_str());
        if (sl.count() > 15) {
          wp.lanelet_id = atof(sl.at(15).toStdString().c_str());
        } else {
          wp.lanelet_id = 0;
        }
        vectormap.wp.push_back(wp);
        //--
        if (vectormap.type_id < Vm_T::Type_Num)
          setCurrentVectorMap(vectormap.type_id, vectormap.type_num_id,
                              vectormap);
        else
          std::cout << "type_id > Type_Num,data erro!" << std::endl;
        //--清扫边界线点
        if (vectormap.type_id == Vm_T::ClearArea && wp.property == TYPE_P1_TYPE_CLEARAREA::PURGEBOUNDARY_TYPE)
        { 
          int index = seachIdVector(vectormap.type_num_id, vmap_[vectormap.type_id]);
          if (purge_arrows_.find(vectormap.type_num_id) == purge_arrows_.end()) //不存在此条线信息
          {
            VectorMap_ vp = vmap_[vectormap.type_id].at(index);
            purge_arrows_.insert(std::make_pair(vectormap.type_num_id,vp));
          } 
          else { //存在此条线信息
            purge_arrows_.find(vectormap.type_num_id)->second = vmap_[vectormap.type_id].at(index);
          }
        }
      }
    }
    file.close();
  }
}
/*读取边界线信息*/
std::map<int, VectorMap_> VmapData::getPurgeArrowInfo() {
  return purge_arrows_;
}
/*读取gid信息*/
void VmapData::readGidFile(QString &path_str, QList<QMap<QString,QList<QString>>> &lists) {
  if (path_str.isEmpty()) {
    return;
  }
  QFile file(path_str);

  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  QTextStream in(&file);
  while (!in.atEnd()) {
    QList<QString> infos;
    QMap<QString,QList<QString>> listinfo;

    QString line = in.readLine();
    QStringList sl = line.split(",");
    if (sl.count() > 0) {
      QString numb = sl.at(0);
      std::cout << "读取gid信息：" << numb.toStdString() << std::endl;
      for (size_t i = 1; i < sl.size(); i++)
      {
        if (i < (sl.size()-1))
        {
          QStringList access =  sl.at(i).split(":");
          for (size_t j = 0; j < access.size(); j++)
          {
            std::cout << "读取gid信息：" << access.at(j).toStdString() << std::endl;
            infos.push_back(access.at(j));
          }
          continue;
        }
        infos.push_back(sl.at(i));
        std::cout << "读取gid信息：" << sl.at(i).toStdString() << std::endl;
      }
      listinfo.insert(numb,infos);
    }
    lists.push_back(listinfo);
  }
  file.close();
}
/*读取最新关联dock信息*/
void VmapData::readNewestDockFile(QString &path_str, 
                                  std::vector<std::pair<std::string, std::vector<double>>> &uploadDockInfos)
{
  if (path_str.isEmpty()) {
    return;
  }
  uploadDockInfos.clear();
  QFile file(path_str);

  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  QTextStream in(&file);
  while (!in.atEnd()) {
    QString line = in.readLine();
    QStringList sl = line.split(",");
    std::vector<double> locationVec;
    std::pair<std::string, std::vector<double>> dockInfo;

    std::string dockId = sl.at(DockFile::Dock_id).toStdString();
    locationVec.push_back(atof(sl.at(DockFile::Location_lat).toStdString().c_str()));
    locationVec.push_back(atof(sl.at(DockFile::Location_lon).toStdString().c_str()));
    locationVec.push_back(atof(sl.at(DockFile::Location_alt).toStdString().c_str()));
    locationVec.push_back(atof(sl.at(DockFile::Dock_direct).toStdString().c_str()));

    dockInfo = make_pair(dockId, locationVec);
    uploadDockInfos.push_back(dockInfo);
  }
  file.close();
}

/*把点放进对应的边界点库中
 */
void VmapData::setCurrentPurgeArrow(const int type_id, const int type_num_id,
                                    std::map<int, VectorMap_> &purge_arrows) {
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  int numb = type_num_id;
  if (index >= 0) { //存在
    if (purge_arrows.find(type_num_id) == purge_arrows.end()) //不存在此条线信息
    {
      VectorMap_ vp = vmap_[type_id].at(index);
      purge_arrows.insert(std::make_pair(numb,vp));
    } 
    else { //存在此条线信息
      purge_arrows.find(type_num_id)->second = vmap_[type_id].at(index);
    }
  }
    // purge_arrows.push_back(vmap_[type_id].at(index).wp);
  // std::cout << "边界线: " << purge_arrows.size() << std::endl;
}

/*把点放进对应类型对应编号的地图数据库vmap_中
 */
bool VmapData::setCurrentVectorMap(const int type_id, const int type_num_id,
                                   const VectorMap_ &vectormap) {
  bool ret = true;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    for (auto wp : vectormap.wp) {
      vmap_[type_id].at(index).wp.push_back(wp);
    }
  } else {
    vmap_[type_id].push_back(vectormap);
  }
  return ret;
}
/* --
 * des:检索当前编号ID是否存在,当前id是否有矢量数据
 * input:id(要检索的id),pg(要检索的源)
 * output:-1(没有检索到),>=0(检索到对应位置)
 */
int VmapData::seachIdVector(const int type_num_id,
                            const std::vector<VectorMap_> &vm) {
  bool ret = false;
  int index;
  for (index = 0; index < (int)vm.size(); index++) {
    if (vm.at(index).type_num_id == type_num_id) {
      ret = true;
      break;
    }
  }
  if (!ret) index = -1;
  return index;
}
/*一次性保存所有文件
 */
void VmapData::writeVmapFiles(std::string &save_path) {
  if (save_path[save_path.length() - 1] != '/') save_path = save_path + "/";
  //#pragma omp parallel for
  for (int i = 0; i < Vm_T::Type_Num; i++) {
    if (!getVectorType((Vm_T)i).empty()) {
      QString path_str =
          QString::fromStdString(save_path + type_name_str[i] + ".csv");
      qDebug() << path_str ;
      writeVmapFile(path_str, i);
    }
  }
}
/*保存某个类型的文件----坐标转经纬度（经纬度为空）
 */
void VmapData::writeVmapFile(QString &path_str, int &type_id) {
  if (path_str.isEmpty()) {
    return;
  }
  QFile file;
  file.setFileName(path_str);
  //判断文件是否可写
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
  //---------数据格式----------
  for (const auto vectormap : getVectorType((Vm_T)type_id)) {
    int type_id = vectormap.type_id;
    int type_num_id = vectormap.type_num_id;
    for (auto wp : vectormap.wp) {
      // std::cout << "aaaaa: " << wp.satfix.latitude << wp.satfix.longitude << std::endl;
      // std::cout << "origin信息： " << origin_pos_.lat_rad << " " << origin_pos_.lon_rad << std::endl;
      map_projection_reproject(&origin_pos_, wp.point.y, wp.point.x,
                               &wp.satfix.latitude, &wp.satfix.longitude);
      // std::cout << "bbbbb: " << wp.satfix.latitude << wp.satfix.longitude << std::endl;
      if (!mapdataformat_.vmapdataFormat(type_id, wp, origin_pos_)) {
        std::cout << " vmapdata 格式有误，"
                  << "类型：" << type_name_str[(Vm_T)type_id] << ";"
                  << "id:" << type_num_id << std::endl;
      }
      QString str =
          QString::number(type_id) + "," + QString::number(type_num_id) + "," +
          QString::number(wp.point.x, 'f', 4) + "," +
          QString::number(wp.point.y, 'f', 4) + "," +
          QString::number(wp.point.z, 'f', 4) + "," +
          QString::number(wp.point.quate_x, 'f', 4) + "," +
          QString::number(wp.point.quate_y, 'f', 4) + "," +
          QString::number(wp.point.quate_z, 'f', 4) + "," +
          QString::number(wp.point.quate_w, 'f', 4) + "," +
          QString::number(wp.satfix.latitude, 'f', 8) + "," +
          QString::number(wp.satfix.longitude, 'f', 8) + "," +
          QString::number(wp.satfix.altitude, 'f', 8) + "," +
          QString::number(wp.property) + "," + QString::number(wp.ltype) + "," +
          QString::number(wp.limit_vel, 'f', 4) + "," +
          QString::number(wp.lanelet_id) + "\n";
      file.write(str.toUtf8());
    }
  }
  //----------------------------------------------
  file.close();
}
//生成json文件
std::string VmapData::writeVmapJson(std::string projectname, std::string &path_str, int &type_id, std::string blockId) {
    Json::Value root;/*JSON文件的根节点*/
    //子节点
    Json::Value cleanarea;
    Json::StyledWriter writer;
    // std::string dataType;
    
     //根节点的属性
    root["parkId"] = blockId;
    root["parkName"] = projectname;
    
    for (const auto vectormap : getVectorType((Vm_T)type_id)) {
      //区域
      // std::cout << "enter json" << std::endl;
      /*子节点的根节点*/
      Json::Value area;
      int type_id = vectormap.type_id;
      int type_num_id = vectormap.type_num_id;
      int property_id = vectormap.wp[0].property;
      
      if (type_id == (int)Vm_T::ClearArea)
      {
        root["areaType"] = data_Type[type_id];
        switch (property_id) {
          case 0:
            area["attribute"] = "SWEEP";
            break;
          case 1:
            area["attribute"] = "NONSWEEP";
            break;
          case 2:
            area["attribute"] = "CLEAN";
            break;
          case 3:
            area["attribute"] = "PURGE";
            break;
          default:
            area["attribute"] = "";
            break;
          } 
      } else {
        area["attribute"] = "NORMAL";
        root["dataType"] = data_Type[type_id];
        std::cout << "dataType: " << data_Type[type_id] << std::endl;
      }

      area["index"] = type_num_id;
      for (auto wp : vectormap.wp) {
        //区域对应点
        // std::cout << "enter point" << std::endl;
        map_projection_reproject(&origin_pos_, wp.point.y, wp.point.x,
                               &wp.satfix.latitude, &wp.satfix.longitude);
        double latitude = wp.satfix.latitude;
        double longitude = wp.satfix.longitude;
        double altitude = wp.satfix.altitude;
        //子节点
        Json::Value pos;
        pos["latitude"] = latitude;
        pos["longitude"] = longitude;
        pos["altitude"] = altitude;
        /*将子节点挂在子节点的根节点上*/
        area["point"].append(pos);
      }
      // std::cout << "enter area" << std::endl;
      //所属园区以及点位所有点位信息,挂到根节点
      root["area"].append(area);
      // std::cout << "quit area" << std::endl;
    }
    std::string uploadArea = root.toStyledString();
    // return uploadArea;
    std::string json_file = writer.write(root);
    // areaupload_.uploadInfo(json_file);

    std::ofstream desFile(path_str, std::ios::out | std::ios::app);
    if (!desFile.is_open())
    {
      return uploadArea;
    }
    desFile << json_file;
  	desFile.close();
    return uploadArea;
}
std::vector<Way_Point_> VmapData::jsonGet(std::string fileStr) {
    std::vector<Way_Point_> poses_str;
    std::vector<QString> points;
    QFile file;
    file.setFileName(QString::fromStdString(fileStr));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return poses_str;

    QTextStream in(&file);
    while (!in.atEnd()) {
      std::cout << "enter------" << std::endl;
      QString line = in.readLine();
      std::string result = line.toStdString();
      // std::cout << "数据: " << result << std::endl;
      // Json::Reader Reader;
      // Json::Value value_res;
      // if (Reader.parse(result, value_res)) {
      //   Json::Value arrValue = value_res["route"];
      //   for (int j = 0; j < arrValue.size(); j++)
      //   {
          // std::string pose = arrValue[j].asString();
      QStringList pss = line.split("],");
      for (size_t i = 0; i < pss.size(); i++) {
        points.push_back(pss.at(i));
        // std::cout << "line数据: " << pss.at(i).toStdString() << std::endl;
      }  
      //   }
      // }
      std::cout << "line数据个数: " << points.size() << std::endl;
      for (int j = 0; j < points.size(); j++) {
        QString pt = points.at(j);
        if (pt.isEmpty())
        {
          std::cout << "空" << std::endl;
          continue;
        }
        
        QStringList pts = pt.split(",");
        Way_Point_ wp;
        wp.satfix.latitude = pts[0].split("[").back().toDouble();
        wp.satfix.longitude = pts[1].toDouble();
        wp.satfix.altitude = pts[2].toDouble();
        map_projection_project(&origin_pos_, wp.satfix.latitude,
                               wp.satfix.longitude, &wp.point.y, &wp.point.x);
        wp.point.z = wp.satfix.altitude;
        // std::cout << "pose info: " << wp.satfix.latitude << " " << 
        //                               wp.satfix.longitude << " " <<
        //                               wp.satfix.altitude << " " <<
        //                               wp.point.x << " " << 
        //                               wp.point.y << " " <<
        //                               wp.point.z << " " << std::endl;
        // std::cout << "pose------数据: " << poses_str.size() << std::endl;
        poses_str.push_back(wp);
      }
    }
    std::cout << "pose数据: " << poses_str.size() << std::endl;
    return poses_str;
} 
/*保存gid文件信息*/
void VmapData::writeGidFile(QString &path_str, QList<QMap<QString,QList<QString>>> &lists) {
  if (path_str.isEmpty()) {
    return;
  }
  QFile file;
  file.setFileName(path_str);
  //判断文件是否可写
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
  //---------数据格式----------
  for (int i{0}; i < lists.size(); i++ ) {
    QMap<QString, QList<QString>> origin = lists.at(i);
    QMap<QString, QList<QString>>::iterator it = origin.begin();
    QList<QString> infos = it.value();

    QString str = it.key() + ",";
    int numInfo = infos.size();
    for (int i = 0; i < numInfo; i++) {
      //以“#”分隔
      if (i == (numInfo-1))
      {
        str = str + "," + infos.at(i);
        break;
      }
      else if (i == 0) {
        str = str + infos.at(i);
        continue;
      }
      str = str + ":" + infos.at(i);
    }
    str = str + "\n";
    file.write(str.toUtf8());
  }
  //----------------------------------------------
  file.close();
}
/*读取对应类型的数据*/
std::vector<VectorMap_> &VmapData::getVectorType(const Vm_T type_id) {
  return vmap_[type_id];
}
std::vector<VectorMap_> &VmapData::getCurrentMap() {
  //#pragma omp parallel for
  for (int i = 0; i < Vm_T::Type_Num; i++) {
    if (!getVectorType((Vm_T)i).empty()) {
      vectormap_.insert(vectormap_.end(), getVectorType((Vm_T)i).begin(),
                        getVectorType((Vm_T)i).end());
    }
  }
}
/* --
 * des:读取当前索引index指向的点
 * input:type_id(类型ID),type_num_id(编号ID),point_index(点索引),point(返回点数据)
 * output:false(不存在),true(存在)
 */
bool VmapData::getCurrentTypeNumIdPoint(const int type_id,
                                        const int type_num_id,
                                        const int point_index,
                                        Way_Point_ &w_p) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num =
        vmap_[type_id].at(index).wp.size();  // num对应某类型某编号下wp点的个数
    // std::cout << "point_index: " << point_index << std::endl;
    if (point_index >= 0 && point_index < num) {
      w_p = vmap_[type_id].at(index).wp.at(point_index);
      ret = true;
    }
  }
  return ret;
}
//////////////////
/*依照枚举顺序，随机获取活跃类型点
 */
bool VmapData::getRandActiveId(int &type_id, int &type_num_id) {
  bool ret = false;
  for (int i = 0; i < Vm_T::Type_Num; i++) {
    if (vmap_[i].size() > 0) {
      std::cout << "size > 0 " << std::endl;
      type_id = vmap_[i].back().type_id;
      type_num_id = vmap_[i].back().type_num_id;
      ret = true;
      break;
    }
  }
  return ret;
}
/*通过类型ID 查找它的空闲的一条线*/
int VmapData::getRandEmptyTypeNumId(const int &type_id) {
  int size = vmap_[type_id].size();
  int index = size;
  for (int i = 0; i < size; i++) {
    bool isFinded = false;
    for (int j = 0; j < size; j++) {
      if (i == vmap_[type_id].at(j).type_num_id &&
          !vmap_[type_id].at(j).wp.empty()) {
        isFinded = true;
        break;
      }
    }
    if (!isFinded) {
      index = i;
      break;
    }
  }
  return index;
}
/*通过类型ID 查找它的最后一条线*/
int VmapData::getRandActiveTypeNumId(const int &type_id) {
  int ret = 0;
  if (vmap_[type_id].size() > 0) {
    ret = vmap_[type_id].back().type_num_id;
  }
  return ret;
}

//删除最后一个点
bool VmapData::setTypeNumIdDelBackPoint(const int type_id,
                                        const int type_num_id) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    if (num > 0) {
      vmap_[type_id].at(index).wp.pop_back();
      ret = true;
    }
  }
  return ret;
}
//删除所有的点
bool VmapData::setTypeNumIdDelPoints(const int type_id, const int type_num_id) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    if (num > 0) {
      vmap_[type_id].at(index).wp.clear();
      ret = true;
    }
  }
  return ret;
}

/* --
 * des:读取当前类型ID数量
 * input:type_id(类型ID)
 * output:-1(不存在),>=0(数量)
 */
int VmapData::getCurrentTypeIdNumber(const int type_id) {
  if (type_id < Vm_T::Type_Num) {
    return vmap_[type_id].size();
  } else {
    return (-1);
  }
}
/* --
 * des:读取当前编号ID指向point数量，id的矢量数据有几个
 * input:type_id(类型ID),type_num_id(编号ID)
 * output:-1(不存在),>=0(数量)
 */
int VmapData::getCurrentTypeNumIdNumber(const int type_id,
                                        const int type_num_id) {
  int ret = -1;
  int index = seachIdVector(type_num_id, vmap_[type_id]);  //当前id是否存在
  if (index >= 0) {                                        //存在
    ret = vmap_[type_id].at(index).wp.size();
  }
  return ret;
}
/* --
 * des:读取当前编号waypoint
 * input:type_id(类型ID),type_num_id(编号ID),vectormap(返回点数据)
 * output:false(不存在),true(存在)
 */
bool VmapData::getCurrentVectorMap(const int type_id, const int type_num_id,
                                   VectorMap_ &vectormap) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    vectormap = vmap_[type_id].at(index);
    ret = true;
  }
  return ret;
}
/* --
 * 统一的平移量
 * 统一的旋转量
 * 不同的平移量
 */
void VmapData::getTransVectorMap(double detax, double detay, double detaz) {
  int index, num;
  for (int type_id = 0; type_id < Type_Num; type_id++) {
    index = getCurrentTypeIdNumber(type_id);
    if (index > 0) {
      for (int i = 0; i < index; i++)  // type_num_id
      {
        num = vmap_[type_id].at(i).wp.size();
        for (int j = 0; j < num; j++) {
          vmap_[type_id].at(i).wp.at(j).point.x =
              vmap_[type_id].at(i).wp.at(j).point.x + detax;
          vmap_[type_id].at(i).wp.at(j).point.y =
              vmap_[type_id].at(i).wp.at(j).point.y + detay;
          vmap_[type_id].at(i).wp.at(j).point.z =
              vmap_[type_id].at(i).wp.at(j).point.z + detaz;
          double latitude, longitude;
          map_projection_reproject(
              &origin_pos_, vmap_[type_id].at(i).wp.at(j).point.y,
              vmap_[type_id].at(i).wp.at(j).point.x, &latitude, &longitude);
          printf("latitude=%f, longitude=%f\n", latitude, longitude);
          vmap_[type_id].at(i).wp.at(j).satfix.latitude = latitude;
          vmap_[type_id].at(i).wp.at(j).satfix.longitude = longitude;
          vmap_[type_id].at(i).wp.at(j).satfix.altitude =
              vmap_[type_id].at(i).wp.at(j).point.z;
        }
      }
    }
  }
}
void VmapData::getRotateVectorMap(double sitaz) {
  double yaw = sitaz * M_PI / 180;
  Eigen::AngleAxisd t_V(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d t_R = t_V.matrix();
  int index;
  int num;
  //#pragma omp parallel for
  for (int type_id = 0; type_id < Type_Num; type_id++) {
    index = getCurrentTypeIdNumber(type_id);
    if (index > 0) {
      //#pragma omp parallel for
      for (int i = 0; i < index; i++)  // type_num_id
      {
        num = vmap_[type_id].at(i).wp.size();
        //#pragma omp parallel for
        for (int j = 0; j < num; j++) {
          Eigen::Vector3d p_1(vmap_[type_id].at(i).wp.at(j).point.x,
                              vmap_[type_id].at(i).wp.at(j).point.y,
                              vmap_[type_id].at(i).wp.at(j).point.z);
          Eigen::Vector3d p_3;
          p_3 = t_R * p_1;
          vmap_[type_id].at(i).wp.at(j).point.x = p_3[0];
          vmap_[type_id].at(i).wp.at(j).point.y = p_3[1];
          double latitude, longitude;
          map_projection_reproject(
              &origin_pos_, vmap_[type_id].at(i).wp.at(j).point.y,
              vmap_[type_id].at(i).wp.at(j).point.x, &latitude, &longitude);
          printf("latitude=%f, longitude=%f\n", latitude, longitude);
          vmap_[type_id].at(i).wp.at(j).satfix.latitude = latitude;
          vmap_[type_id].at(i).wp.at(j).satfix.longitude = longitude;
          vmap_[type_id].at(i).wp.at(j).satfix.altitude =
              vmap_[type_id].at(i).wp.at(j).point.z;
        }
      }
    }
  }
}

/* --
 * des:设置当前索引index指向的点
 * input:type_id(类型ID),type_num_id(编号ID),point_index(点索引),point(设置点数据)
 * output:false(失败),true(成功)
 */
bool VmapData::setCurrentTypeNumIdPoint(const int type_id,
                                        const int type_num_id,
                                        const int point_index,
                                        const Way_Point_ &w_p) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    if (point_index >= 0 && point_index < num) {
      vmap_[type_id].at(index).wp.at(point_index).property = w_p.property;
      vmap_[type_id].at(index).wp.at(point_index).ltype = w_p.ltype;
      vmap_[type_id].at(index).wp.at(point_index).limit_vel = w_p.limit_vel;
      vmap_[type_id].at(index).wp.at(point_index).lanelet_id = w_p.lanelet_id;

      vmap_[type_id].at(index).wp.at(point_index).point.x = w_p.point.x;
      vmap_[type_id].at(index).wp.at(point_index).point.y = w_p.point.y;
      vmap_[type_id].at(index).wp.at(point_index).point.z = w_p.point.z;
      vmap_[type_id].at(index).wp.at(point_index).point.quate_x =
          w_p.point.quate_x;
      vmap_[type_id].at(index).wp.at(point_index).point.quate_y =
          w_p.point.quate_y;
      vmap_[type_id].at(index).wp.at(point_index).point.quate_z =
          w_p.point.quate_z;
      vmap_[type_id].at(index).wp.at(point_index).point.quate_w =
          w_p.point.quate_w;
      vmap_[type_id].at(index).wp.at(point_index).satfix.latitude =
          w_p.satfix.latitude;
      vmap_[type_id].at(index).wp.at(point_index).satfix.longitude =
          w_p.satfix.longitude;
      vmap_[type_id].at(index).wp.at(point_index).satfix.altitude =
          w_p.satfix.altitude;
      ret = true;
    }
  }
  return ret;
}
//查找最近点---
std::tuple<int, int> VmapData::findCurrentNearPointNumber(int type_id,
                                                          point_ point) {
  int index = -1;
  int typeNumId = -1;
  double len = std::numeric_limits<double>::max();
  unsigned int i = 0;
  for (auto vmap : vmap_[type_id % Vm_T::Type_Num]) {
    i = 0;
    for (auto waypoint : vmap.wp) {
      i++;
      double l = pow((point.x - waypoint.point.x), 2) +
                 pow((point.y - waypoint.point.y), 2);
      if (len > l) {
        len = l;
        typeNumId = vmap.type_num_id;
        index = i;
      }
    }
  }
  printf("index = %d,typeNumId = %d\n", index, typeNumId);
  return std::make_tuple(index, typeNumId);
}

bool VmapData::insertFrontTypeNumIdPoint(const int type_id,
                                         const int type_num_id,
                                         int &point_index,
                                         const Way_Point_ &w_p) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    if (point_index >= 0 && point_index < num) {
      vmap_[type_id].at(index).wp.insert(
          vmap_[type_id].at(index).wp.begin() + point_index + 1, w_p);
      point_index++;
      ret = true;
    }
  }
  return ret;
}
bool VmapData::insertBackTypeNumIdPoint(const int type_id,
                                        const int type_num_id, int &point_index,
                                        const Way_Point_ &w_p) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    if (point_index >= 0 && point_index < num) {
      vmap_[type_id].at(index).wp.insert(
          vmap_[type_id].at(index).wp.begin() + point_index, w_p);
      ret = true;
    }
  }
  return ret;
}
bool VmapData::delCurrentTypeNumIdPoint(const int type_id,
                                        const int type_num_id,
                                        const int point_index) {
  bool ret = false;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    if (point_index >= 0 && point_index < num) {
      vmap_[type_id].at(index).wp.erase(vmap_[type_id].at(index).wp.begin() +
                                        point_index);
      ret = true;
    }
  }
  return ret;
}
bool VmapData::delCurrentTender() {
  bool ret = false;
  int type_id = 8;
  if (!vmap_[type_id].empty()) {
    for (auto p : vmap_[type_id]) {
      if (!p.wp.empty()) {
        p.wp.clear();
      }
    }
    vmap_[type_id].clear();
    ret = true;
    // std::cout<<"tender clear ok"<<std::endl;
  }
  return ret;
}

void VmapData::getNewlineFromcurrent(const int type_id, const int type_num_id,
                                     const int point_index) {
  int newTypenum = getRandEmptyTypeNumId(type_id);
  VectorMap_ vectormap;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    if (num < 2) return;
    if (point_index >= 0 && point_index < num) {
      for (int i = point_index; i < num; i++) {
        vectormap.wp.push_back(
            *(vmap_[type_id].at(index).wp.end() - 1 - (num - i - 1)));
        if (i == point_index) continue;
        vmap_[type_id].at(index).wp.erase(vmap_[type_id].at(index).wp.end() -
                                          1 - (num - i - 1));
      }
    }
  }
  vectormap.type_id = type_id;  //矢量地图对应的类型，如lane,roadedge,sigald等
  vectormap.type_num_id = newTypenum;  //编号
  setCurrentVectorMap(type_id, newTypenum, vectormap);
}

void VmapData::findVectormapswithVectormap(
    std::vector<VectorMap_> vectormap_vec, point_ point, int type_id,
    int type_num_id) {
  //点--》图
  VectorMap_ vectormap;
  if (getCurrentVectorMap(type_id, type_num_id, vectormap)) {
    vectormap_vec.push_back(vectormap);
  }
  // for lines neighbor line
  findVectormapswithPoints(vectormap_vec, point);
}
void VmapData::findVectormapswithPoints(std::vector<VectorMap_> vectormap_vec,
                                        point_ point) {
  std::vector<VectorMap_> vectormaps_nei;
  for (auto vectormap : vectormap_vec) {
    int type_num_id = vectormap.type_num_id;
    int type_id = vectormap.type_id;
    for (auto p : vectormap.wp) {
      int index = -1;
      int typeNumId = -1;
      std::tie(index, typeNumId) = findCurrentNearPointNumber(type_id, p.point);
      if (typeNumId != type_num_id) {
        // for point of different line
        Way_Point_ w_p;
        if (getCurrentTypeNumIdPoint(type_id, typeNumId, index - 1, w_p)) {
          double dis = pow((p.point.x - w_p.point.x), 2) +
                       pow((p.point.y - w_p.point.y), 2);
          if (dis < 2) {
            std::cout << "get neighbor project" << typeNumId << std::endl;
            VectorMap_ vectormap_nei;
            if (getCurrentVectorMap(type_id, typeNumId, vectormap_nei)) {
              vectormaps_nei.push_back(vectormap_nei);
            }
          }
        }
      }
    }
  }
  vectormap_vec.insert(vectormap_vec.end(), vectormaps_nei.begin(),
                       vectormaps_nei.end());
  sort(
      vectormap_vec.begin(), vectormap_vec.end(),
      [](VectorMap_ a, VectorMap_ b) { return a.type_num_id < b.type_num_id; });
  vectormap_vec.erase(unique(vectormap_vec.begin(), vectormap_vec.end(),
                             [](VectorMap_ a, VectorMap_ b) {
                               return a.type_num_id == b.type_num_id;
                             }),
                      vectormap_vec.end());
  // TO DO 是否循环的条件
}
bool VmapData::isNeedClean(int property) {
  Way_Point_ waypoint;
  waypoint.property = property;
  PRO_CLEARAREA_TEST test(waypoint.property_type.P1_TYPE);
  if (test.pro_type.TYPE1 == TYPE_P1_TYPE_CLEARAREA::CLEARAREA_TYPE)
    return true;
  else {
    return false;
  }
}

/*判断清扫去是否交叉
输入：正在创建的点
输出：交叉提示，取消则撤回操作，确定默认创建
方法：当创建的清扫区点大于4的时候，判断所有点的位置关系
*/
bool VmapData::isIntersectClearArea(const int type_id,
                                    const int type_num_id){
  bool ret = false;
  Way_Point_ point_o,point_o2;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();

    std::cout << "索引：" << index << " 总数：" << num << " 个数：" << type_num_id << std::endl;
    if (num > 3) {
      Way_Point_ point_e = vmap_[type_id].at(index).wp.back();
      Way_Point_ point_e2 = vmap_[type_id].at(index).wp.at(num -2);
      for (size_t i = 1; i < num-2; i++)
      {
        point_o = vmap_[type_id].at(index).wp[i];
        point_o2 = vmap_[type_id].at(index).wp[i-1];

        ret = isINterserctString(point_e,point_e2,point_o,point_o2);
        if (ret)
        {
          break;
        }  
      }
    }
  }
  return ret;
}
bool VmapData::isIntersectClearArea(const int type_id, const int type_num_id,
                                    const Way_Point_ &wp, const int current_point_index){
  bool ret = false;
  Way_Point_ point_o,point_o2,point_e2;
  int index = seachIdVector(type_num_id, vmap_[type_id]);
  if (index >= 0) {  //存在
    int num = vmap_[type_id].at(index).wp.size();
    std::cout << "索引：" << index << " 总数：" << num << " 个数：" << type_num_id  << "当前选中的点： " << current_point_index << std::endl;
    if (num > 3) {
      Way_Point_ point_e = wp;
      if (current_point_index > 0) {
        point_e2 = vmap_[type_id].at(index).wp.at(current_point_index - 1);
      } else {
        point_e2 = vmap_[type_id].at(index).wp.at(current_point_index + 1);
      }
      for (size_t i = 1; i < num-2; i++)
      {
        point_o = vmap_[type_id].at(index).wp[i];
        point_o2 = vmap_[type_id].at(index).wp[i-1];
        if (point_o.point.x == point_e.point.x || point_o.point.y == point_e.point.y ||
            point_o.point.x == point_e2.point.x || point_o.point.y == point_e2.point.y ||
            point_o2.point.x == point_e.point.x || point_o2.point.y == point_e.point.y ||
            point_o2.point.x == point_e2.point.x || point_o2.point.y == point_e2.point.y)
        {
          i = i + 1;
          break;
        }
        ret = isINterserctString(point_e,point_e2,point_o,point_o2);
        if (ret)
        {
          break;
        }  
      }
    }
  }
  return ret;
}

bool VmapData::isINterserctString(const Way_Point_ &point_e, const Way_Point_ &point_e2, 
                                  const Way_Point_ &point_o, const Way_Point_ &point_o2) {
  bool ret = false;
  Function func;
  point_ point_in;
  func.D2_value = point_e.point.x - point_e2.point.x;
  func.k_2 = (point_e.point.y - point_e2.point.y)/func.D2_value;
  func.b_2 = (point_e.point.x*point_e2.point.y - point_e2.point.x*point_e.point.y)/func.D2_value;
  
  func.D1_value = point_o2.point.x - point_o.point.x;         
  func.D1_vertical = point_o2.point.y - point_o.point.y;

  func.k_1 = (point_o2.point.y - point_o.point.y) / func.D1_value;
  func.b_1 = (point_o2.point.x * point_o.point.y - point_o.point.x * point_o2.point.y) / func.D1_value;
  
  point_in.x = (func.b_2 - func.b_1) / (func.k_1 - func.k_2);
  point_in.y = (func.k_2 * func.b_1 - func.k_1 * func.b_2) / (func.k_2 - func.k_1);
  
  if ((point_in.x > point_e.point.x && point_in.x > point_e2.point.x) || (point_in.x < point_e.point.x && point_in.x < point_e2.point.x) ||
      (point_in.y > point_e.point.y && point_in.y > point_e2.point.y) || (point_in.y < point_e.point.y && point_in.y < point_e2.point.y) ||
      (point_in.x > point_o2.point.x && point_in.x > point_o.point.x) || (point_in.x < point_o2.point.x && point_in.x < point_o.point.x) ||
      (point_in.y > point_o2.point.y && point_in.y > point_o.point.y) || (point_in.y < point_o2.point.y && point_in.y < point_o.point.y))     
  {
      // std::cout << "no intersect" << std::endl;
      ret = false;
  }
  else //存在相交，退出循环
  {
      // std::cout << "intersect" << std::endl;
      ret = true;
      return ret;
  }
  return ret;
}

/*断开清扫区
输入：两个点
输出：根据所找到的清扫区上的两点，把该清扫区分为两块．
方法：修改原来清扫区，并新建一个清扫区
*/
// void VmapData::cutClearArea(std::vector<geometry_msgs::Pose> &poses){
//   int type_id = Vm_T::ClearArea;
//   int index1 = -1;
//   int index2 = -1;
//   int clear_id1 = -1;
//   int clear_id2 = -1;
//   point_ point;
//   point.x=poses[0].position.x;
//   point.y=poses[0].position.y;
//   point.z=poses[0].position.z;
//   std::tie(index1, clear_id1) =
//       findCurrentNearPointNumber(type_id, point);
//   point.x=poses[1].position.x;
//   point.y=poses[1].position.y;
//   point.z=poses[1].position.z;
//   std::tie(index2, clear_id2) =
//       findCurrentNearPointNumber(type_id, point);
//   if(index1>index2){
//     int tmp=index1;
//     index1 = index2;
//     index2 = tmp;
//   }
//   std::cout << "即将断开的清扫区序号： " << clear_id2 << "和" << clear_id1 << std::endl;
//   if(clear_id1!=clear_id2 || index1==index2 || clear_id1==-1 || clear_id2==-1){
//     poses.clear();
//     return;
//   }
//   else{
//     int new_numberid=getRandEmptyTypeNumId(type_id);
//     VectorMap_ new_vectormap;
//     new_vectormap.type_id=type_id;
//     new_vectormap.type_num_id =new_numberid;
//     // std::cout<<"..index1 ="<<index1<<std::endl;
//     // std::cout<<"..index2 ="<<index2<<std::endl;
//     for (auto &vmap : vmap_[type_id % Vm_T::Type_Num]) {
//       if(vmap.type_num_id==clear_id2){
//         new_vectormap.wp.insert(new_vectormap.wp.end(), vmap.wp.begin()+index1-1, vmap.wp.begin()+index2+1);
//         vmap.wp.erase(vmap.wp.begin()+index1,vmap.wp.begin()+index2);
//         // std::cout<<"..vmap.wp.size ="<<vmap.wp.size()<<std::endl;
//         // std::cout<<"..new_vectormap.wp.size ="<<new_vectormap.wp.size()<<std::endl;
//         break;
//       }
//     }
//     vmap_[type_id].push_back(new_vectormap);
//     poses.clear();
//   }
// }
