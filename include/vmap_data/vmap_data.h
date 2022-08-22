#ifndef VMAP_DATA_H
#define VMAP_DATA_H

//所需要包含的头文件
// #include <geometry_msgs/PolygonStamped.h>
// #include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <tf/tf.h>

#include <Eigen/Dense>
#include <QDebug>
#include <QFile>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <iostream>
#include <fstream>
#include <json/json.h>
#include <limits>
#include <tuple>
#include <vector>

#include "geo/geo.h"
#include "vmap_data/datatype.h"
#include "map_data/mapdata_format.h"

enum DockFile : int {
    Dock_id = 0,
    Dock_name,
    Location_lat,
    Location_lon,
    Location_alt,
    Dock_direct,
    Dock_open,
    Dock_type,
    Dock_Num
};

struct Function
{
    double D1_vertical;
    double D1_value;
    double D2_value;
    double k_1;
    double k_2;
    double b_1;
    double b_2;
};

class VmapData {
 public:
  VmapData();
  ~VmapData();
  void clearAll();
  void readVmapFiles(QStringList &fileNames);
  std::map<int, VectorMap_> getPurgeArrowInfo();
  void readGidFile(QString &path_str, QList<QMap<QString,QList<QString>>> &lists);
  std::vector<Way_Point_> jsonGet(std::string fileStr);
  void readNewestDockFile(QString &path_str, 
                          std::vector<std::pair<std::string, std::vector<double>>> &uploadDockInfos);
  bool setCurrentVectorMap(const int type_id, const int type_num_id,
                           const VectorMap_ &vectormap);
  int seachIdVector(const int id, const std::vector<VectorMap_> &vm);
  void writeVmapFiles(std::string &save_path);
  void writeVmapFile(QString &path_str, int &type_id);
  std::string writeVmapJson(std::string projectname, std::string &path_str, int &type_id, std::string blockId);
  void writeGidFile(QString &path_str, QList<QMap<QString,QList<QString>>> &lists);
  std::vector<VectorMap_> &getVectorType(const Vm_T type_id);
  std::vector<VectorMap_> &getCurrentMap();
  bool getCurrentTypeNumIdPoint(const int type_id, const int type_num_id,
                                const int point_index, Way_Point_ &w_p);
  bool getCurrentVectorMap(const int type_id, const int type_num_id,
                           VectorMap_ &vectormap);
  int getCurrentTypeNumIdNumber(const int type_id, const int type_num_id);
  std::tuple<int, int> findCurrentNearPointNumber(int type_id, point_ point);
  bool setCurrentTypeNumIdPoint(const int type_id, const int type_num_id,
                                const int point_index, const Way_Point_ &w_p);
  bool getRandActiveId(int &type_id, int &type_num_id);
  int getRandActiveTypeNumId(const int &type_id);
  // function buntion
  bool insertFrontTypeNumIdPoint(const int type_id, const int type_num_id,
                                 int &point_index, const Way_Point_ &w_p);
  bool insertBackTypeNumIdPoint(const int type_id, const int type_num_id,
                                int &point_index, const Way_Point_ &w_p);
  bool delCurrentTypeNumIdPoint(const int type_id, const int type_num_id,
                                const int point_index);
  void getNewlineFromcurrent(const int type_id, const int type_num_id,
                             const int point_index);
  bool setTypeNumIdDelPoints(const int type_id, const int type_num_id);

//   double getValueZFromTender(std::vector<point_> tender_points,
//                              geometry_msgs::Pose pose);
  int getRandEmptyTypeNumId(const int &type_id);
  int getCurrentTypeIdNumber(const int type_id);
  bool setTypeNumIdDelBackPoint(const int type_id, const int type_num_id);
  bool delCurrentTender();
  // trans
  void getRotateVectorMap(double sitaz);
  void getTransVectorMap(double detax, double detay, double detaz);

  // for projector to image
  void findVectormapswithVectormap(std::vector<VectorMap_> vectormap_vec,
                                   point_ point, int type_id, int type_num_id);
  void findVectormapswithPoints(std::vector<VectorMap_> vectormap_vec,
                                point_ point);
  // for debug
//   void getClearAreasFill(
//       std::vector<geometry_msgs::PolygonStamped> &clearAreas,
//       std::vector<geometry_msgs::PolygonStamped> &clearAreasFill);
  bool isNeedClean(int property);
//   void getAttributeAreasFill(
//       std::vector<geometry_msgs::PolygonStamped> &clearAreas,
//       std::vector<geometry_msgs::PolygonStamped> &clearAreasFill);
//   bool isNeedAttribute(int property);
//   bool insidePolygon(geometry_msgs::Pose pose,
//                      geometry_msgs::PolygonStamped polygonStamped);
//   bool getNearPolygon(geometry_msgs::Pose pose,
//                   std::vector<geometry_msgs::PolygonStamped> &polygons,
//                   std::vector<geometry_msgs::PolygonStamped> polygonsFill,
//                   geometry_msgs::PolygonStamped &polygonStamped);
  bool isIntersectClearArea(const int type_id, const int type_num_id);
  bool isIntersectClearArea(const int type_id, const int type_num_id,
                            const Way_Point_ &wp, const int current_point_index);
  bool isINterserctString(const Way_Point_ &point_e, const Way_Point_ &point_e2, 
                          const Way_Point_ &point_o, const Way_Point_ &point_o2);
//   void cutClearArea(std::vector<geometry_msgs::Pose> &poses);
//--
  void setCurrentPurgeArrow(const int type_id, const int type_num_id, 
                            std::map<int, VectorMap_> &purge_arrows);

 public:
  map_projection_reference_s origin_pos_;
  MapDataFormat mapdataformat_;

 private:
  //--数据存贮
  std::vector<VectorMap_> vmap_[Type_Num];
  std::vector<VectorMap_> vectormap_;
  std::map<int, VectorMap_> purge_arrows_;
  // UploadAreas areaupload_;
};

#endif  // TELEOP_PANEL_H
