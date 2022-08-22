#ifndef MAP_WIDGET_H
#define MAP_WIDGET_H

#include <QMessageBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QDialog>
#include <QSettings>
#include <QKeyEvent>
#include <QFileDialog>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdlib.h>

#include "render_panel/render_panel.h"
#include "lanelet_data/lanelet2_data.h"
#include "map_data/pointcloudmap.h"
#include "vmap_data/vmap_data.h"
#include "tool/Tools.h"
#include "docks_vehicle_data/dock_rest_requests.h"

enum Color : int {
  BLACK,
  GRAY,
  LIGHT_RED,
  LIGHT_GREEN,
  LIGHT_BLUE,
  LIGHT_YELLOW,
  LIGHT_CYAN,
  LIGHT_MAGENTA,
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  WHITE,
  Null = 200
};

struct PurgeArrow_
{
  std::vector<Way_Point_> wps;
  std::vector<Ogre::Quaternion> quats;
};

const double COLOR_VALUE_MIN = 0.0;
const double COLOR_VALUE_MAX = 1.0;
const double COLOR_VALUE_MEDIAN = 0.5;
const double COLOR_VALUE_LIGHT_LOW = 0.56;
const double COLOR_VALUE_LIGHT_HIGH = 0.93;
double const DOUBLE_MAX = std::numeric_limits<double>::max();
double const DOUBLE_MIN = std::numeric_limits<double>::min();
float const FLOAT_MIN = std::numeric_limits<float>::min();
const std::string HOME_STR = std::string(std::getenv("HOME"));

class MapWidget : public QWidget{
  Q_OBJECT
 public:
  MapWidget();
  ~MapWidget();

  //renderpanel/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MyRenderPanel *getRenderPanel();
  void setDispalyMode(MyRenderPanel::DisplayMode mode);
  void setWorkMode(MyRenderPanel::WorkMode mode);
  void setPcIntensityModeDisplay(bool mode_flag, double min_value, double max_value, std::string id);
  void setPcIntensityMode(bool mode_flag,double min_value, double max_value);
  void setPointCloudSize(const std::string id, float pointsize);
  void setLineSize(double linesize);
  void setGridColor(const Ogre::ColourValue& color);
  void setGridSize(uint32_t gridsize);
  void setGridCellLength(float len);
  void setGridLineWidth(float width);
  void seceneObjectRemove();
  void seceneMapRemove();
  void seceneObjectVisible(bool visible);
  void seceneMapVisible(bool visible);
  void setMapZmin(double zvalue);
  void setCameraZero();
  Ogre::Vector3 getFocalPoint();
  Ogre::ColourValue getGridColor();
  float getPitch();
  float getYaw();
  float getDistance();  
  float getGridSize();
  float getGridCellLength();
  float getGridLineWidth();
  int getFps();

  std::tuple<double, double> getPushPoint();
  void loadFramePcd(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

  //pointcloud/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void loadPcdMap(const std::string &id);
  void loadBlockMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  bool initMapOriginPostion(std::vector<std::string> &pcd_paths);
  bool initMapOriginPostion(std::string &pcd_path);
  bool initMapOriginPostion(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, Eigen::Vector4f &origin, Eigen::Quaternionf &orientation);
  void reloadPcdMap(double point_zmin, double point_zmax);
  void getPcdmapRangeZ(double &zmin, double &zmax);
  bool loadPcd(std::string pcd_path);
  double getPcdZmin();
  double getPcdZmax();
  float getPcdSize();

  // vmap data/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void loadVmap(QStringList vmap_fileNames);
 void setCurrentVectorMap(const int type_id, const int type_num_id,
                          const VectorMap_ &vectormap);
 int getRandEmptyTypeNumId(const int type_id);
 bool getCurrentVectorMap(const int type_id, const int type_num_id,
                          VectorMap_ &vectormap);
 void writeVmapFile(QString &path_str, int &type_id);
 void writeVmapFiles(std::string &save_path);
 std::string writeVmapJson(std::string &name, std::string &path_str, int &type_id, std::string blockId);
 void writeGidFile(QString &path_str, QList<QMap<QString,QList<QString>>> &lists);
 std::vector<VectorMap_> &getVectorType(const Vm_T type_id);
 void localCoordinatesToGps(Way_Point_ &wp);
 void gpsCoordinatesToLocal(Way_Point_ &wp);
 bool getCurrentTypeNumIdPoint(Way_Point_ &wp);
 void updateShowVMapData(int t_typeid, int t_typenumid, int t_current_index, bool viewflag);
 void updateShowVMapData(bool viewflag);
 bool getCurrentTypeNumIdPoint(const int type_id, const int type_num_id,
                               const int point_index, Way_Point_ &w_p);
 void setCurrentPurgeArrow(const int type_id, const int type_num_id, 
                           std::map<int, VectorMap_> &purge_arrows);
 std::map<int, VectorMap_> getPurgeArrowInfo();
                               
//view vmap data/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void getCurrentVmapElementId(int t_typeid, int t_typenumid,
                              int t_current_index);
 void createCurrentPointViewer(Way_Point_ wp, Color color);
 void createBindPointViewer(std::vector<point_> points, Color color);
 void getEqualPoint(std::vector<VectorMap_> vectormap, point_ pt,
                    int ven, int wpn);
 void viewerVmapPoint(std::vector<VectorMap_> vectormap, Color color, bool viewflag);
 void viewerVmapLine(std::vector<VectorMap_> vectormap, Color color, bool viewflag);
 Ogre::ColourValue createColorRGBA(Color color);

 int getCurrentTypeNumIdNumber(const int type_id, const int type_num_id);
 std::tuple<int, int> findCurrentNearPointNumber(int type_id, point_ point);
 bool setCurrentTypeNumIdPoint(const int type_id, const int type_num_id,
                               const int point_index, const Way_Point_ &w_p);
 bool getRandActiveId(int &type_id, int &type_num_id);
 int getRandActiveTypeNumId(const int &type_id);
 bool insertFrontTypeNumIdPoint(const int type_id, const int type_num_id,
                                const Way_Point_ &w_p);
 bool insertBackTypeNumIdPoint(const int type_id, const int type_num_id,
                               const Way_Point_ &w_p);
 bool delCurrentTypeNumIdPoint(const int type_id, const int type_num_id);
 void getNewlineFromCurrentPoint(const int type_id, const int type_num_id);
 void clearAllVmap();
 bool setTypeNumIdDelBackVmapPoint(const int type_id, const int type_num_id);
 bool setTypeNumIdDelVmapPoints(const int type_id, const int type_num_id);
 bool setClearareaIntersect(const int type_id, const int type_num_id);
 bool setClearareaIntersect(const int type_id,
                            const int type_num_id,
                            const Way_Point_ &wp);
 bool findNearPoint(int type_id, point_ &find_point, point_ &point,
                    double distance);
  // debug cleararea
  // lanelet viewer/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void getCurrentLaneletMapElementId(int point3d_id, std::vector<int> ele_lanelet_ids);
  void LaneletMapPointViewer(const lanelet::ConstPoints3d &points,
                             Color color);
  void LaneletMapLinestringViewer(const lanelet::ConstLineStrings3d &linestrings, Color color);
  void LaneletMapLaneletViewer(const lanelet::ConstLanelets lanelets, Color color);
  void LaneletMapPolygonViewer(lanelet::ConstPolygons3d &polygons,
                               Color color);
  void creatLaneletTextViewer(const lanelet::ConstLanelet &lanelet,
                              Ogre::ColourValue color);
  void updateShowLaneletMapData(int current_pt, std::vector<int> ids);
  // lanelet map///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool laneletmapEmpty();
  void moveFrontPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                        lanelet::LineString3d linestring);
  void moveFrontPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                        lanelet::Polygon3d poly);
  void moveBackPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                       lanelet::Polygon3d poly);
  void moveBackPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                       lanelet::LineString3d linestring);
  void insertFrontPoint3d(lanelet::Point3d point,
                          lanelet::LineString3d &linestring,
                          lanelet::Point3d &new_point3d);
  void insertFrontPoint3d(lanelet::Point3d point, lanelet::Polygon3d &poly,
                          lanelet::Point3d &new_point3d);
  void insertBackPoint3d(lanelet::Point3d point,
                         lanelet::LineString3d &linestring,
                         lanelet::Point3d &new_point3d);
  void insertBackPoint3d(lanelet::Point3d point, lanelet::Polygon3d &poly,
                         lanelet::Point3d &new_point3d);
  void loadLaneletMap(QString lanelet_path);
  void saveLaneletMap(std::string lanelet_path);
  lanelet::Lanelet createRoadLanelet(
      std::vector<lanelet::Point3d> &point3d_vec);
  lanelet::Lanelet createCrosswalkLanelet(
      std::vector<lanelet::Point3d> &point3d_vec);
  void extendLanelet(lanelet::Lanelet &Lanelet, lanelet::Point3d &point3d);
  void extendLanelet(int lanelet_id, lanelet::Point3d &point3d);
  lanelet::LineString3d createRoadLinestring(
      std::vector<lanelet::Point3d> &point3d_vec);
  lanelet::LineString3d createStopline(
      std::vector<lanelet::Point3d> &point3d_vec);
  lanelet::LineString3d createBump(std::vector<lanelet::Point3d> &point3d_vec);
  void extendLinestring(int linestring_id, lanelet::Point3d &point3d);
  lanelet::Polygon3d createIntersection(
      std::vector<lanelet::Point3d> &point3d_vec);
  void extendIntersection(int poly_id, lanelet::Point3d &point3d);
  void invertLanelet();
  lanelet::Point3d searchNearstPoint3d(Way_Point_ pose);
  lanelet::LineString3d searchNearstLinestring(Way_Point_ pose);
  lanelet::Lanelet searchNearstLanelet(Way_Point_ pose);
  lanelet::Polygon3d searchNearstPolygon(Way_Point_ pose);
  bool findPoint3dWithId(int &point3d_id, lanelet::Point3d &point3d);
  bool isPoint3dinLinestringLayer(int point3d_id);
  bool isPoint3dinLinestring(int point3d_id, lanelet::LineString3d ll);
  bool isPoint3dinPoly(int point3d_id, lanelet::Polygon3d pl);
  bool findLinestringWithId(int &linestring_id,
                            lanelet::LineString3d &linestring);
  bool findLaneletWithId(int &lanelet_id, lanelet::Lanelet &lanelet);
  bool findPoly3dWithId(int &poly3d_id, lanelet::Polygon3d &polygon3d);
  void resetPoint3d(lanelet::Point3d &point3d);
  void rmPoint3d(lanelet::Point3d &pt);
  void rmChooseLanelet2Ids();
  int addLanelet();
  void jointLanelet();
  bool addregular(int &regular_id, reguinfo_list &reg_list);
  void getNewlineFromCurrentPoint();
  bool getLaneletCurrentTypeBind(point_ &point);
  bool getLaneletCurrentTypeBind();
  void clearAllLanelet2map();
  void gpsCoordinatesToLocal(LatLonInfo latlon_info, lanelet::Point3d &point);
  std::vector<reguinfo_list> readLaneletMapRegular();
  void clearRegelem();
  void writeLaneletMapRegular(std::vector<reguinfo_list> &regular);
  std::vector<tag_list> readLaneletMapCustomizeTags();
  void writeLaneletMapCustomizeTags(std::vector<tag_list> tags_vec);
  void initRegularInfoList(reguinfo_list &reg_list, int regular_id,
                           int lanelet_id, int member_id,
                           std::string member_str);
  bool haveCurrentRefIdRole(int ref_id, std::string &ref_role);
  std::vector<point_> geteachLinestringsPoints(lanelet::LineString3d &line);
  void findLinestringswithPoint3d(lanelet::LineStrings3d &linestrings,
                                  lanelet::Point3d point);
  std::vector<int> findNextLanelet(lanelet::Lanelet &Lanelet);
  void deletRegularInfo(int regul_id);
  void addRegularInfo(reguinfo_list &reg_list);
  std::vector<reguinfo_list> getRegularInfoList();
  std::vector<tag_list> getTagInfoList();
  void createNewRegularInfoList(std::vector<reguinfo_list> &regularlist);
  void createNewTagInfoList(std::vector<tag_list> &taglist);
  // task control////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void saveCsvMapFiles(int id);
  std::string saveCsvJsonFiles(std::string name, int id, std::string blockId);
  void saveOsmMapFiles();
  void saveGidFiles(QList<QMap<QString,QList<QString>>> &lists);
  void autoSaveFile(std::string &path);
  QString readOnlyPcdMapFile(QString &id,std::vector<std::string> &paths);
  QString readVmapFile(QStringList &pronames);
  QString readAllVmapFile(QStringList &fileNames, QList<QMap<QString,QList<QString>>> &lists);
  void readGidFile(QString &str, QList<QMap<QString,QList<QString>>> &lists);
  QString filePath(QString file);
  void loadReguandTagInfo();
  void ReadSettings(void);
  void WriteSettings(void);

  // docks////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void readNewestDockFile(QString &path_str,
                          std::vector<std::pair<std::string, std::vector<double>>> &uploadDockInfos);
  bool uploadDockById(std::vector<std::pair<std::string, std::vector<double>>> uploadDockInfos);
  bool getdockViewInfo(std::string &mapname);
  std::string getBlockId();
  int findCurrentNearDocksNumber(point_ point, double distance);
  void viewerDocksPoint(std::vector<MapDockInfo> &Res_docks, Color color);
  void udpateShowDocksData();
  void seceneDocksRemove();
  void seceneDocksVisible(bool visible);
  std::vector<MapDockInfo> docksViewData();
  void setDockRequestDomain(std::string domain);

  //vehicle///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::set<std::string> getCurrentRobotId(QString projectname);
  bool getVehicleViewInfo(QString projectname,std::set<std::string> currentrobot);
  void viewerVehicleData(std::vector<MapInfo> &vehicle_info, 
                         std::string &robotname, Color color);
  void updateShowVehicleData(std::set<std::string> &currentrobot);
  void chooseVehicleView(std::vector<MapInfo> &vehicle_info, 
                         std::set<std::string> &currentrobot, Color color);
  void closeMysqlVehical();
  std::unordered_map<std::string, std::vector<std::string>> getVehicleRobotName();
  void mapDataGetFromMysql(std::vector<MapData> &vec_mapdata);
  void mapDataUpdateFromMysql(MapData &mapdata);
  void mapDataIncreaseFromMysql(MapData &mapdata);
  void mapDataDeleteFromMysql(MapData &mapdata);

  //slam-data keyframe////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool get_keyframes(const std::string& directory, std::deque<KeyFrame> &key_frames);
  double findCurrentNearPointHeight(Way_Point_ &pose, double distance, std::deque<KeyFrame> &key_frames);
  double findCurrentNearPointHeight(Way_Point_ &pose, double distance, std::vector<Way_Point_> &keyframes_poses);
  void get_QuaterAndVector(const QString& directory, 
                           boost::optional<Eigen::Vector3d> &ZERO_GNSS);
  bool savePointCloudPcdFile(QString &project_name,
                             std::deque<KeyFrame> &key_frames,
                             boost::optional<Eigen::Vector3d> &map_zero_gnss,
                             bool downsample_flag,
                             double downsample_resolution, 
                             bool saveCompressedPCD);
  bool writeKeyFramesFiles(QString &path_str, std::vector<Way_Point_> &keyframes_poses);
  QString readKeyFramesFiles(QString &file_Path, std::vector<Way_Point_> &keyframes_poses, int &num);
  // void get_QuaterAndVector(const QString& directory, Eigen::Vector4f &origin, Eigen::Quaternionf &orientation);
  // bool save_pointcloud(const std::string &filename,
  //                               boost::optional<Eigen::Vector3d> &map_zero_gnss,
  //                               bool downsample_flag,
  //                               double downsample_resolution, 
  //                               bool saveCompressedPCD /* = true*/)
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////'
  void setPlatformRequestDomain(std::string domain);
  void setPlatformRequesauthorizationToken(std::string authorizationToken);
  std::string pullParkToken(std::string &user, std::string &password, std::string &keyDir);
  QList<QMap<QString, QString>> pullByToken(std::string authorizationToken);
  void post_initia_dock(std::string &map_id, 
                        Eigen::Matrix4d pose, 
                        std::string authorizationToken);
  void postUploadAllData(std::string authorizationToken, std::string uploadArea);
  QList<QMap<int, QString>> getSignalDataIdFromParkId(std::string authorizationToken,
                                                      std::string parkId, 
                                                      std::string dataType);
  void deleteAllData(std::string authorizationToken, 
                      std::string dataType, 
                      std::string parkId);
  void odinaryAllData(std::string authorizationToken, 
                      std::string uploadData, 
                      std::string parkId);
  void deletSignalDataFromId(std::string authorizationToken, std::string id);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::tuple<Ogre::Quaternion, double> angleToOuaterGetDire(float angle);
  void viewerDeviationPose(PurgeArrow_ &pa, Color color, int info);
  void getAndShowDeviationPose(std::vector<Way_Point_> &pose, double offset, int info);
  std::vector<Way_Point_> getDeviationPose(double dire, std::vector<Way_Point_> &poses, double offset);
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void udpateShowInitData(std::vector<std::string> &init_data);
  void viewerInitPoint(std::vector<std::string> &init_data, Color color);

  //--
  void showPoint(std::string &fileStr);
  std::vector<Way_Point_> jsonGet(std::string &fileStr);
  void viewerPoint(std::vector<Way_Point_> wps, Color color, bool viewflag);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void setRobotTransform(Eigen::Vector3d &post, Eigen::Quaterniond &orient);
  void setRobotTransform();

 public:
  bool handler_added = false;

 private:
  PointCloudMap pointcloudmap_;
  VmapData vmapdata_;
  Lanelet2Data lanelet2data_;
  MyRenderPanel *render_panel_;
  DocksViewRequests *docks_view_;
  //pcd
  double local_points_zmin_ = DOUBLE_MIN;
  double local_points_zmax_ = DOUBLE_MAX;
  float pcd_size_ = FLOAT_MIN;
  // vmap
  bool remove_flag_ = false;
  int type_id_;
  int type_num_id_;
  int current_point_index_ = 0;
  std::vector<point_> near_points_;
  std::vector<reguinfo_list> regularinfo_list_vec_;
  std::vector<tag_list> tag_list_vec_;
  // lanelet map
  int current_point3d_id_ = 0;
  std::vector<int> lanelet2_ids_;
  //task control
  Tools *tools_;
  QString filePath_pcd_;
  QString filePath_vmap_;
  QString filePath_frame_;
  std::vector<MapDockInfo> Res_docks_;
  std::vector<MapInfo> vehicle_info_;
};

class MapDialog : public QDialog {
 public:
  MapDialog();
  ~MapDialog(){};
  bool okEnabled = false;
  //   virtual void accept();
  //   virtual void reject();
  void keyPressEvent(QKeyEvent *evt);
};

#endif
