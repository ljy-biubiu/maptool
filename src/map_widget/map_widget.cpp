#include "map_widget.h"
#include <QTime>

MapWidget::MapWidget() : render_panel_(new MyRenderPanel) {
  docks_view_ = new DocksViewRequests();
  docks_view_->initMysql();
}
MapWidget::~MapWidget() {}

MyRenderPanel *MapWidget::getRenderPanel() {
  return render_panel_;
}
/*
renderpanel API//////////////////////////////////////////////////////////////////////////////////////////////////
*/
void MapWidget::setPointCloudSize(const std::string id, float pointsize) {
  pcd_size_ = pointsize;
  render_panel_->setPointsDimensions(id, pointsize * 0.1f, pointsize * 0.1f, pointsize * 0.1f);
}
void MapWidget::setLineSize(double linesize) {
  // render_panel_->_mapelem_state.linewidth = linesize;
  // osg_map_data_->SetLineWid();
}
void MapWidget::setDispalyMode(MyRenderPanel::DisplayMode mode) {
  render_panel_->setDispalyMode(mode);
}
void MapWidget::setWorkMode(MyRenderPanel::WorkMode mode) {
  render_panel_->setWorkMode(mode);
}
void MapWidget::setGridColor(const Ogre::ColourValue& color) {
  render_panel_->setGridColor(color);
}
Ogre::ColourValue MapWidget::getGridColor() {
  return render_panel_->getGridColor();
}
void MapWidget::setGridSize(uint32_t gridsize) {
  render_panel_->setGridCellCount(gridsize);
}
float MapWidget::getGridSize() {
  return render_panel_->getGridCellCount();
}
void MapWidget::setGridCellLength(float len) {
  render_panel_->setGridCellLength(len);
}
float MapWidget::getGridCellLength() {
  return render_panel_->getGridCellLength();
}
void MapWidget::setGridLineWidth(float width) {
  render_panel_->setGridLineWidth(width);
}
float MapWidget::getGridLineWidth() {
  return render_panel_->getGridLineWidth();
}
void MapWidget::setMapZmin(double zvalue) {
  // render_panel_->_mapelem_state.z = zvalue;
}
void MapWidget::setPcIntensityModeDisplay(bool mode_flag, double min_value, double max_value, std::string id) {
  render_panel_->setIntensityModeDisplay(mode_flag, min_value, max_value, id);
}
void MapWidget::setPcIntensityMode(bool mode_flag,double min_value, double max_value) {
  render_panel_->setIntensityMode(mode_flag, min_value, max_value);
}
int MapWidget::getFps() {
  return render_panel_->getFps();
}
void MapWidget::seceneObjectRemove() {
  render_panel_->seceneObjectRemove();
}
void MapWidget::seceneMapRemove() {
  render_panel_->seceneMapRemove();
}
void MapWidget::seceneObjectVisible(bool visible) {
  render_panel_->seceneObjectVisible(visible);
}
void MapWidget::seceneMapVisible(bool visible) {
  render_panel_->seceneMapVisible(visible);
}
void MapWidget::setCameraZero() {
  render_panel_->setCameraZero();
}
float MapWidget::getPitch() {
  return render_panel_->getPitch();
}
float MapWidget::getYaw() {
  return render_panel_->getYaw();
}
float MapWidget::getDistance() {
  return render_panel_->getDistance();
}
Ogre::Vector3 MapWidget::getFocalPoint() {
  return render_panel_->getFocalPoint();
}

/*
pointcloud renderpanel API////////////////////////////////////////////////////////////////////////////////////////////////
*/
// pcdmap_data to osg
void MapWidget::loadPcdMap(const std::string &id) {
  std::cout << "load pcd map" << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map(
      new pcl::PointCloud<pcl::PointXYZI>());
  map = pointcloudmap_.getMap();
  QTime time;
  time.start();
  render_panel_->addPointsDisplay(id, map);
  int time_Diff = time.elapsed();
  std::cout << "display time diff ============ " << (float)(time_Diff/1000.0) << "s" << std::endl;
  map.reset(new pcl::PointCloud<pcl::PointXYZI>);
}
void MapWidget::loadBlockMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map;
  // cloud_map = cloud.makeShared();
  std::string id = "0";
  render_panel_->addPointsDisplay(id, cloud);
  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
}
bool MapWidget::initMapOriginPostion(std::vector<std::string> &pcd_paths) {
  pointcloudmap_.createMap(pcd_paths);
  vmapdata_.origin_pos_ = pointcloudmap_.getMapOriginPostion();
//  lanelet2data_.origin_pos_ = pointcloudmap_.getMapOriginPostion();
  if (int(vmapdata_.origin_pos_.lat_rad) <= 0 &&
     int(vmapdata_.origin_pos_.lon_rad) <= 0) {
    return false;
  } else {
    return true;
  }
}
bool MapWidget::initMapOriginPostion(std::string &pcd_path) {
  pointcloudmap_.createMap(pcd_path);
  vmapdata_.origin_pos_ = pointcloudmap_.getMapOriginPostion();
//  lanelet2data_.origin_pos_ = pointcloudmap_.getMapOriginPostion();
  if (int(vmapdata_.origin_pos_.lat_rad) <= 0 &&
     int(vmapdata_.origin_pos_.lon_rad) <= 0) {
    return false;
  } else {
    return true;
  }
}
bool MapWidget::initMapOriginPostion(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
                                     Eigen::Vector4f &origin, 
                                     Eigen::Quaternionf &orientation) {
  pointcloudmap_.createMap(cloud,origin,orientation);
  vmapdata_.origin_pos_ = pointcloudmap_.getMapOriginPostion();
//  lanelet2data_.origin_pos_ = pointcloudmap_.getMapOriginPostion();
  if (int(vmapdata_.origin_pos_.lat_rad) <= 0 &&
     int(vmapdata_.origin_pos_.lon_rad) <= 0) {
    return false;
  } else {
    return true;
  }  
}
void MapWidget::reloadPcdMap(double point_zmin, double point_zmax) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr map(
      new pcl::PointCloud<pcl::PointXYZI>());
  local_points_zmin_ = point_zmin;
  local_points_zmax_ = point_zmax;
  std::cout << "pcdheight: " << point_zmin << " " << point_zmax << std::endl;
  QTime time;
  time.start();
  map = pointcloudmap_.clipMap(point_zmin, point_zmax);
  std::string id = "0";
  render_panel_->addPointsDisplay(id, map);
  int time_Diff = time.elapsed();
  std::cout << "reload display time diff: " << (float)(time_Diff/1000.0) << "s" << std::endl;
  map.reset(new pcl::PointCloud<pcl::PointXYZI>);
}
void MapWidget::getPcdmapRangeZ(double &zmin, double &zmax) {
  ExtremumInfo info = pointcloudmap_.getMapExtremum();
  std::tuple<double, double> result = std::make_tuple(info.min_value, info.max_value);
  std::tie(zmin, zmax) = result;
}
bool MapWidget::loadPcd(std::string pcd_path) {
  bool cloud = pointcloudmap_.loadMap(pcd_path);
  return cloud;
}
double MapWidget::getPcdZmin() {
  return local_points_zmin_;
}
double MapWidget::getPcdZmax() {
  return local_points_zmax_;
}
float MapWidget::getPcdSize() {
  return pcd_size_;
}

/*vmap data////////////////////////////////////////////////////////////////////////////////////////////////////
*/
void MapWidget::loadVmap(QStringList vmap_fileNames) {
  if (vmap_fileNames.size() > 0) {
    vmapdata_.clearAll();
    render_panel_->seceneObjectRemove();
    vmapdata_.readVmapFiles(vmap_fileNames);
  }
}
void MapWidget::setCurrentPurgeArrow(const int type_id, const int type_num_id,
                                     std::map<int, VectorMap_> &purge_arrows) {
 vmapdata_.setCurrentPurgeArrow(type_id, type_num_id, purge_arrows);
}
std::map<int, VectorMap_> MapWidget::getPurgeArrowInfo() {
  return vmapdata_.getPurgeArrowInfo();
}
void MapWidget::setCurrentVectorMap(const int type_id, const int type_num_id,
                                   const VectorMap_ &vectormap) {
 vmapdata_.setCurrentVectorMap(type_id, type_num_id, vectormap);
}
int MapWidget::getRandEmptyTypeNumId(const int type_id) {
 return (vmapdata_.getRandEmptyTypeNumId(type_id));
}
bool MapWidget::getCurrentVectorMap(const int type_id, const int type_num_id,
                                   VectorMap_ &vectormap) {
 bool ret = false;
 if (vmapdata_.getCurrentVectorMap(type_id, type_num_id, vectormap)) {
   ret = true;
 }
 return ret;
}
std::vector<VectorMap_> &MapWidget::getVectorType(const Vm_T type_id) {
 return (vmapdata_.getVectorType((Vm_T)type_id));
}
void MapWidget::writeVmapFiles(std::string &save_path) {
 vmapdata_.writeVmapFiles(save_path);
}
void MapWidget::writeVmapFile(QString &path_str, int &type_id) {
 vmapdata_.writeVmapFile(path_str, type_id);
}
std::string MapWidget::writeVmapJson(std::string &name, std::string &path_str, int &type_id, std::string blockId) {
  return vmapdata_.writeVmapJson(name, path_str, type_id, blockId);
}
void MapWidget::writeGidFile(QString &path_str, QList<QMap<QString,QList<QString>>> &lists) {
  vmapdata_.writeGidFile(path_str, lists);
}
std::vector<Way_Point_> MapWidget::jsonGet(std::string &fileStr) {
  return vmapdata_.jsonGet(fileStr);
}
void MapWidget::localCoordinatesToGps(Way_Point_ &wp) {
 map_projection_reproject(&vmapdata_.origin_pos_, wp.point.y, wp.point.x,
                          &wp.satfix.latitude, &wp.satfix.longitude);
}
void MapWidget::gpsCoordinatesToLocal(Way_Point_ &wp) {
 map_projection_project(&vmapdata_.origin_pos_, wp.satfix.latitude,
                        wp.satfix.longitude, &wp.point.y, &wp.point.x);
}
std::tuple<int, int> MapWidget::findCurrentNearPointNumber(int type_id,
                                                          point_ point) {
 int index = -1;
 int typeNumId = -1;
 std::tie(index, typeNumId) =
     vmapdata_.findCurrentNearPointNumber(type_id, point);
 return std::make_tuple(index, typeNumId);
}
bool MapWidget::setCurrentTypeNumIdPoint(const int type_id,
                                        const int type_num_id,
                                        const int point_index,
                                        const Way_Point_ &w_p) {
 bool ret = false;
 ret = vmapdata_.setCurrentTypeNumIdPoint(type_id, type_num_id, point_index,
                                          w_p);
 return ret;
}
bool MapWidget::getRandActiveId(int &type_id, int &type_num_id) {
  bool ret = false;
  ret = vmapdata_.getRandActiveId(type_id, type_num_id);
  return ret;
}
int MapWidget::getRandActiveTypeNumId(const int &type_id) {
 int num = vmapdata_.getRandActiveTypeNumId(type_id);
 return num;
}
bool MapWidget::getCurrentTypeNumIdPoint(Way_Point_ &wp) {
 bool ret = false;
 if (vmapdata_.getCurrentTypeNumIdPoint(type_id_, type_num_id_,
                                        current_point_index_, wp))
   ret = true;
 return ret;
}
bool MapWidget::getCurrentTypeNumIdPoint(const int type_id,
                                        const int type_num_id,
                                        const int point_index,
                                        Way_Point_ &w_p) {
 bool ret = false;
 if (vmapdata_.getCurrentTypeNumIdPoint(type_id, type_num_id, point_index,
                                        w_p))
   ret = true;
 return ret;
}
int MapWidget::getCurrentTypeNumIdNumber(const int type_id,
                                        const int type_num_id) {
 return (vmapdata_.getCurrentTypeNumIdNumber(type_id, type_num_id));
}
bool MapWidget::insertFrontTypeNumIdPoint(const int type_id,
                                         const int type_num_id,
                                         const Way_Point_ &w_p) {
 bool ret;
 ret = vmapdata_.insertFrontTypeNumIdPoint(type_id, type_num_id,
                                           current_point_index_, w_p);
 return ret;
}
bool MapWidget::insertBackTypeNumIdPoint(const int type_id,
                                        const int type_num_id,
                                        const Way_Point_ &w_p) {
 bool ret;
 ret = vmapdata_.insertBackTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_, w_p);
 return ret;
}
bool MapWidget::delCurrentTypeNumIdPoint(const int type_id,
                                        const int type_num_id) {
 bool ret;
 ret = vmapdata_.delCurrentTypeNumIdPoint(type_id, type_num_id,
                                          current_point_index_);
 return ret;
}
void MapWidget::getNewlineFromCurrentPoint(const int type_id,
                                          const int type_num_id) {
 vmapdata_.getNewlineFromcurrent(type_id, type_num_id, current_point_index_);
}
void MapWidget::clearAllVmap() { vmapdata_.clearAll();
                                 render_panel_->seceneObjectRemove();}
bool MapWidget::setTypeNumIdDelBackVmapPoint(const int type_id,
                                            const int type_num_id) {
 bool ret;
 ret = vmapdata_.setTypeNumIdDelBackPoint(type_id, type_num_id);
 return ret;
                                            }
bool MapWidget::setClearareaIntersect(const int type_id,
                                     const int type_num_id) {
 bool ret;
 ret = vmapdata_.isIntersectClearArea(type_id, type_num_id);
 return ret;
                                     }
bool MapWidget::setClearareaIntersect(const int type_id, const int type_num_id,
                                     const Way_Point_ &wp) {
 bool ret;
 ret = vmapdata_.isIntersectClearArea(type_id, type_num_id, wp, current_point_index_);
 return ret;
                                     }
bool MapWidget::setTypeNumIdDelVmapPoints(const int type_id,
                                         const int type_num_id) {
 bool ret;
 ret = vmapdata_.setTypeNumIdDelPoints(type_id, type_num_id);
 return ret;
}
bool MapWidget::findNearPoint(int type_id, point_ &find_point, point_ &point,
                             double distance) {
 bool ret = false;
 int index = -1;
 int typeNumId = -1;
 Way_Point_ wp;
 std::tie(index, typeNumId) = findCurrentNearPointNumber(type_id, find_point);
 if (typeNumId >= 0 && index > 0) {
   if (getCurrentTypeNumIdPoint(type_id, typeNumId, index - 1, wp)) {
     double l = pow((find_point.x - wp.point.x), 2) +
                pow((find_point.y - wp.point.y), 2);
     if (fabs(l) < distance) {
       point.x = wp.point.x;
       point.y = wp.point.y;
       point.z = wp.point.z;
       ret = true;
     }
   }
 }
 return ret;
}

/*debug cleararea
*/

/*lanelet2map data//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
bool MapWidget::laneletmapEmpty() {
  bool ret = false;
  ret = lanelet2data_.mapEmpty();
  return ret;
}
void MapWidget::moveFrontPoint3d(lanelet::Point3d point,
                                 lanelet::Point3d &new_point3d,
                                 lanelet::LineString3d linestring) {
  lanelet2data_.moveFrontPoint3d(point, new_point3d, linestring);
}
void MapWidget::moveFrontPoint3d(lanelet::Point3d point,
                                 lanelet::Point3d &new_point3d,
                                 lanelet::Polygon3d poly) {
  lanelet2data_.moveFrontPoint3d(point, new_point3d, poly);
}
void MapWidget::moveBackPoint3d(lanelet::Point3d point,
                                lanelet::Point3d &new_point3d,
                                lanelet::Polygon3d poly) {
  lanelet2data_.moveBackPoint3d(point, new_point3d, poly);
}
void MapWidget::moveBackPoint3d(lanelet::Point3d point,
                                lanelet::Point3d &new_point3d,
                                lanelet::LineString3d linestring) {
  lanelet2data_.moveBackPoint3d(point, new_point3d, linestring);
}
void MapWidget::insertFrontPoint3d(lanelet::Point3d point,
                                   lanelet::LineString3d &linestring,
                                   lanelet::Point3d &new_point3d) {
  lanelet2data_.insertFrontPoint3d(point, linestring, new_point3d);
}
void MapWidget::insertFrontPoint3d(lanelet::Point3d point,
                                   lanelet::Polygon3d &poly,
                                   lanelet::Point3d &new_point3d) {
  lanelet2data_.insertFrontPoint3d(point, poly, new_point3d);
}
void MapWidget::insertBackPoint3d(lanelet::Point3d point,
                                  lanelet::LineString3d &linestring,
                                  lanelet::Point3d &new_point3d) {
  lanelet2data_.insertBackPoint3d(point, linestring, new_point3d);
}
void MapWidget::insertBackPoint3d(lanelet::Point3d point,
                                  lanelet::Polygon3d &poly,
                                  lanelet::Point3d &new_point3d) {
  lanelet2data_.insertBackPoint3d(point, poly, new_point3d);
}
void MapWidget::loadLaneletMap(QString lanelet_path) {
  std::string lanelet_file = lanelet_path.toStdString();
  if (lanelet_file.empty()) return;
  if (lanelet2data_.loadLanelet2OSMfile(lanelet_file)) {
    std::cout << " lanelet map load ok --"
              << lanelet2data_.map_->pointLayer.size() << std::endl;
  }
}
void MapWidget::saveLaneletMap(std::string lanelet_path) {
  // for invalid value -- map out
  lanelet2data_.map_out();
  if (lanelet2data_.exportLanelet2OSMfile(lanelet_path)) {
    std::cout << " lanelet map save ok --"
              << lanelet2data_.map_->pointLayer.size() << std::endl;
  }
}
lanelet::Lanelet MapWidget::createRoadLanelet(
    std::vector<lanelet::Point3d> &point3d_vec) {
  std::cout << "....debug.,,in creat lanelet...." << std::endl;
  lanelet::Lanelet llt = lanelet2data_.createLanelet(point3d_vec);
  std::cout << "....after.,,creat lanelet...." << std::endl;
  lanelet2data_.addRoadAtrribute(llt);
  return llt;
}
lanelet::Lanelet MapWidget::createCrosswalkLanelet(
    std::vector<lanelet::Point3d> &point3d_vec) {
  lanelet::Lanelet llt = lanelet2data_.createLanelet(point3d_vec);
  lanelet2data_.addCrosswalkAtrribute(llt);
  return llt;
}
void MapWidget::extendLanelet(lanelet::Lanelet &Lanelet,
                              lanelet::Point3d &point3d) {
  lanelet2data_.extendLanelet(Lanelet, point3d);
}
void MapWidget::extendLanelet(int lanelet_id, lanelet::Point3d &point3d) {
  lanelet::Lanelet Lanelet;
  if (lanelet2data_.findLaneletWithId(lanelet_id, Lanelet))
    lanelet2data_.extendLanelet(Lanelet, point3d);
}
lanelet::LineString3d MapWidget::createRoadLinestring(
    std::vector<lanelet::Point3d> &point3d_vec) {
  lanelet::LineString3d ls = lanelet2data_.createLinestring(point3d_vec);
  lanelet2data_.addlinestringAtrribute(ls);
  return ls;
}
lanelet::LineString3d MapWidget::createStopline(
    std::vector<lanelet::Point3d> &point3d_vec) {
  lanelet::LineString3d ls = lanelet2data_.createLinestring(point3d_vec);
  lanelet2data_.addStoplineAtrribute(ls);
  return ls;
}
lanelet::LineString3d MapWidget::createBump(
    std::vector<lanelet::Point3d> &point3d_vec) {
  lanelet::LineString3d ls = lanelet2data_.createLinestring(point3d_vec);
  lanelet2data_.addBumpAtrribute(ls);
  return ls;
}
void MapWidget::extendLinestring(int linestring_id, lanelet::Point3d &point3d) {
  lanelet::LineString3d ls;
  if (lanelet2data_.findLinestringWithId(linestring_id, ls))
    lanelet2data_.extendLinestring(ls, point3d);
}
lanelet::Polygon3d MapWidget::createIntersection(
    std::vector<lanelet::Point3d> &point3d_vec) {
  lanelet::Polygon3d poly = lanelet2data_.createPolygon(point3d_vec);
  return poly;
}
void MapWidget::extendIntersection(int poly_id, lanelet::Point3d &point3d) {
  lanelet::Polygon3d pl;
  if (lanelet2data_.findPoly3dWithId(poly_id, pl))
    lanelet2data_.extendPolygon(pl, point3d);
}
void MapWidget::invertLanelet() {
  lanelet::Lanelet ll;
  int id1 = lanelet2_ids_.back();
  if ((lanelet2data_.findLaneletWithId(id1, ll))) {
    lanelet2data_.changeLaneletBound(ll);
  }
}

lanelet::Point3d MapWidget::searchNearstPoint3d(Way_Point_ pose) {
  lanelet::BasicPoint2d searchPoint = {pose.point.x, pose.point.y};
  lanelet::Point3d pt = lanelet2data_.searchNearstPoint(searchPoint);
  return pt;
}
lanelet::LineString3d MapWidget::searchNearstLinestring(Way_Point_ pose) {
  lanelet::BasicPoint2d searchPoint = {pose.point.x, pose.point.y};
  lanelet::LineString3d ls = lanelet2data_.searchNearstLinestring(searchPoint);
  return ls;
}
lanelet::Lanelet MapWidget::searchNearstLanelet(Way_Point_ pose) {
  lanelet::BasicPoint2d searchPoint = {pose.point.x, pose.point.y};
  lanelet::Lanelet ll = lanelet2data_.searchNearstLanelet(searchPoint);
  return ll;
}
lanelet::Polygon3d MapWidget::searchNearstPolygon(Way_Point_ pose) {
  lanelet::BasicPoint2d searchPoint = {pose.point.x, pose.point.y};
  lanelet::Polygon3d pl = lanelet2data_.searchNearstPolygon(searchPoint);
  return pl;
}
bool MapWidget::findPoint3dWithId(int &point3d_id, lanelet::Point3d &point3d) {
  bool ret = false;
  current_point3d_id_ = point3d_id;
  ret = lanelet2data_.findPoint3dWithId(current_point3d_id_, point3d);
  return ret;
}
bool MapWidget::isPoint3dinLinestringLayer(int point3d_id) {
  bool ret = false;
  ret = lanelet2data_.isPoint3dinLinestringLayer(point3d_id);
  return ret;
}
bool MapWidget::isPoint3dinLinestring(int point3d_id,
                                      lanelet::LineString3d ll) {
  bool ret = false;
  ret = lanelet2data_.isPoint3dinLinestring(point3d_id, ll);
  return ret;
}
bool MapWidget::isPoint3dinPoly(int point3d_id, lanelet::Polygon3d pl) {
  bool ret = false;
  ret = lanelet2data_.isPoint3dinPoly(point3d_id, pl);
  return ret;
}
bool MapWidget::findLinestringWithId(int &linestring_id,
                                     lanelet::LineString3d &linestring) {
  bool ret = false;
  // current_point3d_id_ = linestring_id;
  // ret = lanelet2data_.findLinestringWithId(current_point3d_id_, linestring);
  ret = lanelet2data_.findLinestringWithId(linestring_id, linestring);
  return ret;
}
bool MapWidget::findLaneletWithId(int &lanelet_id, lanelet::Lanelet &lanelet) {
  bool ret = false;
  // current_point3d_id_ = lanelet_id;
  // ret = lanelet2data_.findLaneletWithId(current_point3d_id_, lanelet);
  ret = lanelet2data_.findLaneletWithId(lanelet_id, lanelet);
  return ret;
}
bool MapWidget::findPoly3dWithId(int &poly3d_id,
                                 lanelet::Polygon3d &polygon3d) {
  bool ret = false;
  // current_point3d_id_ = poly3d_id;
  // ret = lanelet2data_.findPoly3dWithId(current_point3d_id_, polygon3d);
  ret = lanelet2data_.findPoly3dWithId(poly3d_id, polygon3d);
  return ret;
}
void MapWidget::resetPoint3d(lanelet::Point3d &point3d) {
  lanelet2data_.createPoint(point3d);
}
void MapWidget::rmPoint3d(lanelet::Point3d &pt) {
  lanelet2data_.rmPrimitive(pt);
}
void MapWidget::rmChooseLanelet2Ids() {
  for (auto id : lanelet2_ids_) {
    lanelet::Point3d pt;
    lanelet::LineString3d ls;
    lanelet::Lanelet ll;
    lanelet::Polygon3d pl;
    if (lanelet2data_.findPoint3dWithId(id, pt)) {
      lanelet2data_.rmPrimitive(pt);
    } else if (lanelet2data_.findLinestringWithId(id, ls)) {
      lanelet2data_.rmPrimitive(ls);
    } else if (lanelet2data_.findLaneletWithId(id, ll)) {
      lanelet2data_.rmPrimitive(ll);
    } else if (lanelet2data_.findPoly3dWithId(id, pl)) {
      lanelet2data_.rmPrimitive(pl);
    }
  }
}
int MapWidget::addLanelet() {
  lanelet::LineString3d ls_left, ls_right;
  int id1 = lanelet2_ids_.front();
  int id2 = lanelet2_ids_.back();
  lanelet::Lanelet ll;
  if ((lanelet2data_.findLinestringWithId(id1, ls_left)) &&
      (lanelet2data_.findLinestringWithId(id2, ls_right))) {
    ll = lanelet2data_.createLanelet(ls_left, ls_right);
    lanelet2data_.addRoadAtrribute(ll);
  }
  return ll.id();
}
void MapWidget::jointLanelet() {
  lanelet::Lanelet ll_back, ll_next;
  int id1 = lanelet2_ids_.front();
  int id2 = lanelet2_ids_.back();
  if ((lanelet2data_.findLaneletWithId(id1, ll_back)) &&
      (lanelet2data_.findLaneletWithId(id2, ll_next))) {
    lanelet2data_.jointLanelet(ll_back, ll_next);
  }
}
bool MapWidget::addregular(int &regular_id, reguinfo_list &reg_list) {
  bool ret = false;
  int id1 = lanelet2_ids_.front();
  int id2 = lanelet2_ids_.back();
  lanelet::Lanelet ll;
  lanelet::LineString3d ls;
  lanelet::Polygon3d pl;
  if (lanelet2data_.findLaneletWithId(id1, ll)) {
    if (lanelet2data_.findLinestringWithId(id2, ls)) {
      if (ls.attribute(lanelet::AttributeName::Type).value() ==
          lanelet::AttributeValueString::LineThin) {
        return ret;
      } else {
        regular_id = lanelet2data_.addRegElement(ll, ls);
        initRegularInfoList(reg_list, regular_id, ll.id(), ls.id(),
                            ls.attribute(lanelet::AttributeName::Type).value());
        ret = true;
        return ret;
      }
    } else if (lanelet2data_.findPoly3dWithId(id2, pl)) {
      regular_id = lanelet2data_.addRegElement(ll, pl);
      initRegularInfoList(reg_list, regular_id, ll.id(), pl.id(),
                          pl.attribute(lanelet::AttributeName::Type).value());
      ret = true;
      return ret;
    }
  }
  if (lanelet2data_.findLaneletWithId(id2, ll)) {
    if (lanelet2data_.findLinestringWithId(id1, ls)) {
      if (ls.attribute(lanelet::AttributeName::Type).value() ==
          lanelet::AttributeValueString::LineThin) {
        return ret;
      } else {
        regular_id = lanelet2data_.addRegElement(ll, ls);
        initRegularInfoList(reg_list, regular_id, ll.id(), ls.id(),
                            ls.attribute(lanelet::AttributeName::Type).value());
        ret = true;
        return ret;
      }
    } else if (lanelet2data_.findPoly3dWithId(id1, pl)) {
      regular_id = lanelet2data_.addRegElement(ll, pl);
      initRegularInfoList(reg_list, regular_id, ll.id(), pl.id(),
                          pl.attribute(lanelet::AttributeName::Type).value());
      ret = true;
      return ret;
    }
  }
  return ret;
}
void MapWidget::initRegularInfoList(reguinfo_list &reg_list, int regular_id,
                                    int lanelet_id, int member_id,
                                    std::string member_str) {
  reg_list.laneletid = lanelet_id;
  reg_list.regmemberid = member_id;
  reg_list.Subtype = member_str;
  reg_list.reglists.Regelement_id = regular_id;
}
bool MapWidget::haveCurrentRefIdRole(int ref_id, std::string &ref_role) {
  bool ret = false;
  ret = lanelet2data_.haveCurrentRefIdRole(ref_id, ref_role);
  if (!ret) {
    QMessageBox::question(this, tr("提示"),
                          tr("当前referrer id 的地图元素不存在!").arg(ref_id),
                          QMessageBox::Yes);
    // pBtn_addreguinfo->setEnabled(false);
    // regulaInfoStatus(false);
  }
  return ret;
}
void MapWidget::getNewlineFromCurrentPoint() {
  lanelet::Point3d pt;
  if (findPoint3dWithId(current_point3d_id_, pt))
    lanelet2data_.getNewlineFromcurrentPoint(current_point3d_id_);
}
bool MapWidget::getLaneletCurrentTypeBind(point_ &point) {
  bool ret = false;
  lanelet::Point3d pt;
  pt.x() = point.x;
  pt.y() = point.y;
  pt.z() = point.z;
  ret = lanelet2data_.getBindPointAtLine(lanelet2_ids_.back(),
                                         current_point3d_id_, pt);
  return ret;
}
bool MapWidget::getLaneletCurrentTypeBind() {
  bool ret = false;
  ret = lanelet2data_.getBindPointAtLine(lanelet2_ids_.back(),
                                         current_point3d_id_);
  return ret;
}
void MapWidget::clearAllLanelet2map() { 
  lanelet2data_.clearMap();
  std::vector<tag_list>().swap(tag_list_vec_);
  std::vector<reguinfo_list>().swap(regularinfo_list_vec_);
}
void MapWidget::gpsCoordinatesToLocal(LatLonInfo latlon_info,
                                      lanelet::Point3d &point) {
  gps_point_ pose;
  lanelet2data_.gps2pose(latlon_info, pose);
  point.x() = pose.x;
  point.y() = pose.y;
  lanelet2data_.createPoint(point);
}
std::vector<reguinfo_list> MapWidget::readLaneletMapRegular() {
  std::vector<reguinfo_list> regular;
  regular = lanelet2data_.regelementload();
  lanelet2data_.clearRegelem();
  if (regular.size() > 0) {
    std::cout << "readLaneletMapRegular.size()= " << regular.size()
              << std::endl;
    std::cout << "read regular_vec.at(0).Subtype =" << regular.at(0).Subtype
              << std::endl;
  }
  return regular;
}
void MapWidget::clearRegelem() { lanelet2data_.clearRegelem(); }
void MapWidget::writeLaneletMapRegular(
    std::vector<reguinfo_list> &regular_vec) {
  std::cout << "write regular_vec to map" << std::endl;
  if (regular_vec.size() > 0) {
    lanelet2data_.addReglist(regular_vec);
    // for (auto reginfo:regular_vec)
    // {
    // std::cout<<"reginfo.Subtype ="<<reginfo.Subtype<<std::endl;
    // std::cout<<"reginfo.regmemberid ="<<reginfo.regmemberid<<std::endl;
    // if (reginfo.reglists.refers_list.size()>0)
    // {std::cout<<"in ..data.cpp  referName
    // ="<<reginfo.reglists.refers_list[0].referName<<std::endl;}
    // }
  }
}
std::vector<tag_list> MapWidget::readLaneletMapCustomizeTags() {
  std::vector<tag_list> tags_vec = lanelet2data_.tagsFromRlanelet2map();
  return tags_vec;
}
void MapWidget::writeLaneletMapCustomizeTags(std::vector<tag_list> tags_vec) {
  lanelet2data_.addCustomizeTags(tags_vec);
}
std::vector<point_> MapWidget::geteachLinestringsPoints(
    lanelet::LineString3d &line) {
  std::vector<point_> wp;
  for (auto &value : line) {
    point_ p;
    p.x = value.x();
    p.y = value.y();
    p.z = value.z();
    wp.push_back(p);
  }
  return wp;
}
void MapWidget::findLinestringswithPoint3d(lanelet::LineStrings3d &linestrings,
                                           lanelet::Point3d point) {
  lanelet2data_.findLinestringswithPoint3d(linestrings, point);
}
std::vector<int> MapWidget::findNextLanelet(lanelet::Lanelet &Lanelet) {
  std::vector<int> nextids = lanelet2data_.findNextLanelet(Lanelet);
  return nextids;
}
void MapWidget::deletRegularInfo(int regul_id) {
  for (auto it = regularinfo_list_vec_.begin();it < regularinfo_list_vec_.end();) {
    reguinfo_list reg = *it;
    if (reg.reglists.Regelement_id == regul_id)
      it = regularinfo_list_vec_.erase(it);
    else
      it++;
  }
}
void MapWidget::addRegularInfo(reguinfo_list &reg_list) {
  regularinfo_list_vec_.push_back(reg_list);
}
std::vector<reguinfo_list> MapWidget::getRegularInfoList() {
  return regularinfo_list_vec_;
}
std::vector<tag_list> MapWidget::getTagInfoList() {
  return tag_list_vec_;
}
void MapWidget::createNewRegularInfoList(std::vector<reguinfo_list> &regularlist) {
  regularinfo_list_vec_ = regularlist;
}
void MapWidget::createNewTagInfoList(std::vector<tag_list> &taglist) {
  tag_list_vec_ = taglist;
}

/*viewer info////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
void MapWidget::getEqualPoint(std::vector<VectorMap_> vectormap, point_ pt,
                              int ven, int wpn) {
  for (size_t i = 0; i < vectormap.size(); i++) {
    if (vectormap[i].wp.size() > 0) {
      for (size_t j = 0; j < vectormap[i].wp.size(); j++) {
        if (i == ven && j == wpn) {
            continue;
        } else {
          if (pt.x == vectormap[i].wp[j].point.x &&
              pt.y == vectormap[i].wp[j].point.y &&
              pt.z == vectormap[i].wp[j].point.z) {
            near_points_.push_back(pt);
            near_points_.push_back(vectormap[i].wp[j].point);
          }
        }
      } 
    }
  } 
}
void MapWidget::getCurrentVmapElementId(int t_typeid, int t_typenumid,
                                        int t_current_index) {
  type_id_ = t_typeid;
  type_num_id_ = t_typenumid;
  current_point_index_ = t_current_index;
}
void MapWidget::createCurrentPointViewer(Way_Point_ wp, Color color) {
  std::vector<Ogre::Vector3> points;
  Ogre::Vector3 pt;
  pt.x = wp.point.x;
  pt.y = wp.point.y;
  pt.z = wp.point.z;
  points.push_back(pt);
  std::string id = "current_point";
  Ogre::ColourValue c = createColorRGBA(color);
  render_panel_->createPointsObject(id, Shape::Type::Cube, points, 0.2, c);
}
void MapWidget::createBindPointViewer(std::vector<point_> points, Color color) {
  std::vector<Ogre::Vector3> v_points;
  Ogre::Vector3 pt;
  for (auto point : points) {
    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    v_points.push_back(pt);
  }
  std::string id = "bind_point";
  Ogre::ColourValue c = createColorRGBA(color);
  render_panel_->createPointsObject(id, Shape::Type::Cube, v_points, 0.2, c);
}
void MapWidget::viewerVmapPoint(std::vector<VectorMap_> vectormap, Color color, bool viewflag) {
  for (auto vp : vectormap) {
    Color showColor = color;
    if (type_id_ == vp.type_id && type_num_id_ == vp.type_num_id && viewflag) {
      showColor = Color::LIGHT_YELLOW;
    }
    if (vp.wp.size() > 0) {
      std::vector<Ogre::Vector3> points;
      Ogre::Vector3 pt;
      for (auto m_p : vp.wp) {
        pt.x = m_p.point.x;
        pt.y = m_p.point.y;
        pt.z = m_p.point.z;
        points.push_back(pt);
      }
      std::string id =
          type_name_str[vp.type_id] + std::to_string(vp.type_num_id);
      Ogre::ColourValue c = createColorRGBA(showColor);
      render_panel_->createPointsObject(id, Shape::Type::Cube, points, 0.4, c);
      // text
      pt.x = vp.wp.front().point.x;
      pt.y = vp.wp.front().point.y;
      pt.z = vp.wp.front().point.z;
      std::string text = std::to_string(vp.type_num_id);
      Color textColor = Color::LIGHT_YELLOW;
      Ogre::ColourValue tc = createColorRGBA(textColor);
      render_panel_->createTextObject(id, text, points.at(0), 0.8, tc);
    }
  }
}
//--临时(显示其他点)
void MapWidget::showPoint(std::string &fileStr) {
  std::cout << "enter------" << std::endl;
  std::vector<Way_Point_> poses_str = jsonGet(fileStr);
  viewerPoint(poses_str,Color::WHITE,true);
}
void MapWidget::viewerPoint(std::vector<Way_Point_> wps, Color color, bool viewflag) {
  int i = 0;
  for (auto wp : wps) {
    Color showColor = color;
    // if (type_id_ == vp.type_id && type_num_id_ == vp.type_num_id && viewflag) {
    //   showColor = Color::LIGHT_YELLOW;
    // }
    // if (wp.size() > 0) {
      std::vector<Ogre::Vector3> points;
      Ogre::Vector3 pt;
      // for (auto m_p : wp) {
      pt.x = wp.point.x;
      pt.y = wp.point.y;
      pt.z = wp.point.z;
      points.push_back(pt);
      // }
      std::string id =  std::to_string(i);
      Ogre::ColourValue c = createColorRGBA(showColor);
      render_panel_->createPointsObject(id, Shape::Type::Cube, points, 0.5, c);
      // text
      id = std::to_string(i);
      Color textColor = Color::LIGHT_YELLOW;
      Ogre::ColourValue tc = createColorRGBA(textColor);
      render_panel_->createTextObject(id, id, pt, 1.0, tc);
      i = i+1;
    // }
  }
}
void MapWidget::viewerVmapLine(std::vector<VectorMap_> vectormap, Color color, bool viewflag) {
  for (size_t i = 0; i < vectormap.size(); i++) {
    Color showColor = color;
    if (type_id_ == vectormap[i].type_id && type_num_id_ == vectormap[i].type_num_id && viewflag) {
      showColor = Color::LIGHT_YELLOW;
    }
    if (vectormap[i].wp.size() > 0) {
      std::vector<Ogre::Vector3> points;
      Ogre::Vector3 pt;
      for (size_t j = 0; j < vectormap[i].wp.size(); j++) {
        pt.x = vectormap[i].wp[j].point.x;
        pt.y = vectormap[i].wp[j].point.y;
        pt.z = vectormap[i].wp[j].point.z;
        points.push_back(pt);
        if (vectormap[i].type_id == 0 || vectormap[i].type_id == 9) {
          getEqualPoint(vectormap, vectormap[i].wp[j].point, i, j);
        }
      }
      std::string id =
          type_name_str[vectormap[i].type_id] + std::to_string(vectormap[i].type_num_id);
      Ogre::ColourValue c = createColorRGBA(showColor);
      render_panel_->createLineObject(id, points, 0.02, c);
      // text
      pt.x = vectormap[i].wp.front().point.x;
      pt.y = vectormap[i].wp.front().point.y;
      pt.z = vectormap[i].wp.front().point.z;
      std::string text = std::to_string(vectormap[i].type_num_id);
      Color textColor = Color::WHITE;
      Ogre::ColourValue tc = createColorRGBA(textColor);
      render_panel_->createTextObject(id, text, points.at(0), 0.8, tc);
    }
  }
  // near point
  if (near_points_.size() > 0) {
    createBindPointViewer(near_points_, Color::MAGENTA);
  }
}
void MapWidget::updateShowVMapData(int t_typeid, int t_typenumid,
                                  int t_current_index, bool viewflag) {
near_points_.clear();
 type_id_ = t_typeid;
 type_num_id_ = t_typenumid;
 current_point_index_ = t_current_index;
 getCurrentVmapElementId(type_id_, type_num_id_,
                                        current_point_index_);
 Way_Point_ wp;
 if (getCurrentTypeNumIdPoint(wp) && viewflag) {
   createCurrentPointViewer(wp, Color::RED);
 }
 updateShowVMapData(viewflag);
}
void MapWidget::updateShowVMapData(bool viewflag) {
 
 viewerVmapPoint(getVectorType(Vm_T::StopPoint), Color::RED, viewflag);
 viewerVmapPoint(getVectorType(Vm_T::Elevator), Color::GREEN, viewflag);
 viewerVmapPoint(getVectorType(Vm_T::ConvergePoint), Color::BLUE, viewflag);
 viewerVmapPoint(getVectorType(Vm_T::NodePoint), Color::BLUE, viewflag);

 viewerVmapLine(getVectorType(Vm_T::Signal), Color::GREEN, viewflag);
 viewerVmapLine(getVectorType(Vm_T::Lane), Color::GREEN, viewflag);
 viewerVmapLine(getVectorType(Vm_T::RoadEdge), Color::WHITE, viewflag);
 viewerVmapLine(getVectorType(Vm_T::CrossWalk), Color::WHITE, viewflag);
 viewerVmapLine(getVectorType(Vm_T::WaitLine), Color::WHITE, viewflag);
 viewerVmapLine(getVectorType(Vm_T::DeceZone), Color::YELLOW, viewflag);

 viewerVmapLine(getVectorType(Vm_T::Junction), Color::BLUE, viewflag);
 viewerVmapLine(getVectorType(Vm_T::SprayArea), Color::MAGENTA, viewflag);
 viewerVmapLine(getVectorType(Vm_T::ClearArea), Color::MAGENTA, viewflag);
 viewerVmapLine(getVectorType(Vm_T::AttributeArea), Color::BLUE, viewflag);
 viewerVmapLine(getVectorType(Vm_T::Gate), Color::WHITE, viewflag);
 // viewerVmapOverPoint(getVectorType(Vm_T::Lane), Color::MAGENTA);
}
  
//lanelet-view/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapWidget::getCurrentLaneletMapElementId(
    int point3d_id, std::vector<int> ele_lanelet_ids) {
  current_point3d_id_ = point3d_id;
  std::vector<int>().swap(lanelet2_ids_);
  lanelet2_ids_.assign(ele_lanelet_ids.begin(), ele_lanelet_ids.end());
}
//lanelet点层
void MapWidget::LaneletMapPointViewer(const lanelet::ConstPoints3d &points,
                                      Color color) {
  Color showColor;
  for (auto p : points) {
    showColor = color;
    if (p.id() == current_point3d_id_) {
      showColor = Color::LIGHT_YELLOW;
    }
    std::vector<Ogre::Vector3> points_p;
    Ogre::Vector3 r;
    r.x = p.x();
    r.y = p.y();
    r.z = p.z();
    points_p.push_back(r);
    std::string id = std::to_string(p.id());
    Ogre::ColourValue c = createColorRGBA(showColor);
    render_panel_->createPointsObject(id, Shape::Type::Sphere, points_p, 0.5, c);
  }
}
//lanelet线层
void MapWidget::LaneletMapLinestringViewer(
    const lanelet::ConstLineStrings3d &linestrings, Color color) {
  for (auto lt = linestrings.begin(); lt != linestrings.end(); lt++) {
    Color showColor = color;
    std::vector<Ogre::Vector3> points;
    lanelet::ConstLineString3d ls = *lt;
    for (auto i = ls.begin(); i != ls.end(); i++) {
      lanelet::ConstPoint3d pt = *i;
      Ogre::Vector3 r;
      r.x = pt.x();
      r.y = pt.y();
      r.z = pt.z();
      points.push_back(r);
    }
    for (auto cp : lanelet2_ids_) {
      if (ls.id() == cp) {
        showColor = Color::LIGHT_YELLOW;
      }
    }
    std::string id = std::to_string(ls.id());
    Ogre::ColourValue c = createColorRGBA(showColor);
    render_panel_->createLineObject(id, points, 0.1, c);
  }
}
//lanelet层
void MapWidget::LaneletMapLaneletViewer(const lanelet::ConstLanelets lanelets, Color color) {
  lanelet::ConstLineStrings3d lines;
  std::vector<int> ids_show;
  Color showColor = color;
  for (auto lt = lanelets.begin(); lt != lanelets.end(); lt++) {
    showColor = color;
    lanelet::ConstLanelet ll = *lt;
    if (ll.leftBound().size() == 0 || ll.rightBound().size() == 0) continue;
    lines.push_back(ll.leftBound());
    lines.push_back(ll.rightBound());
    for (auto cp : lanelet2_ids_) {
      if (cp == ll.id()) {  // for current lanelet
        ids_show.push_back(ll.leftBound().id());
        ids_show.push_back(ll.rightBound().id());
        showColor = Color::LIGHT_YELLOW;
      } else if (cp == ll.leftBound().id()) {
        ids_show.push_back(ll.leftBound().id());
      } else if (cp == ll.rightBound().id()) {
        ids_show.push_back(ll.rightBound().id());
      }
    }
    // text
    Ogre::ColourValue c = createColorRGBA(showColor);
    creatLaneletTextViewer(ll, c);
  }
  sort(lines.begin(), lines.end(),
       [](lanelet::ConstLineString3d a, lanelet::ConstLineString3d b) {
         return a.id() < b.id();
       });
  lines.erase(
      unique(lines.begin(), lines.end(),
             [](lanelet::ConstLineString3d a, lanelet::ConstLineString3d b) {
               return a.id() == b.id();
             }),
      lines.end());
  for (auto ls : lines) {
    showColor = color;
    for (auto id : ids_show) {
      if (ls.id() == id) {
        showColor = Color::LIGHT_YELLOW;
      }
    }
    std::vector<Ogre::Vector3> points;
    for (auto i = ls.begin(); i != ls.end(); i++) {
      lanelet::ConstPoint3d pt = *i;
      Ogre::Vector3 r;
      r.x = pt.x();
      r.y = pt.y();
      r.z = pt.z();
      points.push_back(r);
    }
    std::string id = std::to_string(ls.id());
    Ogre::ColourValue c = createColorRGBA(showColor);
    render_panel_->createLineObject(id, points, 0.1, c);
  }
}
//lanelet，poly层
void MapWidget::LaneletMapPolygonViewer(lanelet::ConstPolygons3d &polygons,
                                        Color color) {
  for (auto lt = polygons.begin(); lt != polygons.end(); lt++) {
    if (lt->size() > 0) {
      Color showColor = color;
      lanelet::ConstPolygon3d ls = *lt;
      for (auto cp : lanelet2_ids_) {
        if (ls.id() == cp) {
          showColor = Color::LIGHT_YELLOW;
        }
      }
      std::vector<Ogre::Vector3> points;
      Ogre::Vector3 r;
      for (auto i = ls.begin(); i != ls.end(); i++) {
        lanelet::ConstPoint3d pt = *i;
        r.x = pt.x();
        r.y = pt.y();
        r.z = pt.z();
        points.push_back(r);
      }
      std::string id = std::to_string(ls.id());
      Ogre::ColourValue c = createColorRGBA(showColor);
      render_panel_->createLineObject(id, points, 0.2, c);
      // text
      r.x = lt->begin()->x();
      r.y = lt->begin()->y();
      r.z = lt->begin()->z();
      render_panel_->createTextObject(id, id, r, 0.8, c);
    }
  }
}
//lanelet ids text文本的显示
void MapWidget::creatLaneletTextViewer(const lanelet::ConstLanelet &lanelet,
                                       Ogre::ColourValue color) {
  if (lanelet.leftBound3d().size() == 0 || lanelet.rightBound3d().size() == 0)
    return;
  std::string id = std::to_string(lanelet.id());

  std::string ret_str;
  std::string attr_str = "turn_direction";
  if (lanelet.hasAttribute(attr_str)) {
    lanelet::Attribute attr = lanelet.attribute(attr_str);
    ret_str = attr.value();
  }
  std::string str1 = "right";
  std::string str2 = "left";
  Ogre::Vector3 pos;
  if (str1 == ret_str) {
    lanelet::ConstLineString3d right_ls = lanelet.rightBound();
    pos.x = right_ls.begin()->x();
    pos.y = right_ls.begin()->y();
    pos.z = right_ls.begin()->z();
  } else if (str2 == ret_str) {
    lanelet::ConstLineString3d left_ls = lanelet.leftBound();
    pos.x = left_ls.begin()->x();
    pos.y = left_ls.begin()->y();
    pos.z = left_ls.begin()->z();
  } else {
    lanelet::ConstLineString3d center_ls = lanelet.centerline();
    pos.x = center_ls.begin()->x();
    pos.y = center_ls.begin()->y();
    pos.z = center_ls.begin()->z();
  }
  render_panel_->createTextObject(id, id, pos, 0.8, color);
}
void MapWidget::updateShowLaneletMapData(int current_pt, std::vector<int> ids) {
  current_point3d_id_ = current_pt;
  lanelet2_ids_.clear();
  lanelet2_ids_.assign(ids.begin(), ids.end());
  getCurrentLaneletMapElementId(current_point3d_id_,lanelet2_ids_);

  lanelet::ConstPoints3d points = lanelet2data_.pointLayer();
  lanelet::ConstLineStrings3d lss = lanelet2data_.lineStringLayer();
  lanelet::ConstLanelets lls = lanelet2data_.laneletLayer();
  lanelet::ConstPolygons3d polygons = lanelet2data_.PolygonLayer();
  LaneletMapPointViewer(points, Color::RED);
  LaneletMapLinestringViewer(lanelet2data_.stoplineLineStrings(lss), Color::RED);
  LaneletMapLinestringViewer(lanelet2data_.bumpLineStrings(lss), Color::WHITE);
  LaneletMapLaneletViewer(lanelet2data_.roadLanelets(lls), Color::WHITE);
  LaneletMapLaneletViewer(lanelet2data_.crosswalkLanelets(lls), Color::YELLOW);
  LaneletMapPolygonViewer(polygons, Color::GREEN);
  LaneletMapLinestringViewer(lanelet2data_.stampLinethinLineStrings(lss, lls), Color::WHITE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//view docks
void MapWidget::seceneDocksRemove() {
  render_panel_->seceneDocksRemove();
}
void MapWidget::seceneDocksVisible(bool visible) {
  render_panel_->seceneDocksVisible(visible);
}
//设置桩点所在环境
void MapWidget::setDockRequestDomain(std::string domain) {
  docks_view_->setDockRequestDomain(domain);
}
//查找最近桩点---
int MapWidget::findCurrentNearDocksNumber(point_ point, double distance) {
  double len = std::numeric_limits<double>::max();
  int index = -1;
  for (size_t i = 0; i < Res_docks_.size(); i++) {
    double l = pow((point.x - Res_docks_[i].post(0)), 2) +
                pow((point.y - Res_docks_[i].post(1)), 2);
    if (len > l) {
      len = l;
      index = i;
      if (len > distance)
      {
        index = -1;
      }
    }
  }
  std::cout << "最近docker++++++++++++++++=： " << index << std::endl;
  // std::cout << "docker信息：" << Res_docks_[index].name << std::endl;
  return index;
}
//docks-push
void MapWidget::readNewestDockFile(QString &path_str,
                                   std::vector<std::pair<std::string, std::vector<double>>> &uploadDockInfos) {
  vmapdata_.readNewestDockFile(path_str,uploadDockInfos);
}
bool MapWidget::uploadDockById(std::vector<std::pair<std::string, std::vector<double>>> uploadDockInfos) {
  return docks_view_->uploadById(uploadDockInfos);
}
//docks-get
bool MapWidget::getdockViewInfo(std::string &mapname) {
  Res_docks_.clear();
  LonAndLat mapinit_lonlat = pointcloudmap_.getMapLatLon();
  if (mapinit_lonlat.isInit) {
    Res_docks_ = docks_view_->pullMapDockInfo(mapname, mapinit_lonlat.latitude, mapinit_lonlat.longitude);
    if (Res_docks_.size() != 0) {
      return true;
    } else {
      return false;
    }
  }
}
//docks-id
std::string MapWidget::getBlockId() {
  return docks_view_->getBlockId();
}
//桩点数据
std::vector<MapDockInfo> MapWidget::docksViewData() {
  return Res_docks_;
}
//docks显示
void MapWidget::viewerDocksPoint(std::vector<MapDockInfo> &Res_docks, Color color) {
  // int num = 0;
  for (size_t i = 0; i < Res_docks.size(); i++) {
    Color showColor = color;
    // std::vector<Ogre::Vector3> points;
    Ogre::Vector3 pt;
    Ogre::Quaternion ot;
    OrgePose pq;
    for (size_t i = 0; i < 3; i++) {
      std::cout << "test get pose-----------: " << Res_docks[i].post(i) << std::endl;
    }
    pt.x = Res_docks[i].post(0);
    pt.y = Res_docks[i].post(1);
    pt.z = Res_docks[i].post(2);
    pq.point = pt;

    Eigen::Quaterniond orient(Res_docks[i].q.w(), Res_docks[i].q.x(),
                              Res_docks[i].q.y(), Res_docks[i].q.z());
    // 初始化欧拉角
    Eigen::Vector3d eulerAngle = orient.matrix().eulerAngles(2, 1, 0);
    //欧拉角转旋转向量，矩阵，四元数
    Eigen::AngleAxisd rollAngle(
        Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(
        Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(
        Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()));
    Eigen::AngleAxisd pitchAngle_pre(
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle * pitchAngle_pre;
    Ogre::Quaternion ogre_orient(quaternion.w(), quaternion.x(),
                                  quaternion.y(), quaternion.z());
    pq.quaternion = ogre_orient;
    // points.push_back(pt);
    std::string id = std::to_string(i);
    std::string info = "桩位: " + Res_docks[i].name;
    Ogre::ColourValue c = createColorRGBA(showColor);
    std::cout << "info-----------: " << info << std::endl;
    // render_panel_->createArrowObject()
    render_panel_->createArrowObject(id, pq, 1.0f, 1.0f, 1.3f, 1.3f, c);
    // text
    // Color textColor = Color::GREEN;
    // Ogre::ColourValue tc = createColorRGBA(textColor);
    // render_panel_->createTextObject(id, info, pt, 0.8, tc);
    // num=num+1;
  }
}
void MapWidget::udpateShowDocksData() {
  viewerDocksPoint(Res_docks_, Color::WHITE);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//初始定位点
void MapWidget::viewerInitPoint(std::vector<std::string> &init_data, Color color) {
  // int num = 0;
  // for (size_t i = 0; i < init_data.size(); i++) {
    Color showColor = color;
    // std::vector<Ogre::Vector3> points;
    Ogre::Vector3 pt;
    Ogre::Quaternion ot;
    OrgePose pq;
    for (size_t i = 0; i < 3; i++) {
      std::cout << "test get pose-----------: " << init_data[1] << " " << init_data[2] << " " << init_data[3] << std::endl;
    }
    pt.x = std::stof(init_data[Po_F::XF]);
    pt.y = std::stof(init_data[Po_F::YF]);
    pt.z = std::stof(init_data[Po_F::ZF]);
    pq.point = pt;

    Eigen::Quaterniond orient(std::stof(init_data[Po_F::Qwf]), std::stof(init_data[Po_F::Qxf]),
                              std::stof(init_data[Po_F::Qyf]), std::stof(init_data[Po_F::Qzf]));
    std::cout << "initpose : " << std::stof(init_data[Po_F::Qwf]) << " "
              << std::stof(init_data[Po_F::Qxf]) << " "
              << std::stof(init_data[Po_F::Qyf]) << " "
              << std::stof(init_data[Po_F::Qzf]) << std::endl;
    // 初始化欧拉角
    Eigen::Vector3d eulerAngle = orient.matrix().eulerAngles(2, 1, 0);
    //欧拉角转旋转向量，矩阵，四元数
    Eigen::AngleAxisd rollAngle(
        Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(
        Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(
        Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()));
    Eigen::AngleAxisd pitchAngle_pre(
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle * pitchAngle * rollAngle * pitchAngle_pre;
    Ogre::Quaternion ogre_orient(quaternion.w(), quaternion.x(),
                                  quaternion.y(), quaternion.z());
    std::cout << "initpose : " << quaternion.w() << " "
              << quaternion.x() << " "
              << quaternion.y() << " "
              << quaternion.z() << std::endl;
    // std::cout << "ogre orient: " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << std::endl;
    pq.quaternion = ogre_orient;
    std::cout << "initpose : " << ogre_orient.w << " "
              << ogre_orient.x << " "
              << ogre_orient.y << " "
              << ogre_orient.z << std::endl;
    // points.push_back(pt);
    std::string id = std::to_string(100);
    std::string info = "桩位: initPose";
    Ogre::ColourValue c = createColorRGBA(showColor);
    std::cout << "info-----------: " << info << std::endl;
    // render_panel_->createArrowObject()
    render_panel_->createArrowObject(id, pq, 1.0f, 1.0f, 1.3f, 1.3f, c);
}
void MapWidget::udpateShowInitData(std::vector<std::string> &init_data) {
  viewerInitPoint(init_data, Color::YELLOW);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//vehicle-mysql-view 数据库 车辆信息获取 显示
std::set<std::string> MapWidget::getCurrentRobotId(QString projectname) {
  return docks_view_->getRobotId(projectname);
}
bool MapWidget::getVehicleViewInfo(QString projectname,std::set<std::string> currentrobot) {
  vehicle_info_.clear();
  vehicle_info_ = docks_view_->pullMapVehiclaMysqlInfo(projectname,currentrobot);
  if (vehicle_info_.size() != 0) {
    return true;
  }
  return false;
}
void MapWidget::closeMysqlVehical() {
  docks_view_->closeMysql();
}
// void MapWidget::initMysqlVehical() {
//   std::cout << "----------------" << std::endl;
//   docks_view_->initMysql();
// }
std::unordered_map<std::string, std::vector<std::string>> MapWidget::getVehicleRobotName() {
  std::vector<std::string> robots_name;
  std::unordered_map<std::string, std::vector<std::string>> robots_info;
  if (vehicle_info_.size() != 0) {
    for (size_t i = 0; i < vehicle_info_.size(); i++) {
      robots_name.push_back(vehicle_info_[i].robot_id);
    }
    if (robots_name.size() != 0) {
      robots_info.insert(std::make_pair(vehicle_info_[0].map_name,robots_name));
    }
  }
  return robots_info;
}
void MapWidget::viewerVehicleData(std::vector<MapInfo> &vehicle_info, 
                                  std::string &robotname, Color color) {
    for (size_t i = 0; i < vehicle_info.size(); i++)
    {
      std::cout << "view robot-------------------:" << robotname << std::endl;
      if (vehicle_info[i].robot_id == robotname)
      {
        Color showColor = color;
        // std::vector<Ogre::Vector3> points;
        Ogre::Vector3 pt;
        OrgePose pq;
        std::cout << "test get pose-----------: " << vehicle_info[i].point.pose_pos_x << std::endl;

        pt.x = vehicle_info[i].point.pose_pos_x;
        pt.y = vehicle_info[i].point.pose_pos_y;
        pt.z = vehicle_info[i].point.pose_pos_z;
        pq.point = pt;

        Eigen::Quaterniond orient(vehicle_info[i].point.pose_orient_w, vehicle_info[i].point.pose_orient_x,
                                  vehicle_info[i].point.pose_orient_y, vehicle_info[i].point.pose_orient_z);
        // 初始化欧拉角
        Eigen::Vector3d eulerAngle = orient.matrix().eulerAngles(2, 1, 0);
        //欧拉角转旋转向量，矩阵，四元数
        Eigen::AngleAxisd rollAngle(
            Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(
            Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(
            Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()));
        Eigen::AngleAxisd pitchAngle_pre(
            Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond quaternion;
        quaternion = yawAngle * pitchAngle * rollAngle * pitchAngle_pre;
        Ogre::Quaternion ogre_orient(quaternion.w(), quaternion.x(),
                                      quaternion.y(), quaternion.z());
        pq.quaternion = ogre_orient;
        // points.push_back(pt);
        std::string id = std::to_string(i);
        std::string info = "robot_id:    " + vehicle_info[i].robot_id + ".\n"
                          "nav_version: " + vehicle_info[i].nav_version + ".\n"
                          "time:        " + vehicle_info[i].time + ".";
        Ogre::ColourValue c = createColorRGBA(showColor);
        std::cout << "info-----------: " << info << std::endl;
        // render_panel_->createArrowObject()
        render_panel_->createArrowObject(id, pq, 1.0f, 1.0f, 1.3f, 1.3f, c);
        // text
        Color textColor = Color::YELLOW;
        Ogre::ColourValue tc = createColorRGBA(textColor);
        render_panel_->createTextObject(id, info, pt, 0.8, tc);
      } else {
        continue;
      }
    }
}
void MapWidget::chooseVehicleView(std::vector<MapInfo> &vehicle_info, 
                                  std::set<std::string> &currentrobot, Color color) {
  if (currentrobot.size() != 0) {
    for (auto robot_id : currentrobot)
    {
      viewerVehicleData(vehicle_info,robot_id,color);
    }
  }
}
void MapWidget::updateShowVehicleData(std::set<std::string> &currentrobot) {
  chooseVehicleView(vehicle_info_, currentrobot, Color::RED);
}
/*地图信息
  --获取所有项目地图信息
  --增加项目
  --修改项目
*/
void MapWidget::mapDataGetFromMysql(std::vector<MapData> &vec_mapdata) {//获取
  docks_view_->mapDataGetFromMysql(vec_mapdata);
}
void MapWidget::mapDataUpdateFromMysql(MapData &mapdata) {//修改
  docks_view_->mapDataUpdateFromMysql(mapdata);
}
void MapWidget::mapDataIncreaseFromMysql(MapData &mapdata) {//新增
  docks_view_->mapDataIncreaseFromMysql(mapdata);
}
void MapWidget::mapDataDeleteFromMysql(MapData &mapdata) {//删除
  docks_view_->mapDataDeleteFromMysql(mapdata);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Ogre::ColourValue MapWidget::createColorRGBA(Color color) {
  // std_msgs::ColorRGBA color_rgba;
  float color_rgba_r = COLOR_VALUE_MIN;
  float color_rgba_g = COLOR_VALUE_MIN;
  float color_rgba_b = COLOR_VALUE_MIN;
  float color_rgba_a = COLOR_VALUE_MAX;

  switch (color) {
    case BLACK:
      color_rgba_r = 0.0f;
      color_rgba_b = 0.0f;
      color_rgba_g = 0.0f;
      break;
    case GRAY:
      color_rgba_r = COLOR_VALUE_MEDIAN;
      color_rgba_g = COLOR_VALUE_MEDIAN;
      color_rgba_b = COLOR_VALUE_MEDIAN;
      break;
    case LIGHT_RED:
      color_rgba_r = COLOR_VALUE_LIGHT_HIGH;
      color_rgba_g = COLOR_VALUE_LIGHT_LOW;
      color_rgba_b = COLOR_VALUE_LIGHT_LOW;
      break;
    case LIGHT_GREEN:
      color_rgba_r = COLOR_VALUE_LIGHT_LOW;
      color_rgba_g = COLOR_VALUE_LIGHT_HIGH;
      color_rgba_b = COLOR_VALUE_LIGHT_LOW;
      break;
    case LIGHT_BLUE:
      color_rgba_r = COLOR_VALUE_LIGHT_LOW;
      color_rgba_g = COLOR_VALUE_LIGHT_LOW;
      color_rgba_b = COLOR_VALUE_LIGHT_HIGH;
      break;
    case LIGHT_YELLOW:
      color_rgba_r = COLOR_VALUE_LIGHT_HIGH;
      color_rgba_g = COLOR_VALUE_LIGHT_HIGH;
      color_rgba_b = COLOR_VALUE_LIGHT_LOW;
      break;
    case LIGHT_CYAN:
      color_rgba_r = COLOR_VALUE_LIGHT_LOW;
      color_rgba_g = COLOR_VALUE_LIGHT_HIGH;
      color_rgba_b = COLOR_VALUE_LIGHT_HIGH;
      break;
    case LIGHT_MAGENTA:
      color_rgba_r = COLOR_VALUE_LIGHT_HIGH;
      color_rgba_g = COLOR_VALUE_LIGHT_LOW;
      color_rgba_b = COLOR_VALUE_LIGHT_HIGH;
      break;
    case RED:
      color_rgba_r = COLOR_VALUE_MAX;
      color_rgba_g = 0.0f;
      color_rgba_b = 0.0f;
      break;
    case GREEN:
      color_rgba_r = 0.0f;
      color_rgba_g = COLOR_VALUE_MAX;
      color_rgba_b = 0.0f;
      break;
    case BLUE:
      color_rgba_r = 0.0f;
      color_rgba_b = COLOR_VALUE_MAX;
      color_rgba_g = 0.0f;

      break;
    case YELLOW:
      color_rgba_r = COLOR_VALUE_MAX;
      color_rgba_g = COLOR_VALUE_MAX;
      color_rgba_b = 0.0f;
      break;
    case CYAN:
      color_rgba_r = 0.0f;
      color_rgba_g = COLOR_VALUE_MAX;
      color_rgba_b = COLOR_VALUE_MAX;
      break;
    case MAGENTA:
      color_rgba_r = COLOR_VALUE_MAX;
      color_rgba_b = COLOR_VALUE_MAX;
      color_rgba_g = 0.0f;
      break;
    case WHITE:
      color_rgba_r = COLOR_VALUE_MAX;
      color_rgba_g = COLOR_VALUE_MAX;
      color_rgba_b = COLOR_VALUE_MAX;
      break;
    default:
      color_rgba_r = 0.0f;
      color_rgba_b = 0.0f;
      color_rgba_g = 0.0f;  // hide color from view
      break;
  }
  return Ogre::ColourValue(color_rgba_r, color_rgba_g, color_rgba_b, 1.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*task control
*/
void MapWidget::saveCsvMapFiles(int id) {
  std::cout << "保存全部文件路径：—————————————————— " << filePath_vmap_.toStdString() << std::endl;
  QString suffixName = ".csv";
  QString path_str;
  if (!getVectorType((Vm_T)id).empty()) {
      path_str = QFileDialog::getSaveFileName(
          this, tr("保存文件"),
          filePath_vmap_ + QString(type_name_str[id].c_str()) + suffixName,
          tr("ALL(*.*);;CSV(*.csv)"));
      // std::cout << "地图路径：—————————————————— " << filePath_vmap_.toStdString() << std::endl;
      // std::cout << "地图整体路径：—————————————————— " << path_str.toStdString() << std::endl;
      if (path_str.isEmpty()) return;
      tools_->removeOldFiles(path_str);
      //保存普通csv文件
      writeVmapFile(path_str, id);
  }
  while (!path_str.endsWith('/') && path_str.size() > 0) {
      path_str.remove(path_str.size() - 1, 1);
  }
  filePath_vmap_ = filePath(path_str);
  WriteSettings();
}
//保存清扫区以及json文件
std::string MapWidget::saveCsvJsonFiles(std::string name, int id, std::string blockId) {
  std::cout << "保存全部文件路径：—————————————————— " << filePath_vmap_.toStdString() << std::endl;
  QString suffixName = ".json";
  QString path_str;
  std::string uploadArea;

  if (!getVectorType((Vm_T)id).empty()) {
      path_str = QFileDialog::getSaveFileName(
          this, tr("保存文件"),
          filePath_vmap_ + QString(type_name_str[id].c_str()) + suffixName,
          tr("ALL(*.*);;JSON(*.json)"));

      std::string json_path = path_str.toStdString();
      uploadArea = writeVmapJson(name, json_path, id, blockId);
  }
  while (!path_str.endsWith('/') && path_str.size() > 0) {
      path_str.remove(path_str.size() - 1, 1);
  }
  filePath_vmap_ = filePath(path_str);
  WriteSettings();
  return uploadArea;
}
void MapWidget::saveOsmMapFiles() {
  QString suffixName = ".osm";
  std::string filename = "hdmap";
  if (laneletmapEmpty()) {
    // std::cout << "....no lanelet map save...." << std::endl;
    return;
  }
  QString path_str = QFileDialog::getSaveFileName(
      this, tr("保存文件"),
      filePath_vmap_ + QString(filename.c_str()) + suffixName,
      tr("ALL(*.*);;OSM(*.osm)"));
  if (path_str.isEmpty()) {
    return;
  } else {
    writeLaneletMapRegular(regularinfo_list_vec_);
    writeLaneletMapCustomizeTags(tag_list_vec_);
    tools_->removeOldFiles(path_str);
    saveLaneletMap(path_str.toStdString());
    clearRegelem();
    filePath_pcd_ = filePath(path_str);
    WriteSettings();
  }
}
void MapWidget::saveGidFiles(QList<QMap<QString,QList<QString>>> &lists) {
  // std::cout << "保存全部文件路径：—————————————————— " << filePath_vmap_.toStdString() << std::endl;
  QString suffixName = "gid.csv";
  QString path_str;
  if (lists.size() != 0) {
      path_str = QFileDialog::getSaveFileName(
          this, tr("保存文件"),
          filePath_vmap_ + suffixName, tr("ALL(*.*);;CSV(*.csv)"));
      //保存gid文件
      writeGidFile(path_str,lists);
  }
  else {
    return;
  }
  while (!path_str.endsWith('/') && path_str.size() > 0) {
      path_str.remove(path_str.size() - 1, 1);
  }
  filePath_vmap_ = filePath(path_str);
  WriteSettings();
}
void MapWidget::autoSaveFile(std::string &path) {
  // write regular to map_
  writeLaneletMapRegular(regularinfo_list_vec_);
  // write tags to map_
  writeLaneletMapCustomizeTags(tag_list_vec_);
  // map_ to map_o_ and map_o_ save
  saveLaneletMap(path);
  // regular out from map_
  clearRegelem();
}
void MapWidget::loadReguandTagInfo() {
  // read regular to vector,and delet laneletLayer's and map'regularLayer
  // regular 先排序再去重
  regularinfo_list_vec_ = readLaneletMapRegular();
  sort(regularinfo_list_vec_.begin(), regularinfo_list_vec_.end(),
       [](reguinfo_list a, reguinfo_list b) {
         return a.reglists.Regelement_id < b.reglists.Regelement_id;
       });
  regularinfo_list_vec_.erase(
      unique(regularinfo_list_vec_.begin(), regularinfo_list_vec_.end(),
             [](reguinfo_list a, reguinfo_list b) {
               return a.reglists.Regelement_id == b.reglists.Regelement_id;
             }),
      regularinfo_list_vec_.end());
  // read Customize Tags
  tag_list_vec_ = readLaneletMapCustomizeTags();
}
QString MapWidget::readOnlyPcdMapFile(QString &id,std::vector<std::string> &paths) {
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  // Eigen::Vector4f origin;
  // Eigen::Quaternionf orientation;
  bool pcd_flag;
  // map_paths_.push_back(path.toStdString());
  // if (paths.size() <= 0) return filePath_pcd_;
  if (paths.size() == 1) {
    std::cout << "加载一个" << std::endl;
    pcd_flag = initMapOriginPostion(paths.front());
  } else {
    pcd_flag = initMapOriginPostion(paths);
  }
  if (!pcd_flag) {
    int reply = QMessageBox::warning(
        this, QObject::tr("提示"),
        QObject::tr("初始经纬度为0, 您要继续使用吗!!"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if (reply == QMessageBox::No) {
      return filePath_pcd_;
    } else {
      loadPcdMap("0");
    }
  } else {
    loadPcdMap("0");
  }
  filePath_pcd_ = filePath(QString::fromStdString(paths.back()));
  WriteSettings();
  return filePath_pcd_;
}
QString MapWidget::readVmapFile(QStringList &pronames) {
  if (pronames.size() < 1) return filePath_vmap_;
  near_points_.clear();
  
  for(auto &file : pronames){
      if(tools_->isCsvEcreptFile(file)){
      tools_->fileDecrept(file);
      }
  }
  loadVmap(pronames);
  tools_->removeFiles(remove_flag_,pronames);
  filePath_vmap_ = filePath(pronames.back());
  WriteSettings();
  return filePath_vmap_;
}
QString MapWidget::readAllVmapFile(QStringList &fileNames, QList<QMap<QString,QList<QString>>> &lists) {
  if (fileNames.size() < 1) return filePath_vmap_;
  near_points_.clear();
  QStringList fileNames_scv;
  QString fileName_osm;
  QString fileName_gid;

  for (auto file : fileNames)
  {
    if (tools_->isGidFile(file))
    {
      fileName_gid = file;
    }
    else if(tools_->isCsvFile(file)){
      fileNames_scv.push_back(file);
    }
    else if(tools_->isCsvEcreptFile(file)){
      tools_->fileDecrept(file);
      fileNames_scv.push_back(file);
    }
    else if(tools_->isOsmFile(file)){
      fileName_osm = file;
    }
    else if(tools_->isOsmEcreptFile(file)){
      tools_->fileDecrept(file);
      fileName_osm = file;
    }
  }
  seceneDocksRemove();
  render_panel_->seceneClearareRemove();
  //load vmap
  if (fileNames_scv.size() > 0) {
    loadVmap(fileNames_scv);
    tools_->removeFiles(remove_flag_,fileNames);
    filePath_vmap_ = filePath(fileNames_scv.back());
    WriteSettings();
  }
  if (fileName_gid != "")
  {
    readGidFile(fileName_gid,lists);
  }
  
  clearAllLanelet2map();
  if (fileName_osm != "") {
    loadLaneletMap(fileName_osm);
    tools_->removeFiles(remove_flag_,fileName_osm);
    loadReguandTagInfo();
  }
  return filePath_vmap_;
}
void MapWidget::readGidFile(QString &str, QList<QMap<QString,QList<QString>>> &lists) {
  if (str.isEmpty()) return;
  vmapdata_.readGidFile(str,lists);
}
QString MapWidget::filePath(QString file) {
  QString file_ = file;
  while (!file_.endsWith('/') && file_.size() > 0) {
    file_.remove(file_.size() - 1, 1);
  }
  if (file_.size() > 0) {
    return file_;
  } else {
    return "/home";
  }
}
/*QSettings读写配置文件
TODO 可以把界面配置参数保存在一个配置文件中...
*/
void MapWidget::ReadSettings(void) {
  QSettings settings("Qt-Osg Package", "mapdrawing_tool");
  filePath_pcd_ = settings.value("pcd_path", QString("/home/")).toString();//读值
  filePath_vmap_ = settings.value("osm_path", QString("/home/")).toString();
  filePath_vmap_ = settings.value("csv_path", QString("/home/")).toString();
  filePath_frame_ = settings.value("frame_path", QString("/home/")).toString();
}
void MapWidget::WriteSettings(void) {
  QSettings settings("Qt-Osg Package", "mapdrawing_tool");
  settings.setValue("pcd_path", filePath_pcd_);//赋值
  settings.setValue("osm_path", filePath_vmap_);
  settings.setValue("csv_path", filePath_vmap_);
  settings.setValue("frame_path", filePath_frame_);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//slam-data keyframe
//获取每一帧数据解析
bool MapWidget::get_keyframes(const std::string& directory, std::deque<KeyFrame> &key_frames)
{   
    return docks_view_->get_keyframes(directory,key_frames);
}
void MapWidget::get_QuaterAndVector(const QString& directory, 
                                    boost::optional<Eigen::Vector3d> &ZERO_GNSS) {
    docks_view_->get_QuaterAndVector(directory,ZERO_GNSS);
}
//查找最近点的高度
double MapWidget::findCurrentNearPointHeight(Way_Point_ &pose, double distance, std::deque<KeyFrame> &key_frames) {
  double len = std::numeric_limits<double>::max();
  double height = std::numeric_limits<double>::max();
  std::cout << "keyframes size: " << key_frames.size() << std::endl;
  for (size_t i = 0; i < key_frames.size(); i++) {
    Eigen::Vector3d vector3d = key_frames[i].estimate->translation();
    double l = pow((pose.point.x - vector3d[0]), 2) +
               pow((pose.point.y - vector3d[1]), 2);
    if (len > fabs(l)) {
      len = fabs(l);
      if (len < distance)
      {
        height = vector3d[2];
      }
      // height = frame_poses[i].z;
    }
  }
  std::cout << "最近height++++++++++++++++=： " << height << std::endl;
  return height;
}
double MapWidget::findCurrentNearPointHeight(Way_Point_ &pose, double distance, std::vector<Way_Point_> &keyframes_poses) {
  double len = std::numeric_limits<double>::max();
  double height = std::numeric_limits<double>::max();
  for (size_t i = 0; i < keyframes_poses.size(); i++) {
    Way_Point_ wp = keyframes_poses[i];
    double l = pow((pose.point.x - wp.point.x), 2) +
               pow((pose.point.y - wp.point.y), 2);
    if (len > fabs(l)) {
      len = fabs(l);
      if (len < distance)
      {
        height = wp.point.z;
      }
      // height = frame_poses[i].z;
    }
  }
  std::cout << "最近height++++++++++++++++=： " << height << std::endl;
  return height;
}
bool MapWidget::savePointCloudPcdFile(QString &project_name,
                                      std::deque<KeyFrame> &key_frames,
                                      boost::optional<Eigen::Vector3d> &map_zero_gnss,
                                      bool downsample_flag,
                                      double downsample_resolution, 
                                      bool saveCompressedPCD) {                              
  std::cout << "保存全部文件路径：—————————————————— " << filePath_pcd_.toStdString() << std::endl;
  QString suffixName;
  if (downsample_flag)
  {
    filePath_pcd_ = QString::fromStdString(HOME_STR) + "/maptool-candela/CTI_MAP_PROJECT/" + project_name + "/MAP/" + project_name + "/";
    suffixName = project_name + ".pcd";
  } else {
    filePath_pcd_ = QString::fromStdString(HOME_STR) + "/maptool-candela/CTI_MAP_PROJECT/" + project_name + "/RMAP/" + project_name + "/";
    suffixName = project_name + "_r.pcd";
  }

  QString path_str = filePath_pcd_ + suffixName;
  std::cout << "最终路径：" << path_str.toStdString() << std::endl;
  QDir dir(filePath_pcd_);
  if(!dir.exists())
  {
    qDebug() << "路径不存在.";
    return false;
  }
  //不存在当前目录，创建，可创建多级目录 //mkdir只能创建一级目录
  // bool ok = dir.mkpath(path_str);

  //移除旧文件
  tools_->removeOldFile(path_str);
  //保存
  bool result = docks_view_->save_pointcloud(path_str.toStdString(), key_frames, map_zero_gnss, downsample_flag, downsample_resolution, saveCompressedPCD);
  //权限
  std::string cmd = "chmod 644 " + path_str.toStdString();
  system(cmd.c_str());

  while (!path_str.endsWith('/') && path_str.size() > 0) {
      path_str.remove(path_str.size() - 1, 1);
      // std::cout << "处理后地图整体路径: ——————————————————" << path_str.toStdString() << std::endl;
  }
  filePath_pcd_ = filePath(path_str);
  WriteSettings();
  return result;
}
bool MapWidget::writeKeyFramesFiles(QString &path_str, std::vector<Way_Point_> &keyframes_poses) {
  if (!path_str.isEmpty())
  {
    QString filepath = path_str + "keyframes.csv";
    tools_->removeOldFile(filepath);
  }
  return docks_view_->writeKeyFramesFiles(path_str,keyframes_poses);
}
QString MapWidget::readKeyFramesFiles(QString &file_Path, std::vector<Way_Point_> &keyframes_poses, int &num) {
  keyframes_poses.clear();
  docks_view_->readKeyFramesFiles(file_Path,keyframes_poses,num);
  filePath_frame_ = filePath(file_Path);
  WriteSettings();
  return filePath_frame_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MapWidget::setPlatformRequestDomain(std::string domain) {
  docks_view_->setPlatformRequestDomain(domain);
}
void MapWidget::setPlatformRequesauthorizationToken(std::string authorizationToken) {
  docks_view_->setPlatformRequesauthorizationToken(authorizationToken);
}
std::string MapWidget::pullParkToken(std::string &user, std::string &password, std::string &keyDir) {
  return docks_view_->pullParkToken(user,password,keyDir);
}
QList<QMap<QString, QString>> MapWidget::pullByToken(std::string authorizationToken) {
  return docks_view_->pullByToken(authorizationToken);
}
QList<QMap<int, QString>> MapWidget::getSignalDataIdFromParkId(std::string authorizationToken,
                                                               std::string parkId, 
                                                               std::string dataType) {
  return docks_view_->getSignalDataIdFromParkId(authorizationToken,parkId,dataType);
}
//四元素->弧度->角度
void MapWidget::post_initia_dock(std::string &map_id,
                                 Eigen::Matrix4d pose, 
                                 std::string authorizationToken) { // boost::optional<Eigen::Vector3d> mapZeroGnss
  std::vector<double> initial_DockInfo;
  double lat, lon;
  double dire;
  map_projection_reference_s mapOriginPos = pointcloudmap_.getMapOriginPostion();
  // std::string map_id = getBlockId();
  map_projection_reproject(&mapOriginPos, pose(1, 3), pose(0, 3), &lat, &lon);
  {
    Eigen::Quaterniond pose_q;
    pose_q = pose.block<3, 3>(0, 0);
    dire = docks_view_->getDirection(pose_q);
    std::cout << "direction: " << dire << " lat: " << lat << " lon:" << lon << std::endl;
  }
  initial_DockInfo.push_back(lat);
  initial_DockInfo.push_back(lon);
  initial_DockInfo.push_back(pose(2,3));
  initial_DockInfo.push_back(dire);
  docks_view_->post_initia_dock(map_id,authorizationToken,initial_DockInfo);
}
void MapWidget::postUploadAllData(std::string authorizationToken, std::string uploadData) {
  docks_view_->postUploadAllData(authorizationToken,uploadData);
}
void MapWidget::deleteAllData(std::string authorizationToken, 
                              std::string dataType, 
                              std::string parkId) {
  docks_view_->deleteAllData(authorizationToken,dataType,parkId);
}
void MapWidget::odinaryAllData(std::string authorizationToken, 
                              std::string uploadData, 
                              std::string parkId) {
  docks_view_->odinaryAllData(authorizationToken,uploadData,parkId);
}
void MapWidget::deletSignalDataFromId(std::string authorizationToken, 
                                      std::string id) {
  docks_view_->deletSignalDataFromId(authorizationToken,id);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*方向计算dire：
  --吹扫区方向
  --偏移方向
  --所有点位角度显示*/
std::tuple<Ogre::Quaternion, double> MapWidget::angleToOuaterGetDire(float angle) {
    Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
    Ogre::Quaternion ogrequater = Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x;
    std::cout << "pose : " << ogrequater.w << " "
              << ogrequater.x << " "
              << ogrequater.y << " "
              << ogrequater.z << std::endl;
    Eigen::Quaterniond orient(ogrequater.w, ogrequater.x,
                              ogrequater.y, ogrequater.z);
    
    Eigen::Vector3d eulerAngle = orient.matrix().eulerAngles(2, 1, 0);
    //欧拉角转旋转向量，矩阵，四元数
    Eigen::AngleAxisd rollAngle(
        Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(
        Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(
        Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()));
    Eigen::AngleAxisd pitchAngle_pre(
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
    Eigen::Quaterniond pose_q;
    pose_q = yawAngle * pitchAngle * rollAngle * pitchAngle_pre;
    // Ogre::Quaternion ogre_orient(quaternion.w(), quaternion.x(),
    //                               quaternion.y(), quaternion.z());
    std::cout << "pose : " << pose_q.w() << " "
              << pose_q.x() << " "
              << pose_q.y() << " "
              << pose_q.z() << std::endl;
    double dire = -(docks_view_->getDirection(pose_q));
    // std::cout << "当前点角度： " << dire << std::endl;
    std::tuple<Ogre::Quaternion, double> result = std::make_tuple(ogrequater, dire);
    return result;
}
void MapWidget::viewerDeviationPose(PurgeArrow_ &pa, Color color, int info) {
  // int num = 0;
  for (size_t i = 0; i < pa.wps.size(); i++) {
    Color showColor = color;
    // std::vector<Ogre::Vector3> points;
    Ogre::Vector3 pt;
    OrgePose pq;
      std::cout << "test get pose-----------: " << pa.wps[i].point.x << " " << pa.wps[i].point.y << " " 
                << pa.wps[i].point.z << std::endl;
    pt.x = pa.wps[i].point.x;
    pt.y = pa.wps[i].point.y;
    pt.z = pa.wps[i].point.z;
    pq.point = pt;

    pq.quaternion = pa.quats[i];
    // points.push_back(pt);
    std::string id = std::to_string(info*1000+i);
    Ogre::ColourValue c = createColorRGBA(showColor);
    render_panel_->createLineArrowObject(id, pq, 1.0f, 0.5f, 1.3f, 0.9f, c);
  }
}
void MapWidget::getAndShowDeviationPose(std::vector<Way_Point_> &poses, double offset, int info) {
    PurgeArrow_ pa;
    double dire;
    Ogre::Quaternion ogrequater;
    std::vector<Way_Point_> target_poses;
    std::vector<Ogre::Quaternion> ogrequaters;
    for (size_t i = 0; i < poses.size(); i++)
    {
      double quate_w = poses[i].point.quate_w;
      std::tie(ogrequater, dire) = angleToOuaterGetDire(quate_w);
      ogrequaters.push_back(ogrequater);
      target_poses.push_back(docks_view_->setPoseFromDeviation(dire,poses[i],offset));
    }
    pa.wps = target_poses;
    pa.quats = ogrequaters;
    // std::cout << "targrt_pose: " << target_pose.point.x << " " << target_pose.point.y << " " << target_pose.point.z << std::endl;
    viewerDeviationPose(pa,Color::RED,info);
}
std::vector<Way_Point_> MapWidget::getDeviationPose(double dire, std::vector<Way_Point_> &poses, double offset) {
    return docks_view_->setPoseFromDeviation(dire,poses,offset);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MapDialog::MapDialog(){};
void MapDialog::keyPressEvent(QKeyEvent *evt) {
  if (evt->key() == Qt::Key_Enter || evt->key() == Qt::Key_Return) return;
  QDialog::keyPressEvent(evt);
}
