#ifndef LANELET2_DATA_H
#define LANELET2_DATA_H

//所需要包含的头文件
// #include <geometry_msgs/Pose.h>
#include <lanelet2/lanelet2_core/Attribute.h>
#include <lanelet2/lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2/lanelet2_core/geometry/Lanelet.h>
#include <lanelet2/lanelet2_core/geometry/Point.h>
#include <lanelet2/lanelet2_core/primitives/Lanelet.h>
#include <lanelet2/lanelet2_io/Io.h>
#include <lanelet2/lanelet2_io/io_handlers/Factory.h>
#include <lanelet2/lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2/lanelet2_io/io_handlers/Writer.h>
#include <lanelet2/lanelet2_projection/UTM.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <tf/tf.h>

#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

#include "geo/geo.h"
// #include "spline.h"

using GenericRegulatoryElementPtr =
    std::shared_ptr<lanelet::GenericRegulatoryElement>;
struct LatLonInfo {
  double latitude{0.0f};
  double longitude{0.0f};
  double altitude{0.0f};
  double orientation{0.0f};
};
struct pcd_xyz_grid {
  std::string filename;
  std::string name;
  int grid_id;
  int grid_id_x;
  int grid_id_y;
  int lower_bound_x;
  int lower_bound_y;
  int upper_bound_x;
  int upper_bound_y;
  pcl::PointCloud<pcl::PointXYZI> cloud;
};
struct gridxy {
  double minX;
  double maxX;
  double minY;
  double maxY;
};
struct gps_point_ {
  double x;  //坐标
  double y;
  double z;
  double quate_x;  //四元素
  double quate_y;
  double quate_z;
  double quate_w;
};
struct mapelement_point {
  int id;
  std::string type;
  lanelet::Point3d point;
};
struct mapelement_lanelet {
  int id;
  std::string type;
  lanelet::Lanelet lanelet;
};
struct mapelement_line {
  int id;
  std::string type;
  lanelet::LineString3d line;
};
struct mapelement_polygon {
  int id;
  std::string type;
  lanelet::Polygon3d poly;
};
struct maptypes {
  std::vector<mapelement_point> pointslayer;
  std::vector<mapelement_line> linestringslayer;
  std::vector<mapelement_lanelet> laneletslayer;
  std::vector<mapelement_polygon> polygonslayer;
};
struct tag {
  std::string k;
  std::string v;
};
struct tag_list {
  int id;
  std::vector<tag> tags;
};
struct refer_list {
  int refer_id;
  std::string referName;
};
struct regelement_list {
  using Id = int64_t;
  Id Regelement_id;
  std::vector<refer_list> refers_list;
  std::string maneuver;
  std::string element;
};
struct reguinfo_list {
  regelement_list reglists;
  int laneletid;
  int regmemberid;
  std::string Subtype;
};

class regul_junction : public lanelet::RegulatoryElement {
 public:
  static constexpr char RuleName[] = "junction";
  using Ptr = std::shared_ptr<regul_junction>;
  lanelet::ConstPolygon3d fromWhere() const {
    return getParameters<lanelet::ConstPolygon3d>(lanelet::RoleName::Refers)
        .front();
  }
  static Ptr make(lanelet::Id id, lanelet::Polygon3d fromWhere) {
    return Ptr{new regul_junction(id, fromWhere)};
  };

 private:
  regul_junction(lanelet::Id id, lanelet::Polygon3d fromWhere)
      : RegulatoryElement{
            std::make_shared<lanelet::RegulatoryElementData>(id)} {
    parameters().insert({lanelet::RoleNameString::Refers, {fromWhere}});
    // parameters().insert({lanelet::RoleNameString::Test, {fromWhere}});
  }

  friend class lanelet::RegisterRegulatoryElement<regul_junction>;
  explicit regul_junction(const lanelet::RegulatoryElementDataPtr &data)
      : RegulatoryElement(data) {}
};

class Lanelet2Data {
 public:
  Lanelet2Data();
  ~Lanelet2Data();
  bool loadLanelet2OSMfile(std::string file);
  bool exportLanelet2OSMfile(std::string file);
  lanelet::LaneletMapPtr getCurrentMap() { return this->map_; };
  lanelet::LaneletMapPtr getCurrentShowMap() { return this->maplayer_show_; };
  void create_show_map(double Zmin, double Zmax);
  void createPoint(lanelet::Point3d &point3d);
  lanelet::LineString3d createLinestring(
      std::vector<lanelet::Point3d> point3d_vec);
  void extendLinestring(lanelet::LineString3d &ls_3d, lanelet::Point3d &point3d);
  void extendLinestring_re(lanelet::LineString3d &ls_3d,
                           lanelet::Point3d &point3d);
  void shortcutLinestring(lanelet::LineString3d &ls_3d);

  lanelet::Polygon3d createPolygon(std::vector<lanelet::Point3d> point3d_vec);

  void extendPolygon(lanelet::Polygon3d &poly, lanelet::Point3d point3d);
  lanelet::Lanelet createLanelet(lanelet::LineString3d &left,
                                 lanelet::LineString3d &right);
  lanelet::Lanelet createLanelet(std::vector<lanelet::Point3d> point3d_vec);
  lanelet::Lanelet searchNearstLanelet(lanelet::BasicPoint2d &searchPoint);
  lanelet::Point3d searchNearstPoint(lanelet::BasicPoint2d &searchPoint);
  lanelet::LineString3d searchNearstLinestring(
      lanelet::BasicPoint2d &searchPoint);
  lanelet::Polygon3d searchNearstPolygon(lanelet::BasicPoint2d &searchPoint);
  void movePrimitive(lanelet::LineString3d &ls_3d, Eigen::Vector3d offset);
  void movePrimitive(lanelet::Point3d &point3d, Eigen::Vector3d offset);
  void clearMap() {
    this->map_.reset(new lanelet::LaneletMap());
    this->map_o_.reset(new lanelet::LaneletMap());
  };
  void extendLanelet(lanelet::Lanelet &Lanelet, const lanelet::Point3d point3d);
  std::vector<int> findNextLanelet(lanelet::Lanelet &Lanelet);
  void jointLanelet(lanelet::Lanelet &Lanelet1, lanelet::Lanelet &lanelet2);
  void rmPrimitive(lanelet::Lanelet &Lanelet);
  void rmPrimitive(lanelet::LineString3d &Linestring);
  void rmPrimitive(lanelet::Point3d &pt);
  void rmPrimitive(lanelet::Polygon3d &poly);
  std::vector<int> showPtsId(const lanelet::LineString3d &Linestring);
  std::vector<lanelet::Point3d> getPoints(lanelet::LineString3d Linestring);
  std::vector<lanelet::Point3d> getPoints(lanelet::Polygon3d polygon);
  void addLineAtrribute(lanelet::LineString3d &Linestring,
                        std::vector<tag> &tag_list);
  void addLaneletAtrribute(lanelet::Lanelet &Lanelet,
                           std::vector<tag> &tag_list);
  int addRegElement(lanelet::Lanelet &ll, lanelet::Polygon3d &junction);
  int addRegElement(lanelet::Lanelet &ll, lanelet::Polygon3d &junction,
                    GenericRegulatoryElementPtr &regelem);
  int addRegElement(lanelet::Lanelet &ll, lanelet::LineString3d &refline);
  int addRegElement(lanelet::Lanelet &ll, lanelet::LineString3d &refline,
                    GenericRegulatoryElementPtr &regelem);
  int addRegElement(lanelet::Lanelet &ll, lanelet::LineString3d &refline,
                    GenericRegulatoryElementPtr &regelem, std::string subtype);
  int addRegElement(lanelet::Lanelet &ll, lanelet::Polygon3d &junction,
                    GenericRegulatoryElementPtr &regelem, std::string subtype);
  void addLaneletPedestrian(lanelet::Lanelet &ll);
  void changeLaneletBound(lanelet::Lanelet &Lanelet);
  // void map_out();
  void map_out(std::vector<tag_list> &tag_lists);
  std::vector<tag_list> tagsFromRlanelet2map();
  maptypes mapElementFromRlanelet2map();
  void map_out();
  bool pose2gps(const gps_point_ &pose, LatLonInfo &gps);
  bool gps2pose(const LatLonInfo &gps, gps_point_ &pose);

  int addRegelemTest();
  void addReglist(std::vector<reguinfo_list> &reg_vec);
  std::vector<reguinfo_list> regelementload();
  void clearRegelem();

 public:
  // for cti_maptool
  bool mapEmpty();
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
  void moveFrontPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                        lanelet::Polygon3d poly);
  void moveFrontPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                        lanelet::LineString3d linestring);
  void moveBackPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                       lanelet::Polygon3d poly);
  void moveBackPoint3d(lanelet::Point3d point, lanelet::Point3d &new_point3d,
                       lanelet::LineString3d linestring);

  bool findLaneletWithId(int lanelet_id, lanelet::Lanelet &lanelet);
  bool findLinestringWithId(int linestring_id,
                            lanelet::LineString3d &linestring);
  bool findPoly3dWithId(int poly_id, lanelet::Polygon3d &poly);
  bool findPoint3dWithId(int point3d_id, lanelet::Point3d &point3d);
  bool isPoint3dinLinestringLayer(int point3d_id);
  bool isPoint3dinLinestring(int point3d_id, lanelet::LineString3d ll);
  bool isPoint3dinPoly(int point3d_id, lanelet::Polygon3d pl);
  void addRoadAtrribute(lanelet::Lanelet &llt);
  void addCrosswalkAtrribute(lanelet::Lanelet &llt);
  void addlinestringAtrribute(lanelet::LineString3d &linestring);
  void addStoplineAtrribute(lanelet::LineString3d &linestring);
  void addBumpAtrribute(lanelet::LineString3d &linestring);
  void getNewlineFromcurrentPoint(int pt_id);
  void getNearestStartOrEndpoint(lanelet::Point3d point);
  bool getBindPointAtLine(int ls_id, int pt_id, lanelet::Point3d find_point);
  bool getBindPointAtLine(int ls_id, int pt_id);
  bool haveCurrentRefIdRole(int ref_id, std::string &ref_role);
  void addCustomizeTags(std::vector<tag_list> &tag_lists);
  void findLinestringswithPoint3d(lanelet::LineStrings3d &linestrings,
                                  lanelet::Point3d point);
  void findLinestringswithEndPoint3d(lanelet::LineStrings3d &linestrings,
                                     lanelet::Point3d point);
  // view
  lanelet::ConstLanelets subtypeLanelets(const lanelet::ConstLanelets lls,
                                         const char subtype[]);
  lanelet::ConstLineStrings3d typeLineStrings(
      const lanelet::ConstLineStrings3d lts, const char type[]);
  lanelet::ConstPoints3d pointLayer();
  lanelet::ConstLineStrings3d lineStringLayer();
  lanelet::ConstLineStrings3d stoplineLineStrings(
      const lanelet::ConstLineStrings3d lts);
  lanelet::ConstLineStrings3d bumpLineStrings(
      const lanelet::ConstLineStrings3d lts);
  lanelet::ConstLineStrings3d stampLinethinLineStrings(
      const lanelet::ConstLineStrings3d lts,const lanelet::ConstLanelets lls);
  lanelet::ConstLanelets laneletLayer();
  lanelet::ConstLanelets crosswalkLanelets(const lanelet::ConstLanelets lls);
  lanelet::ConstLanelets roadLanelets(const lanelet::ConstLanelets lls);
  lanelet::ConstPolygons3d PolygonLayer();

 public:
  lanelet::LaneletMapPtr map_;
  map_projection_reference_s origin_pos_;
 private:
  lanelet::LaneletMapPtr maplayer_show_;
  lanelet::LaneletMapPtr map_o_;
  // std::vector<lanelet::Point3d> point3d_vec_;
  // lanelet::LineString3d ls_3d_;
};

// }  // end namespace lanelet2tool
// }  // end namespace cti
#endif  // Lanelet2Data_H
