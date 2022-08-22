#include "lanelet_data/lanelet2_data.h"

#include <lanelet2/lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2/lanelet2_core/geometry/Lanelet.h>
#include <lanelet2/lanelet2_core/geometry/Point.h>
#include <math.h>

// #include "../ExampleHelpers.h"

#pragma GCC diagnostic ignored "-Wunused-variable"

Lanelet2Data::Lanelet2Data() {
  map_ = std::make_shared<lanelet::LaneletMap>();
  map_o_ = std::make_shared<lanelet::LaneletMap>();
  maplayer_show_ = std::make_shared<lanelet::LaneletMap>();
}
Lanelet2Data::~Lanelet2Data() {}

bool Lanelet2Data::pose2gps(const gps_point_ &pose, LatLonInfo &gps) {
  bool ret = false;
  if (origin_pos_.init_done) {
    map_projection_reproject(&origin_pos_, pose.y, pose.x,
                             &gps.latitude, &gps.longitude);
    gps.altitude = pose.z;
    ret = true;
  }
  return ret;
}

bool Lanelet2Data::gps2pose(const LatLonInfo &gps, gps_point_ &pose) {
  bool ret = false;
  if (origin_pos_.init_done) {
    map_projection_project(&origin_pos_, gps.latitude, gps.longitude,
                           &pose.y, &pose.x);
    pose.z = gps.altitude;
    ret = true;
  }
  return ret;
}
bool Lanelet2Data::loadLanelet2OSMfile(std::string file) {
  lanelet::ErrorMessages errors;
  map_ = load(file, lanelet::Origin({0, 0}), &errors);
  // assert(errors.empty());
  if (errors.empty()) {
    return true;
  } else {
    return false;
  }
  return true;
}
bool Lanelet2Data::exportLanelet2OSMfile(std::string file) {
  std::cout << "befor write in exportlaneletmap !" << std::endl;
  try {
    lanelet::write(file, *map_o_);
    // lanelet::write(file, *map_);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }

  return true;
}

void Lanelet2Data::createPoint(lanelet::Point3d &point3d) {
  point3d.attributes()["type"] = "point";
  point3d.attributes()["subtype"] = " ";
  point3d.attributes()["local_x"] = point3d.x();
  point3d.attributes()["local_y"] = point3d.y();
  gps_point_ pose;
  pose.x = point3d.x();
  pose.y = point3d.y();
  pose.z = point3d.z();
  LatLonInfo latlon_info;
  if (pose2gps(pose, latlon_info)) {
    point3d.attributes()["gps_lon"] = latlon_info.longitude;
    point3d.attributes()["gps_lat"] = latlon_info.latitude;
  }
}

lanelet::LineString3d Lanelet2Data::createLinestring(
    std::vector<lanelet::Point3d> point3d_vec) {
  lanelet::LineString3d ls_3d;
  ls_3d = lanelet::LineString3d(lanelet::InvalId, point3d_vec);
  this->map_->add(ls_3d);
  createPoint(point3d_vec[0]);
  createPoint(point3d_vec[1]);
  return ls_3d;
}

void Lanelet2Data::extendLinestring(
    lanelet::LineString3d &ls_3d, lanelet::Point3d &point3d) {
  this->map_->add(point3d);
  createPoint(point3d);
  ls_3d.push_back(point3d);
  std::cout << "in lanelet data extendlinestring" << ls_3d.id() << std::endl;
}
void Lanelet2Data::extendLinestring_re(
    lanelet::LineString3d &ls_3d, lanelet::Point3d &point3d) {
  this->map_->add(point3d);
  createPoint(point3d);
  ls_3d.insert(ls_3d.begin(), point3d);
}

void Lanelet2Data::shortcutLinestring(lanelet::LineString3d &ls_3d) {
  int point_id = ls_3d.back().id();
  ls_3d.pop_back();
}

lanelet::Polygon3d Lanelet2Data::createPolygon(
    std::vector<lanelet::Point3d> point3d_vec) {
  lanelet::Polygon3d poly(lanelet::utils::getId(), point3d_vec);
  poly.attributes()["type"] = "intersection";
  // poly.attributes()["subtype"] = "road";
  this->map_->add(poly);
  createPoint(point3d_vec[0]);
  createPoint(point3d_vec[1]);
  createPoint(point3d_vec[2]);
  return poly;
}

void Lanelet2Data::extendPolygon(lanelet::Polygon3d &poly,
                                 lanelet::Point3d point3d) {
  this->map_->add(point3d);
  createPoint(point3d);
  poly.push_back(point3d);
}

lanelet::Lanelet Lanelet2Data::createLanelet(lanelet::LineString3d &left,
                                             lanelet::LineString3d &right) {
  lanelet::Lanelet Lanelet;
  std::cout << "lanelet id" << Lanelet.id() << std::endl;
  Lanelet = lanelet::Lanelet(lanelet::utils::getId(), left, right);
  map_->add(Lanelet);
  return Lanelet;
}

void Lanelet2Data::changeLaneletBound(lanelet::Lanelet &Lanelet) {
  std::vector<lanelet::Point3d> points_l;
  std::vector<lanelet::Point3d> points_r;
  for (auto it = Lanelet.leftBound().end() - 1; it >= Lanelet.leftBound().begin(); it--) {
    lanelet::Point3d pt = *it;
    points_l.push_back(pt);
  }
  lanelet::LineString3d right = lanelet::LineString3d(lanelet::InvalId, points_l);
  this->map_->add(right);
  addlinestringAtrribute(right);

  for (auto it = Lanelet.rightBound().end() - 1; it >= Lanelet.rightBound().begin(); it--) {
    lanelet::Point3d pt = *it;
    points_r.push_back(pt);
  }
  lanelet::LineString3d left = lanelet::LineString3d(lanelet::InvalId, points_r);
  this->map_->add(left);
  addlinestringAtrribute(left);

  rmPrimitive(Lanelet);
  lanelet::Lanelet ll = lanelet::Lanelet(lanelet::utils::getId(),left, right);
  map_->add(ll);
  addRoadAtrribute(ll);
}

lanelet::Lanelet Lanelet2Data::createLanelet(
    std::vector<lanelet::Point3d> point3d_vec) {
  // for waypoints
  std::vector<Eigen::Vector3d> waypoints;
  for (auto pt = point3d_vec.begin(); pt != point3d_vec.end(); pt++) {
    lanelet::Point3d p = *pt;
    Eigen::Vector3d wp = {p.x(), p.y(), p.z()};
    waypoints.push_back(wp);
  }
  lanelet::LineString3d left;
  lanelet::LineString3d right;
  double k;
  double b;
  double inc;
  double offset = 1.5;
  double x_l;
  double y_l;
  double x_r;
  double y_r;
  lanelet::Point3d point3d_l;
  lanelet::Point3d point3d_r;
  Eigen::Vector2d shift_vec;
  double lanelet_height = waypoints[0][2];

  if ((waypoints[1][0] - waypoints[0][0]) == 0) {
    waypoints[1][0] = waypoints[1][0] + 0.01;
    waypoints[0][0] = waypoints[0][0] - 0.01;
  }
  k = (waypoints[1][1] - waypoints[0][1]) / (waypoints[1][0] - waypoints[0][0]);
  b = waypoints[1][1] - k * waypoints[1][0];
  inc = (waypoints[1][0] - waypoints[0][0]) / 1;
  shift_vec = {-k * offset / sqrt(pow(k, 2) + 1),
               1 * offset / sqrt(pow(k, 2) + 1)};
  auto leftofvec = [](lanelet::Point3d pt, Eigen::Vector3d fst_pt,
                      Eigen::Vector3d scd_pt) {
    double tmp = (fst_pt[1] - scd_pt[1]) * pt.x() +
                 (scd_pt[0] - fst_pt[0]) * pt.y() + fst_pt[0] * scd_pt[1] -
                 scd_pt[0] * fst_pt[1];
    if (tmp > 0) {
      return true;
    } else {
      return false;
    }
  };

  if (k >= 0) {
    x_r = waypoints[0][0];
    x_l = waypoints[0][0];
    // //#pragma omp parallel for
    for (double startpt = 0; startpt <= fabs(waypoints[0][0] - waypoints[1][0]);
         startpt = startpt + fabs(inc)) {
      y_l = k * x_l + b;
      point3d_l = lanelet::Point3d(lanelet::utils::getId(), x_l + shift_vec[0],
                                   y_l + shift_vec[1], lanelet_height);
      x_l = x_l + inc;
      createPoint(point3d_l);
      map_->add(point3d_l);
      if (leftofvec(point3d_l, waypoints[0], waypoints[1]) == true) {
        left.push_back(point3d_l);
      } else {
        right.push_back(point3d_l);
      }

      map_->add(left);

      y_r = k * x_r + b;
      point3d_r = lanelet::Point3d(lanelet::utils::getId(), x_r - shift_vec[0],
                                   y_r - shift_vec[1], lanelet_height);
      x_r = x_r + inc;
      createPoint(point3d_r);
      map_->add(point3d_r);
      if (leftofvec(point3d_r, waypoints[0], waypoints[1]) == true) {
        left.push_back(point3d_r);
      } else {
        right.push_back(point3d_r);
      }
      map_->add(right);
    }
  } else {
    x_r = waypoints[0][0];
    x_l = waypoints[0][0];
    for (double startpt = 0; startpt <= fabs(waypoints[0][0] - waypoints[1][0]);
         startpt = startpt + fabs(inc)) {
      y_l = k * x_l + b;
      lanelet::Point3d point3d_l =
          lanelet::Point3d(lanelet::utils::getId(), x_l + shift_vec[0],
                           y_l + shift_vec[1], lanelet_height);
      x_l = x_l + inc;
      createPoint(point3d_l);
      map_->add(point3d_l);
      if (leftofvec(point3d_l, waypoints[0], waypoints[1]) == true) {
        left.push_back(point3d_l);
      } else {
        right.push_back(point3d_l);
      }
      map_->add(point3d_l);
      map_->add(left);

      y_r = k * x_r + b;
      lanelet::Point3d point3d_r =
          lanelet::Point3d(lanelet::utils::getId(), x_r - shift_vec[0],
                           y_r - shift_vec[1], lanelet_height);
      x_r = x_r + inc;
      createPoint(point3d_r);
      map_->add(point3d_r);
      map_->add(point3d_r);
      // std::cout << "right point added" << point3d_r.id() << std::endl;
      if (leftofvec(point3d_r, waypoints[0], waypoints[1]) == true) {
        left.push_back(point3d_r);
      } else {
        right.push_back(point3d_r);
      }
      map_->add(right);
    }
  }
  lanelet::Lanelet Lanelet;
  Lanelet = lanelet::Lanelet(lanelet::utils::getId(), left, right);
  map_->add(Lanelet);
  std::cout << "....in lanelet_data.cpp create lanelet_id =" << Lanelet.id()
            << std::endl;
  return Lanelet;
}

void Lanelet2Data::extendLanelet(lanelet::Lanelet &Lanelet,
                                 const lanelet::Point3d point3d) {
  // for waypoint const Eigen::Vector3d &waypoint
  Eigen::Vector3d waypoint = {point3d.x(), point3d.y(), point3d.z()};
  double k;
  double b;
  double inc;
  double offset = 1.5;
  double x_l;
  double y_l;
  double x_r;
  double y_r;
  lanelet::Point3d point3d_r;
  lanelet::Point3d point3d_l;
  Eigen::Vector2d shift_vec;
  Eigen::Vector2d first_pt;
  lanelet::LineString3d left = Lanelet.leftBound();
  lanelet::LineString3d right = Lanelet.rightBound();
  double lanelet_height = waypoint[2];
  std::cout << "left id" << left.id() << std::endl;
  auto leftofvec = [](lanelet::Point3d pt, Eigen::Vector2d fst_pt,
                      Eigen::Vector3d scd_pt) {
    double tmp = (fst_pt[1] - scd_pt[1]) * pt.x() +
                 (scd_pt[0] - fst_pt[0]) * pt.y() + fst_pt[0] * scd_pt[1] -
                 scd_pt[0] * fst_pt[1];
    if (tmp > 0) {
      return true;
    } else {
      return false;
    }
  };
  first_pt[0] = (left.back().x() - right.back().x()) / 2 + right.back().x();
  first_pt[1] = (left.back().y() - right.back().y()) / 2 + right.back().y();

  if ((waypoint[0] - first_pt[0]) == 0) {
    waypoint[0] = waypoint[0] + 0.01;
    first_pt[0] = first_pt[0] - 0.01;
  }
  k = (waypoint[1] - first_pt[1]) / (waypoint[0] - first_pt[0]);
  b = waypoint[1] - k * waypoint[0];
  shift_vec = {-k * offset / sqrt(pow(k, 2) + 1),
               1 * offset / sqrt(pow(k, 2) + 1)};
  inc = (waypoint[0] - first_pt[0]) / 1;
  std::cout << "k: " << k << std::endl;
  std::cout << "shift vec" << shift_vec[0] << "***" << shift_vec[1] << "***"
            << std::endl;
  std::cout << "inc: " << inc << std::endl;
  std::cout << "abs(inc): " << abs(inc) << std::endl;
  std::cout << "abs(first_pt[0] - waypoint[0]): "
            << abs(first_pt[0] - waypoint[0]) << std::endl;
  std::cout << "first point" << first_pt[0] << " " << first_pt[1] << std::endl;
  std::cout << "second point" << waypoint[0] << " " << waypoint[1] << std::endl;
  if (k >= 0) {
    x_l = first_pt[0];
    x_r = first_pt[0];
    for (double startpt = 0; startpt <= fabs(first_pt[0] - waypoint[0]);
         startpt = startpt + fabs(inc)) {
      y_l = k * x_l + b;
      if (startpt == 0) {
        point3d_l = lanelet::Point3d(lanelet::utils::getId(), left.back().x(),
                                     left.back().y(), lanelet_height);
      } else {
        point3d_l =
            lanelet::Point3d(lanelet::utils::getId(), (x_l + shift_vec[0]),
                             (y_l + shift_vec[1]), lanelet_height);
        createPoint(point3d_l);
        map_->add(point3d_l);
        if (leftofvec(point3d_l, first_pt, waypoint) == true) {
          left.push_back(point3d_l);
        } else {
          right.push_back(point3d_l);
        }
      }
      x_l = x_l + inc;

      // std::cout << "left point added" << std::endl;

      y_r = k * x_r + b;
      if (startpt == 0) {
        point3d_r = lanelet::Point3d(lanelet::utils::getId(), right.back().x(),
                                     right.back().y(), lanelet_height);
      } else {
        point3d_r =
            lanelet::Point3d(lanelet::utils::getId(), (x_r - shift_vec[0]),
                             (y_r - shift_vec[1]), lanelet_height);
        createPoint(point3d_r);
        map_->add(point3d_r);
        if (leftofvec(point3d_r, first_pt, waypoint) == true) {
          left.push_back(point3d_r);
        } else {
          right.push_back(point3d_r);
        }
      }

      x_r = x_r + inc;

      // std::cout << "right point added" << std::endl;
    }
  } else {
    x_l = first_pt[0];
    x_r = first_pt[0];
    for (double startpt = 0; startpt <= fabs(first_pt[0] - waypoint[0]);
         startpt = startpt + fabs(inc)) {
      y_l = k * x_l + b;

      if (startpt == 0) {
        point3d_l = lanelet::Point3d(lanelet::utils::getId(), left.back().x(),
                                     left.back().y(), lanelet_height);
      } else {
        point3d_l =
            lanelet::Point3d(lanelet::utils::getId(), (x_l + shift_vec[0]),
                             (y_l + shift_vec[1]), lanelet_height);
        createPoint(point3d_l);
        map_->add(point3d_l);
        if (leftofvec(point3d_l, first_pt, waypoint) == true) {
          left.push_back(point3d_l);
        } else {
          right.push_back(point3d_l);
        }
      }

      x_l = x_l + inc;

      // std::cout << "left point added" << std::endl;

      y_r = k * x_r + b;
      if (startpt == 0) {
        point3d_r = lanelet::Point3d(lanelet::utils::getId(), right.back().x(),
                                     right.back().y(), lanelet_height);
      } else {
        point3d_r =
            lanelet::Point3d(lanelet::utils::getId(), (x_r - shift_vec[0]),
                             (y_r - shift_vec[1]), lanelet_height);
        createPoint(point3d_r);
        map_->add(point3d_r);
        if (leftofvec(point3d_r, first_pt, waypoint) == true) {
          left.push_back(point3d_r);
        } else {
          right.push_back(point3d_r);
        }
      }

      x_r = x_r + inc;

      // std::cout << "right point added" << std::endl;
    }
  }
  // map_->add(left);
  // map_->add(right);
  // Lanelet.setLeftBound(left);
  //   Lanelet.setRightBound(right);
  Lanelet = lanelet::Lanelet(Lanelet.id(), left, right);
}
std::vector<int> Lanelet2Data::findNextLanelet(lanelet::Lanelet &Lanelet) {
  lanelet::Point3d Pt3d_end_l = Lanelet.leftBound().back();
  lanelet::Point3d Pt3d_end_r = Lanelet.rightBound().back();
  // bool ret =false;
  std::vector<int> next_ids;
  for (auto &value : map_->laneletLayer) {
    lanelet::Point3d Pt3d_begin_l = value.leftBound().front();
    lanelet::Point3d Pt3d_begin_r = value.rightBound().front();
    if (Pt3d_end_l.x() == Pt3d_begin_l.x() &&
        Pt3d_end_l.y() == Pt3d_begin_l.y() &&
        Pt3d_end_l.z() == Pt3d_begin_l.z() &&
        Pt3d_end_r.x() == Pt3d_begin_r.x() &&
        Pt3d_end_r.y() == Pt3d_begin_r.y() &&
        Pt3d_end_r.z() == Pt3d_begin_r.z()) {
      // ret =true;
      // break;
      next_ids.push_back(value.id());
    }
  }
  return next_ids;
}

void Lanelet2Data::jointLanelet(lanelet::Lanelet &Lanelet1,
                                lanelet::Lanelet &lanelet2) {
  lanelet::Point3d Pt3d_end_l = lanelet2.leftBound().front();
  lanelet::Point3d Pt3d_end_r = lanelet2.rightBound().front();
  Lanelet1.leftBound().push_back(Pt3d_end_l);
  Lanelet1.rightBound().push_back(Pt3d_end_r);
}

lanelet::Lanelet Lanelet2Data::searchNearstLanelet(
    lanelet::BasicPoint2d &searchPoint) {
  auto searchFunc = [&searchPoint](const lanelet::BoundingBox2d &lltBox,
                                   const lanelet::Lanelet & /*llt*/) {
    return lanelet::geometry::distance(searchPoint, lltBox) < 3;
  };
  lanelet::Optional<lanelet::Lanelet> lanelet =
      map_->laneletLayer.nearestUntil(searchPoint, searchFunc);
  if (lanelet != boost::none) {
    std::cout << "in search lanelet function lanelet.id() =" << lanelet->id()
              << std::endl;
    return *lanelet;
  } else {
    std::cout << "search no lanelet!!" << std::endl;
    return lanelet::Lanelet();
  }
}

lanelet::Point3d Lanelet2Data::searchNearstPoint(
    lanelet::BasicPoint2d &searchPoint) {
  double len = std::numeric_limits<double>::max();
  lanelet::Point3d Point3d;
  int id;
  for (auto const &value : map_->pointLayer) {
    double l = pow((searchPoint.x() - value.x()), 2) +
               pow((searchPoint.y() - value.y()), 2);
    if (l < len) {
      len = l;
      Point3d = value;
    }
  }
  return Point3d;
}

lanelet::LineString3d Lanelet2Data::searchNearstLinestring(
    lanelet::BasicPoint2d &searchPoint) {
  auto distance = 10000000000;
  int id;
  lanelet::LineString3d ls;
  std::cout << "in search linstring function" << std::endl;
  try {
    for (auto const &value : map_->lineStringLayer)
    // for(int i = 0; i < laneletMap->pointLayer.size(); i++ )
    {
      if (value.size() == 0) {
        continue;
      }
      lanelet::ConstHybridLineString3d lsHybrid =
          lanelet::utils::toHybrid(value);
      if (boost::geometry::distance(lanelet::utils::to2D(searchPoint),
                                    lanelet::utils::to2D(lsHybrid)) <
          distance) {
        distance = boost::geometry::distance(lanelet::utils::to2D(searchPoint),
                                             lanelet::utils::to2D(lsHybrid));
        id = value.id();
        ls = value;
        std::cout << "distance: "
                  << boost::geometry::distance(
                         lanelet::utils::to2D(searchPoint),
                         lanelet::utils::to2D(lsHybrid))
                  << std::endl;
      }
    }
  } catch (...) {  //三个点表示可以捕获任意类型的异常
    std::cerr << "there is an error" << std::endl;
  }

  return ls;
}

lanelet::Polygon3d Lanelet2Data::searchNearstPolygon(
    lanelet::BasicPoint2d &searchPoint) {
  auto distance = 10000000000;
  int id;
  lanelet::Polygon3d poly;
  std::cout << "in search poly function" << std::endl;
  try {
    for (auto const &value : map_->polygonLayer)
    // for(int i = 0; i < laneletMap->pointLayer.size(); i++ )
    {
      if (value.size() == 0) {
        continue;
      }
      lanelet::ConstHybridPolygon3d polyHybrid =
          lanelet::utils::toHybrid(value);
      if (boost::geometry::distance(lanelet::utils::to2D(searchPoint),
                                    lanelet::utils::to2D(polyHybrid)) <
          distance) {
        distance = boost::geometry::distance(lanelet::utils::to2D(searchPoint),
                                             lanelet::utils::to2D(polyHybrid));
        id = value.id();
        poly = value;
        std::cout << "distance: "
                  << boost::geometry::distance(
                         lanelet::utils::to2D(searchPoint),
                         lanelet::utils::to2D(polyHybrid))
                  << std::endl;
      }
    }
  } catch (...) {  //三个点表示可以捕获任意类型的异常
    std::cerr << "there is an error" << std::endl;
  }

  return poly;
}

void Lanelet2Data::movePrimitive(lanelet::LineString3d &ls_3d,
                                 Eigen::Vector3d offset) {
  //#pragma omp parallel for
  for (int i = 0; i < ls_3d.size(); i++) {
    ls_3d[i].x() = ls_3d[i].x() + offset[0];
    ls_3d[i].y() = ls_3d[i].y() + offset[1];
    ls_3d[i].z() = ls_3d[i].z() + offset[2];
  }
}

void Lanelet2Data::movePrimitive(lanelet::Point3d &point3d,
                                 Eigen::Vector3d offset) {
  point3d.x() = point3d.x() + offset[0];
  point3d.y() = point3d.y() + offset[1];
  point3d.z() = point3d.z() + offset[2];
}

void Lanelet2Data::rmPrimitive(lanelet::Lanelet &Lanelet) {
  Lanelet.leftBound().clear();
  Lanelet.rightBound().clear();
}

void Lanelet2Data::rmPrimitive(lanelet::LineString3d &Linestring) {
  Linestring.clear();
}

void Lanelet2Data::rmPrimitive(lanelet::Polygon3d &poly) {
  poly.clear();
  std::cout << "in rm poly_id = " << poly.id() << std::endl;
}

void Lanelet2Data::rmPrimitive(lanelet::Point3d &pt) {
  for (auto &value : map_->lineStringLayer) {
    auto it = value.begin();
    for (auto const &pt_value : value) {
      if (pt_value.id() == pt.id()) {
        value.erase(it);
        return;
      }
      it++;
    }
  }
  for (auto &value : map_->polygonLayer) {
    auto it = value.begin();
    for (auto const &pt_value : value) {
      if (pt_value.id() == pt.id()) {
        value.erase(it);
        return;
      }
      it++;
    }
  }
}

std::vector<int> Lanelet2Data::showPtsId(
    const lanelet::LineString3d &Linestring) {
  std::vector<int> pts_id;
  for (const auto &value : Linestring) {
    pts_id.push_back(value.id());
  }
  return pts_id;
}
std::vector<lanelet::Point3d> Lanelet2Data::getPoints(
    lanelet::LineString3d Linestring) {
  std::vector<lanelet::Point3d> points;
  for (auto &value : Linestring) {
    points.push_back(value);
  }
  return points;
}
std::vector<lanelet::Point3d> Lanelet2Data::getPoints(
    lanelet::Polygon3d polygon) {
  std::vector<lanelet::Point3d> points;
  for (auto &value : polygon) {
    points.push_back(value);
  }
  return points;
}

void Lanelet2Data::create_show_map(double Zmin, double Zmax) {
  for (auto const &value : map_->lineStringLayer) {
    bool in_layer = true;
    if (!value.empty()) {
      for (auto const &pt_value : value) {
        if (Zmin <= pt_value.z() && pt_value.z() >= Zmax) {
          in_layer = false;
          break;
        }
      }
      if (in_layer) {
        for (auto const &pt_value : value) {
          pt_value.id();
          for (auto const &value : map_->pointLayer) {
            if (value.id() == pt_value.id()) {
              maplayer_show_->add(value);
            }
          }
        }
        maplayer_show_->add(value);
      }
    }
  }
  for (auto const &value : map_->laneletLayer) {
    if (value.leftBound().size() == 0 || value.rightBound().size() == 0) {
      continue;
    }
    bool in_layer = true;
    for (auto const &pt_value : value.leftBound()) {
      if (Zmin <= pt_value.z() && pt_value.z() >= Zmax) {
        in_layer = false;
        break;
      }
    }
    if (!in_layer) {
      continue;
    }
    maplayer_show_->add(value);
  }
  for (auto const &value : map_->polygonLayer) {
    if (value.size() == 0) {
      continue;
    }
    bool in_layer = true;
    for (auto const &pt_value : value) {
      if (Zmin <= pt_value.z() && pt_value.z() >= Zmax) {
        in_layer = false;
        break;
      }
    }
    if (!in_layer) {
      continue;
    }
    maplayer_show_->add(value);
  }
}

void Lanelet2Data::addCustomizeTags(std::vector<tag_list> &tag_lists) {
  for (auto &value_tag : tag_lists) {
    for (auto &value : map_->lineStringLayer) {
      if (value.id() == value_tag.id) {
        addLineAtrribute(value, value_tag.tags);
        continue;
      }
    }
    for (auto &value : map_->laneletLayer) {
      if (value.id() == value_tag.id) {
        addLaneletAtrribute(value, value_tag.tags);
        continue;
      }
    }
  }
}

void Lanelet2Data::map_out() {
  this->map_o_.reset(new lanelet::LaneletMap());
  for (auto &value : map_->lineStringLayer) {
    if (!value.empty()) {
      map_o_->add(value);
      for (auto const &pt_value : value) {
        for (auto &value : map_->pointLayer) {
          if (value.id() == pt_value.id()) {
            // lon/lat for no lon/lat input-map
            createPoint(value);
            // geometry_msgs::Pose pose;
            // pose.position.x = value.x();
            // pose.position.y = value.y();
            // pose.position.z = value.z();
            // LatLonInfo latlon_info;
            // if (pose2gps(pose, latlon_info, init_origin_pos)) {
            //   value.attributes()["gps_lon"] = latlon_info.longitude;
            //   value.attributes()["gps_lat"] = latlon_info.latitude;
            // }
            map_o_->add(value);
          }
        }
      }
    }
  }
  for (auto const &value : map_->laneletLayer) {
    if (value.leftBound().size() == 0 || value.rightBound().size() == 0) {
      continue;
    }
    map_o_->add(value);
  }
  for (auto const &value : map_->polygonLayer) {
    if (value.size() < 3) {
      continue;
    }
    map_o_->add(value);
    for (auto const &pt_value : value) {
      for (auto &value : map_->pointLayer) {
        if (value.id() == pt_value.id()) {
          // lon/lat for no lon/lat input-map
          createPoint(value);
          // geometry_msgs::Pose pose;
          // pose.position.x = value.x();
          // pose.position.y = value.y();
          // pose.position.z = value.z();
          // LatLonInfo latlon_info;
          // if (pose2gps(pose, latlon_info, init_origin_pos)) {
          //   value.attributes()["gps_lon"] = latlon_info.longitude;
          //   value.attributes()["gps_lat"] = latlon_info.latitude;
          // }
          map_o_->add(value);
        }
      }
    }
  }
  for (auto &value : map_->regulatoryElementLayer) {
    // value.reset();
    map_o_->add(value);
  }
}

std::vector<tag_list> Lanelet2Data::tagsFromRlanelet2map() {
  std::vector<tag_list> taglist_vec;
  for (auto &value : map_->lineStringLayer) {
    if (!value.empty()) {
      tag_list taglist;
      for (auto iter : value.attributes()) {
        if (iter.first == lanelet::AttributeNamesString::Type ||
            iter.first == lanelet::AttributeNamesString::Subtype)
          continue;
        tag tagl;
        tagl.k = iter.first;
        std::cout << "tagl.k =" << tagl.k << std::endl;
        tagl.v = iter.second.value();
        taglist.tags.push_back(tagl);
      }
      if (taglist.tags.size() == 0) continue;
      taglist.id = value.id();
      taglist_vec.push_back(taglist);
    }
  }

  for (auto &value : map_->laneletLayer) {
    if (!value.rightBound().empty()) {
      tag_list taglist;
      for (auto iter : value.attributes()) {
        if (iter.first == lanelet::AttributeNamesString::Type ||
            iter.first == lanelet::AttributeNamesString::Subtype)
          continue;
        tag tagl;
        tagl.k = iter.first;
        std::cout << "tagl.k =" << tagl.k << std::endl;
        tagl.v = iter.second.value();
        taglist.tags.push_back(tagl);
      }
      if (taglist.tags.size() == 0) continue;
      taglist.id = value.id();
      taglist_vec.push_back(taglist);
    }
  }

  for (auto &value : map_->polygonLayer) {
    if (!value.empty()) {
      tag_list taglist;
      for (auto iter : value.attributes()) {
        if (iter.first == lanelet::AttributeNamesString::Type ||
            iter.first == lanelet::AttributeNamesString::Area)
          continue;
        tag tagl;
        tagl.k = iter.first;
        std::cout << "tagl.k =" << tagl.k << std::endl;
        tagl.v = iter.second.value();
        taglist.tags.push_back(tagl);
      }
      if (taglist.tags.size() == 0) continue;
      taglist.id = value.id();
      taglist_vec.push_back(taglist);
    }
  }
  // std::cout << "in laneler data taglist_vec=" << taglist_vec.size()
  //           << std::endl;
  return taglist_vec;
}

void Lanelet2Data::addLineAtrribute(lanelet::LineString3d &Linestring,
                                    std::vector<tag> &tag_list) {
  for (const auto &value : tag_list) {
    Linestring.attributes()[value.k] = value.v;
  }
}

void Lanelet2Data::addLaneletAtrribute(lanelet::Lanelet &Lanelet,
                                       std::vector<tag> &tag_list) {
  for (const auto &value : tag_list) {
    Lanelet.attributes()[value.k] = value.v;
  }
}

void Lanelet2Data::addLaneletPedestrian(lanelet::Lanelet &ll) {
  ll.setAttribute(lanelet::AttributeName::Subtype,
                  lanelet::AttributeValueString::Crosswalk);
}

int Lanelet2Data::addRegElement(lanelet::Lanelet &ll,
                                lanelet::Polygon3d &junction) {
  using namespace lanelet;
  RuleParameterMap rpm{{RoleNameString::Refers, {junction}}};
  AttributeMap attributes;
  attributes[AttributeNamesString::Type] =
      lanelet::AttributeValueString::RegulatoryElement;
  attributes[AttributeNamesString::Subtype] = "intersection";
  GenericRegulatoryElementPtr regelem(
      new GenericRegulatoryElement(utils::getId(), rpm, attributes));
  // ll.addRegulatoryElement(regelem);
  std::cout << "in add stopline ....id = " << regelem->id() << std::endl;
  return regelem->id();
}

int Lanelet2Data::addRegElement(lanelet::Lanelet &ll,
                                lanelet::Polygon3d &junction,
                                GenericRegulatoryElementPtr &regelem,
                                std::string subtype) {
  using namespace lanelet;
  RuleParameterMap rpm{{RoleNameString::Refers, {junction}}};
  // AttributeMap attributes;
  // attributes[AttributeNamesString::Type] = "regulatory_element";
  // attributes[AttributeNamesString::Subtype] = "intersection";
  regelem->attributes()[AttributeNamesString::Type] = "regulatory_element";
  regelem->attributes()[AttributeNamesString::Subtype] = "regulatory_element";
  ll.addRegulatoryElement(regelem);
  regelem->addParameter(subtype, junction);
  std::cout << "in add stopline ....id = " << regelem->id() << std::endl;
  this->map_->add(regelem);
  return regelem->id();
}

int Lanelet2Data::addRegElement(lanelet::Lanelet &ll,
                                lanelet::LineString3d &refline,
                                GenericRegulatoryElementPtr &regelem,
                                std::string subtype) {
  std::cout << "in add reflineine ...." << std::endl;
  using namespace lanelet;

  RuleParameterMap rpm{{RoleNameString::RefLine, {refline}}};
  // AttributeMap attributes;
  // attributes[AttributeNamesString::Type] = "regulatory_element";
  // attributes[AttributeNamesString::Subtype] = 'stop_line';
  // regelem->attributes()[AttributeNamesString::Subtype] = "stop_line1";;
  regelem->attributes()[AttributeNamesString::Type] = "regulatory_element";
  regelem->attributes()[AttributeNamesString::Subtype] = "regulatory_element";
  ll.addRegulatoryElement(regelem);
  regelem->addParameter(subtype, refline);
  std::cout << "in add stopline ....id = " << regelem->id() << std::endl;
  this->map_->add(regelem);
  return regelem->id();
}

// int Lanelet2Data::addRegElement(lanelet::Lanelet &ll,
//                                  lanelet::LineString3d &refline) {
//   std::cout << "in add stopline ...." << std::endl;
//   using namespace lanelet;

//   RuleParameterMap rpm{{RoleNameString::RefLine, {refline}}};
//   AttributeMap attributes;
//   attributes[AttributeNamesString::Type] = "regulatory_element";
//   attributes[AttributeNamesString::Subtype] = "stop_line";
//   GenericRegulatoryElementPtr regelem(
//       new GenericRegulatoryElement(utils::getId(), rpm, attributes));
//   // regelem->attributes()[AttributeNamesString::Subtype] = "stop_line1";;
//   // ll.addRegulatoryElement(regelem);
//   std::cout << "in add stopline ....id = " << regelem->id() << std::endl;
//   return regelem->id();
// }
int Lanelet2Data::addRegElement(lanelet::Lanelet &ll,
                                lanelet::LineString3d &refline) {
  std::cout << "in add stopline ...." << std::endl;
  using namespace lanelet;

  RuleParameterMap rpm{{RoleNameString::RefLine, {refline}}};
  AttributeMap attributes;
  attributes[AttributeNamesString::Type] =
      lanelet::AttributeValueString::RegulatoryElement;
  attributes[AttributeNamesString::Subtype] =
      refline.attribute(lanelet::AttributeNamesString::Type).value();
  GenericRegulatoryElementPtr regelem(
      new GenericRegulatoryElement(utils::getId(), rpm, attributes));
  // regelem->attributes()[AttributeNamesString::Subtype] = "stop_line1";;
  // ll.addRegulatoryElement(regelem);
  std::cout << "in add stopline ....id = " << regelem->id() << std::endl;
  return regelem->id();
}

void Lanelet2Data::addReglist(std::vector<reguinfo_list> &reg_vec) {
  using namespace lanelet;
  // RegulatoryElement regelem;
  for (auto &value : reg_vec) {
    GenericRegulatoryElementPtr regelem(
        new GenericRegulatoryElement(value.reglists.Regelement_id));
    bool regelem_exist = false;
    for (auto &regelem_value : map_->regulatoryElementLayer) {
      if (regelem_value->id() == value.reglists.Regelement_id) {
        regelem_exist = true;
        break;
      }
    }
    if (regelem_exist == false) {
      this->map_->add(regelem);
    }
    std::cout << "subtybe of the value " << value.Subtype << std::endl;
    if (value.Subtype == "stop_line" || value.Subtype == "speed_limit" ||
        value.Subtype == "traffic_light") {
      std::cout << "in ..data.cpp addReglist ---value.Subtype= "
                << value.Subtype << std::endl;
      lanelet::LineString3d refelement;
      lanelet::Lanelet reflanelet;
      bool element_found = false;
      for (auto &lanelet_value : map_->laneletLayer) {
        if (lanelet_value.id() == value.laneletid) {
          reflanelet = lanelet_value;
          element_found = true;
          std::cout << "lanelet value found" << std::endl;
          break;
        }
      }
      for (auto &linestring_value : map_->lineStringLayer) {
        if (linestring_value.id() == value.regmemberid) {
          refelement = linestring_value;
          element_found = true;
          std::cout << "linestring value found" << std::endl;
          break;
        }
      }
      if (element_found == true) {
        addRegElement(reflanelet, refelement, regelem, value.Subtype);
      }
    } else if (value.Subtype == "intersection") {
      lanelet::Polygon3d refelement;
      lanelet::Lanelet reflanelet;
      bool element_found = false;
      for (auto &lanelet_value : map_->laneletLayer) {
        if (lanelet_value.id() == value.laneletid) {
          reflanelet = lanelet_value;
          element_found = true;
          std::cout << "lanelet value found" << std::endl;
          break;
        }
      }
      for (auto &polygon_value : map_->polygonLayer) {
        if (polygon_value.id() == value.regmemberid) {
          refelement = polygon_value;
          element_found = true;
          std::cout << "polygon value found" << std::endl;
          break;
        }
      }
      if (element_found == true) {
        addRegElement(reflanelet, refelement, regelem, value.Subtype);
      }
    }
    regelem->attributes()["maneuver"] = value.reglists.maneuver;
    regelem->attributes()["element"] = value.reglists.element;
    for (auto &refers_value : value.reglists.refers_list) {
      for (auto &lanelet_value : map_->laneletLayer) {
        if (lanelet_value.id() == refers_value.refer_id) {
          regelem->addParameter(refers_value.referName, lanelet_value);
          this->map_->add(regelem);
          break;
        }
      }
      for (auto &polygon_value : map_->polygonLayer) {
        if (polygon_value.id() == refers_value.refer_id) {
          regelem->addParameter(refers_value.referName, polygon_value);
          this->map_->add(regelem);
          break;
        }
      }
      for (auto &linestring_value : map_->lineStringLayer) {
        if (linestring_value.id() == refers_value.refer_id) {
          regelem->addParameter(refers_value.referName, linestring_value);
          this->map_->add(regelem);
          break;
        }
      }
    }
  }
  // std::cout << "regulatoryElement size ...... save"
  //           << this->map_->regulatoryElementLayer.size() << std::endl;
  // std::cout << "..............................................................."
  //              "........"
  //           << std::endl;
  // for (auto &lanelet_value : map_->laneletLayer) {
  //   std::cout << "add reglist lanelet _value regular.size ="
  //             << lanelet_value.regulatoryElements().size() << std::endl;
  // }
  // std::cout << "add reglist reg_vec.size =" << reg_vec.size() << std::endl;
}

std::vector<reguinfo_list> Lanelet2Data::regelementload() {
  std::vector<reguinfo_list> regul_list;
  // std::cout << "regulatoryElement size ...... load"
            // << this->map_->regulatoryElementLayer.size() << std::endl;
  for (auto &lanelet_value : map_->laneletLayer) {
    if (lanelet_value.regulatoryElements().size() == 0) {
      continue;
    }
    //#pragma omp parallel for
    for (int i = 0; i < lanelet_value.regulatoryElements().size(); i = i + 1) {
      reguinfo_list regelement;
      regelement.laneletid = lanelet_value.id();
      lanelet::RegulatoryElementPtr regelem =
          lanelet_value.regulatoryElements()[i];
      // std::cout << " before that" << std::endl;
      // regelem->attribute("type");
      // std::cout << "after that" << std::endl;
      if (regelem->getParameters<lanelet::ConstLineString3d>("speed_limit")
              .size() != 0) {
        lanelet::ConstLineString3d RefLine;
        RefLine = regelem->getParameters<lanelet::ConstLineString3d>(
            "speed_limit")[0];
        regelement.regmemberid = RefLine.id();
        regelement.Subtype = "speed_limit";
        // regelem->attributes();
      }
      if (regelem->getParameters<lanelet::ConstLineString3d>("stop_line")
              .size() != 0) {
        lanelet::ConstLineString3d RefLine;
        RefLine =
            regelem->getParameters<lanelet::ConstLineString3d>("stop_line")[0];
        regelement.regmemberid = RefLine.id();
        regelement.Subtype = "stop_line";
      }
      if (regelem->getParameters<lanelet::ConstLineString3d>("traffic_light")
              .size() != 0) {
        lanelet::ConstLineString3d RefLine;
        RefLine = regelem->getParameters<lanelet::ConstLineString3d>(
            "traffic_light")[0];
        regelement.regmemberid = RefLine.id();
        regelement.Subtype = "traffic_light";
      }
      if (regelem->getParameters<lanelet::ConstPolygon3d>("intersection")
              .size() != 0) {
        lanelet::ConstPolygon3d RefPolygon;
        RefPolygon =
            regelem->getParameters<lanelet::ConstPolygon3d>("intersection")[0];
        regelement.regmemberid = RefPolygon.id();
        std::cout << "poly id" << RefPolygon.id();
        regelement.Subtype = "intersection";
        // regelem->attributes();
      }
      regelement_list regelement_list;
      refer_list refer_list;
      //#pragma omp parallel for
      for (int i = 0;
           i <
           regelem->getParameters<lanelet::ConstLanelet>("ref_lanelets").size();
           i++) {
        refer_list.refer_id =
            regelem->getParameters<lanelet::ConstLanelet>("ref_lanelets")[i]
                .id();
        refer_list.referName = "ref_lanelets";
        regelement_list.refers_list.push_back(refer_list);
      }
      //#pragma omp parallel for
      for (int i = 0;
           i < regelem->getParameters<lanelet::ConstLineString3d>("stop_line")
                   .size();
           i++) {
        refer_list.refer_id =
            regelem->getParameters<lanelet::ConstLineString3d>("stop_line")[i]
                .id();
        refer_list.referName = "stop_line";
        regelement_list.refers_list.push_back(refer_list);
      }
      //#pragma omp parallel for
      for (int i = 0;
           i <
           regelem->getParameters<lanelet::ConstLineString3d>("intersection")
               .size();
           i++) {
        refer_list.refer_id =
            regelem
                ->getParameters<lanelet::ConstLineString3d>("intersection")[i]
                .id();
        refer_list.referName = "intersection";
        regelement_list.refers_list.push_back(refer_list);
      }
      //#pragma omp parallel for
      for (int i = 0;
           i <
           regelem->getParameters<lanelet::ConstLineString3d>("ref_linestrings")
               .size();
           i++) {
        refer_list.refer_id = regelem
                                  ->getParameters<lanelet::ConstLineString3d>(
                                      "ref_linestrings")[i]
                                  .id();
        refer_list.referName = "ref_linestrings";
        regelement_list.refers_list.push_back(refer_list);
      }
      //#pragma omp parallel for
      for (int i = 0;
           i < regelem->getParameters<lanelet::ConstPolygon3d>("ref_polygons")
                   .size();
           i++) {
        refer_list.refer_id =
            regelem->getParameters<lanelet::ConstPolygon3d>("ref_polygons")[i]
                .id();
        refer_list.referName = "ref_polygons";
        regelement_list.refers_list.push_back(refer_list);
      }
      if (regelem->hasAttribute("maneuver")) {
        regelement_list.maneuver = regelem->attribute("maneuver").value();
      }
      if (regelem->hasAttribute("element")) {
        regelement_list.element = regelem->attribute("element").value();
      }
      regelement_list.Regelement_id = regelem->id();
      regelement.reglists = regelement_list;
      regul_list.push_back(regelement);
    }
  }
  for (auto &value : regul_list) {
    std::cout << "observe pt 1 "
              << "value of the subtype " << value.Subtype << std::endl;
  }
  return regul_list;
}

template <typename T>
bool findAndErase(const T &primitive, lanelet::RuleParameterMap &parameters,
                  lanelet::RoleName role) {
  auto parameterIt = parameters.find(role);
  if (parameterIt == parameters.end()) {
    return false;
  }
  auto &parameter = parameterIt->second;
  auto it = std::find(parameter.begin(), parameter.end(),
                      lanelet::RuleParameter(primitive));
  if (it == parameter.end()) {
    return false;
  }
  parameter.erase(it);
  if (parameter.empty()) {
    parameters.erase(parameterIt);
  }
  return true;
}

void Lanelet2Data::clearRegelem() {
  for (auto &lanelet_value : map_->laneletLayer) {
    auto &regElems = lanelet_value.regulatoryElements();
    for (auto &value : regElems) {
      lanelet_value.removeRegulatoryElement(value);
    }
  }

  // for (auto &value : map_->regulatoryElementLayer) {
  //   // lanelet::RuleParameterMap t;

  //   lanelet::RuleParameterMap &t =
  //       std::const_pointer_cast<lanelet::RegulatoryElementData>(
  //           value->constData())
  //           ->parameters;
  //   lanelet::Id  &id =
  //       std::const_pointer_cast<lanelet::RegulatoryElementData>(
  //           value->constData())
  //           ->id;
  //   // for (auto &value_line : map_->lineStringLayer) {
  //   //   findAndErase(value_line, t, lanelet::RoleName::Refers);
  //   // }
  //   for (auto &value_regelm : t) {
  //     auto &parameter = value_regelm.second;
  //     parameter.clear();
  //   }
  // }
  map_->remove();
//   std::cout << "regulatoryElement size ...... after clear"
//             << this->map_->regulatoryElementLayer.size() << std::endl;
}

// for cti_maptool
bool Lanelet2Data::mapEmpty() {
  bool ret = false;
  if (!map_ || map_->pointLayer.size()==0) {
    ret = true;//NO MAP
  }
  return ret;
}
void Lanelet2Data::moveFrontPoint3d(lanelet::Point3d point,
                                    lanelet::Point3d &new_point3d,
                                    lanelet::Polygon3d poly) {
  int size = poly.size();
  int index = 0;
  for (auto pt : poly) {
    index++;
    if (pt.id() == point.id()) break;
  }
  index = index < size ? index + 1 : 1;
  new_point3d = *(poly.begin() + index - 1);
}
void Lanelet2Data::moveFrontPoint3d(lanelet::Point3d point,
                                    lanelet::Point3d &new_point3d,
                                    lanelet::LineString3d linestring) {
  int size = linestring.size();
  int index = 0;
  for (auto pt : linestring) {
    index++;
    if (pt.id() == point.id()) break;
  }
  index = index < size ? index + 1 : 1;
  new_point3d = *(linestring.begin() + index - 1);
}
void Lanelet2Data::moveBackPoint3d(lanelet::Point3d point,
                                   lanelet::Point3d &new_point3d,
                                   lanelet::Polygon3d poly) {
  int size = poly.size();
  int index = 0;
  for (auto pt : poly) {
    index++;
    if (pt.id() == point.id()) break;
  }
  index = index > 1 ? index - 1 : size;
  new_point3d = *(poly.begin() + index - 1);
}
void Lanelet2Data::moveBackPoint3d(lanelet::Point3d point,
                                   lanelet::Point3d &new_point3d,
                                   lanelet::LineString3d linestring) {
  int size = linestring.size();
  int index = 0;
  for (auto pt : linestring) {
    index++;
    if (pt.id() == point.id()) break;
  }
  index = index > 1 ? index - 1 : size;
  new_point3d = *(linestring.begin() + index - 1);
}
void Lanelet2Data::insertFrontPoint3d(
    lanelet::Point3d point, lanelet::LineString3d &linestring,
    lanelet::Point3d &new_point3d) {
  int size = linestring.size();
  int index = 0;
  for (auto pt : linestring) {
    index++;
    if (pt.id() == point.id()) break;
  }
  if (index == size && linestring.back().id() != point.id()) {
    std::cout << "in linestring no find point3d" << std::endl;
    return;
  } else {
    map_->add(new_point3d);
    createPoint(new_point3d);
    linestring.insert(linestring.begin() + index, new_point3d);
  }
}
void Lanelet2Data::insertFrontPoint3d(
    lanelet::Point3d point, lanelet::Polygon3d &poly,
    lanelet::Point3d &new_point3d) {
  int size = poly.size();
  int index = 0;
  for (auto pt : poly) {
    index++;
    if (pt.id() == point.id()) break;
  }
  if (index == size && poly.back().id() != point.id()) {
    std::cout << "in poly no find point3d" << std::endl;
    return;
  } else {
    map_->add(new_point3d);
    createPoint(new_point3d);
    poly.insert(poly.begin() + index, new_point3d);
  }
}
void Lanelet2Data::insertBackPoint3d(
    lanelet::Point3d point, lanelet::LineString3d &linestring,
    lanelet::Point3d &new_point3d) {
  int size = linestring.size();
  int index = 0;
  for (auto pt : linestring) {
    index++;
    if (pt.id() == point.id()) break;
  }
  if (index == size && linestring.back().id() != point.id()) {
    std::cout << "in linestring no find point3d" << std::endl;
    return;
  } else {
    map_->add(new_point3d);
    createPoint(new_point3d);
    linestring.insert(linestring.begin() + index - 1, new_point3d);
  }
}
void Lanelet2Data::insertBackPoint3d(
    lanelet::Point3d point, lanelet::Polygon3d &poly,
    lanelet::Point3d &new_point3d) {
  int size = poly.size();
  int index = 0;
  for (auto pt : poly) {
    std::cout << "do back in poly over......" << std::endl;
    index++;
    if (pt.id() == point.id()) break;
  }
  if (index == size && poly.back().id() != point.id()) {
    std::cout << "in poly no find point3d" << std::endl;
    return;
  } else {
    map_->add(new_point3d);
    createPoint(new_point3d);
    std::cout << "do back in poly befor add......" << std::endl;
    poly.insert(poly.begin() + index - 1, new_point3d);
  }
}
bool Lanelet2Data::findPoint3dWithId(int point3d_id,
                                     lanelet::Point3d &point3d) {
  bool ret = false;
  if (!map_ || point3d_id <= 0) {
    std::cerr << "No map received! or id <=0" << std::endl;
    return ret;
  }
  for (auto const &pt : map_->pointLayer) {
    if (pt.id() == point3d_id) {
      point3d = pt;
      ret = true;
    }
  }
  return ret;
}
bool Lanelet2Data::isPoint3dinLinestring(int point3d_id,
                                         lanelet::LineString3d ll) {
  bool ret = false;
  if (ll.size() == 0) return ret;
  for (auto pt : ll) {
    if (pt.id() == point3d_id) return true;
  }
  return ret;
}
bool Lanelet2Data::isPoint3dinPoly(int point3d_id, lanelet::Polygon3d pl) {
  bool ret = false;
  if (pl.size() == 0) return ret;
  for (auto pt : pl) {
    if (pt.id() == point3d_id) return true;
  }
  return ret;
}
bool Lanelet2Data::isPoint3dinLinestringLayer(int point3d_id) {
  bool ret = false;
  if (!map_ || point3d_id <= 0) {
    std::cerr << "No map received! or id <=0" << std::endl;
    return ret;
  }
  for (auto &ls_value : map_->lineStringLayer) {
    for (auto &pt : ls_value) {
      if (pt.id() == point3d_id) {
        ret = true;
        break;
      }
    }
    if (ret) break;
  }
}
bool Lanelet2Data::findLaneletWithId(int lanelet_id,
                                     lanelet::Lanelet &lanelet) {
  bool ret = false;
  if (!map_ || lanelet_id <= 0) {
    std::cerr << "No map received!" << std::endl;
    return ret;
  }
  for (auto &lanelet_value : map_->laneletLayer) {
    if (lanelet_value.id() == lanelet_id) {
      if (lanelet_value.rightBound().size() <= 0 &&
          lanelet_value.leftBound().size() <= 0)
        return ret;
      lanelet = lanelet_value;
      ret = true;
    }
  }
  return ret;
}
bool Lanelet2Data::findLinestringWithId(int linestring_id,
                                        lanelet::LineString3d &linestring) {
  bool ret = false;
  if (!map_ || linestring_id <= 0) {
    std::cerr << "No map received!" << std::endl;
    return ret;
  }
  for (auto &ls_value : map_->lineStringLayer) {
    if (ls_value.id() == linestring_id) {
      if (ls_value.size() <= 0) return ret;
      linestring = ls_value;
      ret = true;
    }
  }
  return ret;
}
bool Lanelet2Data::findPoly3dWithId(int poly_id, lanelet::Polygon3d &poly) {
  bool ret = false;
  if (!map_ || poly_id <= 0) {
    std::cerr << "No map received!" << std::endl;
    return ret;
  }
  for (auto const &pl_value : map_->polygonLayer) {
    if (pl_value.id() == poly_id) {
      if (pl_value.size() <= 0) return ret;
      poly = pl_value;
      ret = true;
    }
  }
  return ret;
}
void Lanelet2Data::addRoadAtrribute(lanelet::Lanelet &llt) {
  llt.setAttribute(lanelet::AttributeName::Subtype,
                   lanelet::AttributeValueString::Road);
  llt.setAttribute(lanelet::AttributeName::Type,
                   lanelet::AttributeValueString::Lanelet);
  // addlinestringAtrribute(llt.leftBound());
  // addlinestringAtrribute(llt.rightBound());
  lanelet::LineString3d ll = llt.leftBound();
  addlinestringAtrribute(ll);
  ll = llt.rightBound();
  addlinestringAtrribute(ll);
}
void Lanelet2Data::addCrosswalkAtrribute(lanelet::Lanelet &llt) {
  llt.setAttribute(lanelet::AttributeName::Subtype,
                   lanelet::AttributeValueString::Crosswalk);
  llt.setAttribute(lanelet::AttributeName::Type,
                   lanelet::AttributeValueString::Lanelet);
  lanelet::LineString3d ll = llt.leftBound();
  addlinestringAtrribute(ll);
  ll = llt.rightBound();
  addlinestringAtrribute(ll);
}
void Lanelet2Data::addlinestringAtrribute(lanelet::LineString3d &linestring) {
  linestring.setAttribute(lanelet::AttributeName::Subtype,
                          lanelet::AttributeValueString::Solid);
  linestring.setAttribute(lanelet::AttributeName::Type,
                          lanelet::AttributeValueString::LineThin);
}
void Lanelet2Data::addStoplineAtrribute(lanelet::LineString3d &linestring) {
  linestring.setAttribute(lanelet::AttributeName::Subtype,
                          lanelet::AttributeValueString::Solid);
  linestring.setAttribute(lanelet::AttributeName::Type,
                          lanelet::AttributeValueString::StopLine);
}
void Lanelet2Data::addBumpAtrribute(lanelet::LineString3d &linestring) {
  linestring.setAttribute(lanelet::AttributeName::Subtype,
                          lanelet::AttributeValueString::Solid);
  linestring.setAttribute(lanelet::AttributeName::Type,
                          lanelet::AttributeValueString::Bump);
}
void Lanelet2Data::getNewlineFromcurrentPoint(int pt_id) {
  if (!map_) {
    std::cerr << "No map received!" << std::endl;
    return;
  }
  int re_index = 0;
  lanelet::LineString3d re_line;
  bool getreline = false;
  for (auto li = map_->lineStringLayer.begin();
       li != map_->lineStringLayer.end(); li++) {
    lanelet::LineString3d ls = *li;
    int index = 0;
    for (auto &pt : ls) {
      index++;
      if (pt.id() == pt_id) {
        re_index = index;
        re_line = ls;
        getreline = true;
        break;
      }
    }
    if (getreline) break;
  }
  if (re_index > 0) {
    std::vector<lanelet::Point3d> point3d_vec;
    point3d_vec.insert(point3d_vec.begin(), re_line.begin() + re_index,
                       re_line.end());
    if (point3d_vec.size() >= 2) {
      std::vector<lanelet::Point3d> pt_vec;
      pt_vec.insert(pt_vec.begin(), point3d_vec.begin(),
                    point3d_vec.begin() + 2);
      lanelet::LineString3d ls_3d =
          lanelet::LineString3d(lanelet::InvalId, pt_vec);
      this->map_->add(ls_3d);
      addlinestringAtrribute(ls_3d);
      ls_3d.insert(ls_3d.end(), point3d_vec.begin() + 2, point3d_vec.end());
    }
    auto it = re_line.begin() + re_index;
    while (it < re_line.end()) {
      re_line.erase(it);
    }
  }
}
bool Lanelet2Data::getBindPointAtLine(int ls_id, int pt_id,
                                      lanelet::Point3d find_point) {
  lanelet::LineString3d ls;
  lanelet::Point3d point3d;
  bool ret = false;
  if (findLinestringWithId(ls_id, ls) && findPoint3dWithId(pt_id, point3d)) {
    double len1 = pow((ls.front().x() - find_point.x()), 2) +
                  pow((ls.front().y() - find_point.y()), 2);
    double len2 = pow((ls.back().x() - find_point.x()), 2) +
                  pow((ls.back().y() - find_point.y()), 2);
    if (len1 < len2) {
      ls.insert(ls.begin(), point3d);
    } else {
      ls.push_back(point3d);
    }
    ret = true;
  }
  return ret;
}
bool Lanelet2Data::getBindPointAtLine(int ls_id, int pt_id) {
  lanelet::LineString3d ls;
  lanelet::Point3d point3d;
  bool ret = false;
  if (findLinestringWithId(ls_id, ls) && findPoint3dWithId(pt_id, point3d)) {
    double len1 = pow((ls.front().x() - point3d.x()), 2) +
                  pow((ls.front().y() - point3d.y()), 2) +
                  pow((ls.front().z() - point3d.z()), 2);
    double len2 = pow((ls.back().x() - point3d.x()), 2) +
                  pow((ls.back().y() - point3d.y()), 2) +
                  pow((ls.back().z() - point3d.z()), 2);
    if (len1 < len2) {
      ls.insert(ls.begin(), point3d);
    } else {
      ls.push_back(point3d);
    }
    ret = true;
  }
  return ret;
}
bool Lanelet2Data::haveCurrentRefIdRole(int ref_id, std::string &ref_role) {
  bool ret = false;
  for (auto &lanelet_value : map_->laneletLayer) {
    if (lanelet_value.id() == ref_id) {
      ref_role = "ref_lanelets";
      ret = true;
      return ret;
    }
  }
  for (auto &pl_value : map_->polygonLayer) {
    if (pl_value.id() == ref_id) {
      ref_role = "intersection";
      ret = true;
      return ret;
    }
  }
  for (auto &ls_value : map_->lineStringLayer) {
    if (ls_value.id() == ref_id) {
      ref_role = ls_value.attribute(lanelet::AttributeName::Type).value();
      ret = true;
      return ret;
    }
  }
  return ret;
}
/*递归函数
输入：lanelet地图上的一个点，lanelet地图上的线集
输出：lanelet地图上的线集
获得，距离点point一定范围，与point所在线，直接或间接相交的lanelet地图上的线集
TODO 可以再整理一下
*/
void Lanelet2Data::findLinestringswithEndPoint3d(
    lanelet::LineStrings3d &linestrings, lanelet::Point3d point) {
  // for lines neighbor these lines
  lanelet::LineStrings3d linestrings_nei;
  lanelet::Points3d endpoints;
  std::vector<double> distance_vec;
  for (auto &value : linestrings) {
    if (value.begin()->id() != point.id()) {
      lanelet::Point3d pt = *value.begin();
      endpoints.push_back(pt);
      double dis = pow((pt.x() - point.x()), 2) + pow((pt.y() - point.y()), 2);
      distance_vec.push_back(dis);
    }
    if ((value.end() - 1)->id() != point.id()) {
      lanelet::Point3d pt = *(value.end() - 1);
      endpoints.push_back(pt);
      double dis = pow((pt.x() - point.x()), 2) + pow((pt.y() - point.y()), 2);
      distance_vec.push_back(dis);
    }
  }
  sort(distance_vec.begin(), distance_vec.end(),
       [](double a, double b) { return a < b; });
  double dis = *(distance_vec.end() - 2);
  if (dis > 60) return;
  //---去重
  sort(endpoints.begin(), endpoints.end(),
       [](lanelet::Point3d a, lanelet::Point3d b) { return a.id() < b.id(); });
  endpoints.erase(unique(endpoints.begin(), endpoints.end(),
                         [](lanelet::Point3d a, lanelet::Point3d b) {
                           return a.id() == b.id();
                         }),
                  endpoints.end());

  for (auto &p : endpoints) {
    for (auto &value : map_->lineStringLayer) {
      if (!value.empty()) {
        if ((value.begin())->id() == p.id()) {
          linestrings_nei.push_back(value);
          lanelet::Point3d pt = *(value.begin());
          double dis =
              pow((pt.x() - point.x()), 2) + pow((pt.y() - point.y()), 2);
          distance_vec.push_back(dis);
        } else if ((value.end() - 1)->id() == p.id()) {
          linestrings_nei.push_back(value);
          lanelet::Point3d pt = *(value.end() - 1);
          double dis =
              pow((pt.x() - point.x()), 2) + pow((pt.y() - point.y()), 2);
          distance_vec.push_back(dis);
        }
      }
    }
  }
  // std::cout << "1...linestrings_nei.size() =" << linestrings_nei.size()
  //           << std::endl;
  // for (auto &p : linestrings_nei) {
  //   std::cout << "1...linestrings_nei line.id() =" << p.id() << std::endl;
  // }
  //
  linestrings.insert(linestrings.end(), linestrings_nei.begin(),
                     linestrings_nei.end());
  sort(linestrings.begin(), linestrings.end(),
       [](lanelet::LineString3d a, lanelet::LineString3d b) {
         return a.id() < b.id();
       });
  linestrings.erase(
      unique(linestrings.begin(), linestrings.end(),
             [](lanelet::LineString3d a, lanelet::LineString3d b) {
               return a.id() == b.id();
             }),
      linestrings.end());
  // std::cout << "1...linestrings.size() =" << linestrings.size() << std::endl;
  // for (auto &p : linestrings) {
  //   std::cout << "1...linestrings line.id() =" << p.id() << std::endl;
  // }
  //
  sort(distance_vec.begin(), distance_vec.end(),
       [](double a, double b) { return a < b; });
  dis = *(distance_vec.end() - 2);
  if (dis > 60) return;
  //
  findLinestringswithEndPoint3d(linestrings, point);
}

void Lanelet2Data::findLinestringswithPoint3d(
    lanelet::LineStrings3d &linestrings, lanelet::Point3d point) {
  // for lines with this point
  for (auto &value : map_->lineStringLayer) {
    if (!value.empty()) {
      for (auto &pt_value : value) {
        if (pt_value.id() == point.id()) {
          linestrings.push_back(value);
          break;
        }
      }
    }
  }
  // for lines neighbor these lines
  findLinestringswithEndPoint3d(linestrings, point);
}
// view
lanelet::ConstLineStrings3d Lanelet2Data::typeLineStrings(
    const lanelet::ConstLineStrings3d lts, const char type[]) {
  lanelet::ConstLineStrings3d type_linestrings;
  for (auto li = lts.begin(); li != lts.end(); li++) {
    lanelet::ConstLineString3d lt = *li;
    if (lt.hasAttribute(lanelet::AttributeName::Type)) {
      lanelet::Attribute attr = lt.attribute(lanelet::AttributeName::Type);
      if (attr.value() == type) {
        type_linestrings.push_back(lt);
      }
    }
  }
  return type_linestrings;
}
lanelet::ConstLanelets Lanelet2Data::subtypeLanelets(
    const lanelet::ConstLanelets lls, const char subtype[]) {
  lanelet::ConstLanelets subtype_lanelets;
  for (auto li = lls.begin(); li != lls.end(); li++) {
    lanelet::ConstLanelet ll = *li;
    if (ll.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype) {
        subtype_lanelets.push_back(ll);
      }
    }
  }
  return subtype_lanelets;
}
lanelet::ConstPoints3d Lanelet2Data::pointLayer() {
  lanelet::ConstPoints3d points;
  if (!map_) {
    std::cerr << "No map received!" << std::endl;
    return points;
  }

  for (auto li = map_->pointLayer.begin(); li != map_->pointLayer.end(); li++) {
    points.push_back(*li);
  }
  return points;
}
lanelet::ConstLineStrings3d Lanelet2Data::lineStringLayer() {
  lanelet::ConstLineStrings3d linestrings;
  if (!map_) {
    std::cerr << "No map received!" << std::endl;
    return linestrings;
  }

  for (auto li = map_->lineStringLayer.begin();
       li != map_->lineStringLayer.end(); li++) {
    linestrings.push_back(*li);
  }
  return linestrings;
}
lanelet::ConstLineStrings3d Lanelet2Data::stoplineLineStrings(
    const lanelet::ConstLineStrings3d lts) {
  return typeLineStrings(lts, lanelet::AttributeValueString::StopLine);
}
lanelet::ConstLineStrings3d Lanelet2Data::bumpLineStrings(
    const lanelet::ConstLineStrings3d lts) {
  return typeLineStrings(lts, lanelet::AttributeValueString::Bump);
}
lanelet::ConstLineStrings3d Lanelet2Data::stampLinethinLineStrings(
    const lanelet::ConstLineStrings3d lts,const lanelet::ConstLanelets lls) {
  std::string type = lanelet::AttributeValueString::LineThin;
  lanelet::ConstLineStrings3d stampls;
  for (auto li = lts.begin(); li != lts.end(); li++) {
    lanelet::ConstLineString3d lt = *li;
    bool isstampLine = true;
    if (lt.hasAttribute(lanelet::AttributeName::Type)) {
      lanelet::Attribute attr = lt.attribute(lanelet::AttributeName::Type);
      if (attr.value() == type) {
        for(auto ll : lls){
          if(ll.leftBound().id()== lt.id() && ll.rightBound().size()==0){
            break;
          }
          else if(ll.rightBound().id()== lt.id() && ll.leftBound().size()==0){
            break;
          }
          else if(ll.leftBound().id()== lt.id() || ll.rightBound().id()== lt.id()){
            isstampLine = false;
            break;
          }
        }
        if(isstampLine){
          stampls.push_back(lt);
        }
      }
    }
  }
  return stampls;
}
lanelet::ConstLanelets Lanelet2Data::laneletLayer() {
  lanelet::ConstLanelets lanelets;
  if (!map_) {
    std::cerr << "No map received!" << std::endl;
    return lanelets;
  }

  for (auto li = map_->laneletLayer.begin(); li != map_->laneletLayer.end();
       li++) {
    lanelets.push_back(*li);
  }
  return lanelets;
}
lanelet::ConstLanelets Lanelet2Data::crosswalkLanelets(
    const lanelet::ConstLanelets lls) {
  return subtypeLanelets(lls, lanelet::AttributeValueString::Crosswalk);
}
lanelet::ConstLanelets Lanelet2Data::roadLanelets(
    const lanelet::ConstLanelets lls) {
  return (subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}
lanelet::ConstPolygons3d Lanelet2Data::PolygonLayer() {
  lanelet::ConstPolygons3d polygons;
  if (!map_) {
    std::cerr << "No map received!" << std::endl;
    return polygons;
  }
  for (auto li = map_->polygonLayer.begin(); li != map_->polygonLayer.end();
       li++) {
    polygons.push_back(*li);
  }
  return polygons;
}

// }  // end namespace lanelet2tool
// }  // namespace cti
