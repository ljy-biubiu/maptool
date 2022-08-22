#ifndef POINTCLOUD_MAP_H
#define POINTCLOUD_MAP_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

#include "geo/geo.h"

typedef pcl::PointXYZI PointType;

struct ExtremumInfo
{
  double min_intensity;
  double max_intensity;
  double min_value;
  double max_value;
};

struct LonAndLat
{
  double latitude;   //纬度
  double longitude;  //经度
  bool isInit;
};

class PointCloudMap
{
public:
  PointCloudMap();
  ~PointCloudMap();
  //--
  bool createMap(const pcl::PointCloud<PointType>::Ptr &cloud, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation);
  bool createMap(const std::vector<std::string> &pcd_paths);
  bool createMap(const std::string &pcd_path);
  bool loadMap(const std::string &pcd_path);
  pcl::PointCloud<PointType>::Ptr clipMap(const double &zmin, const double &zmax);
  pcl::PointCloud<PointType>::Ptr downSampling(const double &leafsize);
  map_projection_reference_s getMapOriginPostion();
  pcl::PointCloud<PointType>::Ptr getMap();
  ExtremumInfo getMapExtremum();
  LonAndLat getMapLatLon();
private:
  pcl::PointCloud<PointType>::Ptr map_;
  map_projection_reference_s origin_pos_;
  LonAndLat mapinit_lonlat_;
};

#endif  // POINTCLOUD_MAP_H
