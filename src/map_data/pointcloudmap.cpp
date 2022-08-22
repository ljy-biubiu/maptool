#include "map_data/pointcloudmap.h"

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

PointCloudMap::PointCloudMap()
{
  this->map_.reset(new pcl::PointCloud<PointType>);
}
PointCloudMap::~PointCloudMap()
{
}

pcl::PointCloud<PointType>::Ptr PointCloudMap::getMap()
{
  if (!this->map_)
  {
    return NULL;
  }
  return this->map_;
}

bool PointCloudMap::createMap(const pcl::PointCloud<PointType>::Ptr &cloud, const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation)
{
  if(cloud->points.size() == 0)
  {
    return false;
  }
  map_ = cloud;
  map_->header.frame_id = "/map";
  double map_init_lat = 1e-3 * origin.x() + 1e-3 * orientation.x();
  double map_init_lon = 1e-3 * origin.y() + 1e-3 * orientation.y();
  double map_init_alt = 1e-3 * origin.z() + 1e-3 * orientation.z();
  printf("map init lat %lf ,lon %lf , alt %lf\n", map_init_lat, map_init_lon, map_init_alt);
  printf("try map projection init!!\n");
  //init map_ lat lon
  if ((int)map_init_lat !=0 && (int)map_init_lon !=0)
  { 
    mapinit_lonlat_.latitude = map_init_lat;
    mapinit_lonlat_.longitude = map_init_lon;
    mapinit_lonlat_.isInit = true;
    map_projection_init(&origin_pos_, map_init_lat, map_init_lon);
    origin_pos_.init_done = true;
  }
  return true;
}

bool PointCloudMap::createMap(const std::vector<std::string> &pcd_paths)
{
  Eigen::Vector4f origin = Eigen::Vector4f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  pcl::PCLPointCloud2 pcd, part;
  for (const std::string &path : pcd_paths)
  {
    if (!std::ifstream(path.c_str()))
    {
      std::cerr << "File Not exist!! Please Check file path: " << path << std::endl;
      continue;
    }
    // Following outputs are used for progress bar of Runtime Manager.
    if (pcd.width == 0)
    {
      if (pcl::io::loadPCDFile(path.c_str(), pcd, origin, orientation) == -1)
      {
        std::cerr << "load failed: " << path << std::endl;
        continue;
      }
    }
    else
    {
      if (pcl::io::loadPCDFile(path.c_str(), part) == -1)
      {
        std::cerr << "load failed " << path << std::endl;
        continue;
      }
      pcd.width += part.width;
      pcd.row_step += part.row_step;
      pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
    }
  }
  if(pcd.width == 0)
  {
    std::cerr << "map data empty!" << std::endl;
    return false;
  }
  pcl::fromPCLPointCloud2(pcd, *map_);
  map_->header.frame_id = "/map";
  double map_init_lat = 1e-3 * origin.x() + 1e-3 * orientation.x();
  double map_init_lon = 1e-3 * origin.y() + 1e-3 * orientation.y();
  double map_init_alt = 1e-3 * origin.z() + 1e-3 * orientation.z();
  printf("map init lat %lf ,lon %lf , alt %lf\n", map_init_lat, map_init_lon, map_init_alt);
  printf("try map projection init!!\n");
  //init map_ lat lon
  if ((int)map_init_lat !=0 && (int)map_init_lon !=0)
  {
    mapinit_lonlat_.latitude = map_init_lat;
    mapinit_lonlat_.longitude = map_init_lon;
    mapinit_lonlat_.isInit = true;
    map_projection_init(&origin_pos_, map_init_lat, map_init_lon);
    std::cout << "origin初始信息： " << origin_pos_.lat_rad << " " << origin_pos_.lon_rad << std::endl;
    origin_pos_.init_done = true;
    std::cout << "origin_pos_.init_done =true !!!" << std::endl;
  }
  return true;
}

bool PointCloudMap::createMap(const std::string &pcd_path)
{
  Eigen::Vector4f origin = Eigen::Vector4f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  if (!std::ifstream(pcd_path.c_str()))
  {
    std::cerr << "File Not exist!! Please Check file path: " << pcd_path.c_str() << std::endl;
    return false;
  }
  pcl::PCLPointCloud2 pcd;
  if (pcl::io::loadPCDFile(pcd_path.c_str(), pcd, origin, orientation) < 0)
  {
    std::cerr << "Open PCD File Failed!! Please Check file path: " << pcd_path << std::endl;
    return false;
  }
  if(pcd.width == 0)
  {
    std::cerr << "map data empty!" << std::endl;
    return false;
  }
  pcl::fromPCLPointCloud2(pcd, *map_);
  map_->header.frame_id = "/map";
  double map_init_lat = 1e-3 * origin.x() + 1e-3 * orientation.x();
  double map_init_lon = 1e-3 * origin.y() + 1e-3 * orientation.y();
  double map_init_alt = 1e-3 * origin.z() + 1e-3 * orientation.z();
  printf("map init lat %lf ,lon %lf , alt %lf\n", map_init_lat, map_init_lon, map_init_alt);
  printf("try map projection init!!\n");
  //init map lat lon
  origin_pos_ = map_projection_reference_s();// TO DO new
  if ((int)map_init_lat !=0 && (int)map_init_lon !=0)
  {
    mapinit_lonlat_.latitude = map_init_lat;
    mapinit_lonlat_.longitude = map_init_lon;
    mapinit_lonlat_.isInit = true;
    map_projection_init(&origin_pos_, map_init_lat, map_init_lon);
    std::cout << "pcd_origin初始信息： " << origin_pos_.lat_rad << " " << origin_pos_.lon_rad << std::endl;
    origin_pos_.init_done = true;
    std::cout << "origin_pos_.init_done =true !!!" <<std::endl;
  }
  return true;
}

bool PointCloudMap::loadMap(const std::string &pcd_path)
{
  if (!std::ifstream(pcd_path.c_str()))
  {
    std::cerr << "File Not exist!! Please Check file path: " << pcd_path.c_str() << std::endl;
    return false;
  }
  pcl::PCLPointCloud2 pcd;
  if (pcl::io::loadPCDFile(pcd_path.c_str(), pcd) < 0)
  {
    std::cerr << "Open PCD File Failed!! Please Check file path: " << pcd_path << std::endl;
    return false;
  }
  if(pcd.width == 0)
  {
    std::cerr << "map data empty!" << std::endl;
    return false;
  }
  pcl::fromPCLPointCloud2(pcd, *map_);
  return true;
}

pcl::PointCloud<PointType>::Ptr PointCloudMap::clipMap(const double &zmin, const double &zmax)
{
  pcl::PointCloud<PointType>::Ptr cloud_filter(new pcl::PointCloud<PointType>());
  pcl::PassThrough<PointType> passthrough;
  passthrough.setInputCloud(map_);//输入点云
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(zmin, zmax);
  passthrough.filter(*cloud_filter);
  return cloud_filter;
}

pcl::PointCloud<PointType>::Ptr PointCloudMap::downSampling(const double &leafsize)
{
  pcl::PointCloud<PointType>::Ptr cloud_filter(new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setInputCloud(map_);
  voxelgrid.setLeafSize(leafsize, leafsize, leafsize);
  voxelgrid.filter(*cloud_filter);
  return cloud_filter;
}

ExtremumInfo PointCloudMap::getMapExtremum()
{
  ExtremumInfo info;
  bool fist_in = false;
  double min_value, max_value, min_intensity, max_intensity;
  for (auto point : map_->points)
  {
    if (!fist_in)
    {
      min_value = point.z;
      max_value = point.z;
      min_intensity = point.intensity;
      max_intensity = point.intensity;
      fist_in =true;
    }
    min_value = MIN(min_value, point.z);
    max_value = MAX(max_value, point.z);
    min_intensity = MIN(min_intensity, point.intensity);
    max_intensity = MAX(max_intensity, point.intensity);
  }
  info.min_value = min_value;
  info.max_value = max_value;
  info.min_intensity = min_intensity;
  info.max_intensity = max_intensity;
  return info;
}
map_projection_reference_s PointCloudMap::getMapOriginPostion()
{
  return origin_pos_;
}

LonAndLat PointCloudMap::getMapLatLon() {
  return mapinit_lonlat_;
}
