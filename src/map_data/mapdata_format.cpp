#include "map_data/mapdata_format.h"

// #include "cti_vmap_impl/vmap_attributes_impl.h"

# define M_PI        3.14159265358979323846
#define LON_MIN 73.0/180.0*M_PI
#define LON_MAX 136.0/180.0*M_PI
#define LAT_MIN 3.0/180.0*M_PI
#define LAT_MAX 54.0/180.0*M_PI
#define XY_MIN -10000.0
#define XY_MAX 10000.0
#define Z_MIN -1000.0
#define Z_MAX 1000.0
#define QUATE_MIN -1.0
#define QUATE_MAX 1.0
#define LANELETID_MAX 300000

MapDataFormat::MapDataFormat(/* args */) {}

MapDataFormat::~MapDataFormat() {}
bool MapDataFormat::mapdataOriginLonLat(map_projection_reference_s origin_pos) {
  if (!origin_pos.init_done) {
    return true;
  } else if ((LAT_MIN < origin_pos.lat_rad && origin_pos.lat_rad < LAT_MAX) &&
             (LON_MIN < origin_pos.lon_rad && origin_pos.lon_rad < LON_MAX)) {
    return true;
  }
  return false;
}

bool MapDataFormat::mapdataRangeXY(Way_Point_ &wp) {
  if ((XY_MIN < wp.point.x && wp.point.x < XY_MAX) &&
      (XY_MIN < wp.point.y && wp.point.y < XY_MAX)) {
    return true;
  } else {
    return false;
  }
}
bool MapDataFormat::mapdataRangeZ(Way_Point_ &wp) {
  if (XY_MIN < wp.point.z && wp.point.z < XY_MAX) {
    return true;
  } else {
    return false;
  }
}
bool MapDataFormat::mapdataRangeQuate(Way_Point_ &wp) {
  if ((QUATE_MIN <= wp.point.quate_x && wp.point.quate_x <= QUATE_MAX) &&
      (QUATE_MIN <= wp.point.quate_y && wp.point.quate_y <= QUATE_MAX) &&
      (QUATE_MIN <= wp.point.quate_z && wp.point.quate_z <= QUATE_MAX) &&
      (QUATE_MIN <= wp.point.quate_w && wp.point.quate_w <= QUATE_MAX)) {
    return true;
  } else {
    return false;
  }
}
bool MapDataFormat::mapdataProperty(int type_id, Way_Point_ &wp) {}
bool MapDataFormat::mapdataLinktype(Way_Point_ &wp) {
  int typeNumId = wp.ltype_type.LINKNUM;
  if (wp.ltype_type.LINKTYPE < 0 || wp.ltype_type.LINKTYPE > (int)Vm_T::Type_Num || typeNumId < 0) {
    std::cout<<wp.ltype_type.LINKTYPE<<";"<<typeNumId<<std::endl;
    return false;
  } else {
    return true;
  }
}
bool MapDataFormat::mapdataLimitVelocity(Way_Point_ &wp) {
  if (wp.limit_vel < 0) {
    return false;
  } else {
    return true;
  }
}
bool MapDataFormat::mapdataLaneletId(int type_id, Way_Point_ &wp) {
  if ((Vm_T)type_id != Vm_T::Lane) {
    wp.lanelet_id = 0;
    return true;
  } else if (wp.lanelet_id < 0) {
    return false;
  }

  if ((Vm_T)type_id == Vm_T::Lane && wp.lanelet_id >= 0) {
    if (wp.lanelet_id >= LANELETID_MAX) {
      wp.lanelet_id = 0;
    }
    return true;
  }
}
bool MapDataFormat::vmapdataFormat(int type_id, Way_Point_ &wp,
                                map_projection_reference_s origin_pos) {
  bool ret = true;
  if (!mapdataOriginLonLat(origin_pos)) {
    ret = false; 
    std::cout << "注意：经纬度初始化，失败！!!" << std::endl;

  }
  if(!mapdataRangeXY(wp)){
    ret = false;
    std::cout << "注意：局部坐标XY，不在范围内！!!" << std::endl;
  }
  if(!mapdataRangeZ(wp)){
    ret = false;
    std::cout << "注意：局部坐标Z，不在范围内！!!" << std::endl;
  }
  if(!mapdataRangeQuate(wp)){
    ret = false;
    std::cout << "注意：出现无效的四元数！!!" << std::endl;
  }
  if(!mapdataLinktype(wp)){
    ret = false;
    std::cout << "注意：出现无效的链接类型, 或者无效的链接id！!!" << std::endl;
  }
  if(!mapdataLimitVelocity(wp)){
    ret = false;
    std::cout << "注意：速度限制不能为负！!!" << std::endl;
  }
  if(!mapdataLaneletId(type_id, wp)){
    ret = false;
    std::cout << "注意：出现lane类型链接无效的lanelet id！!!" << std::endl;
  }
  return ret;
}

/*1.初始经纬度，在中国范围；北纬度，3度-54度；东经度，73-136度
  2.x,y,z的范围：x,y 正负十公里范围，z 正负1000米；-10000,10000，-1000,1000
  3.altitude==z
  4.四元数：范围-1,1
  5.属性：
  6.链接属性：LINKTYPE:<Vm_T; LINKNUM>=0 int
  7.限速度：>=0
  8.lanelet_id: >=0 int
  9.有问题，打印后，进行是否仍然保存的提示
 */