#ifndef MAPDATA_FORMAT_H
#define MAPDATA_FORMAT_H

#include <tuple>

#include "geo/geo.h"
#include "vmap_data/datatype.h"

class MapDataFormat {
 private:
  /* data */
 public:
  MapDataFormat(/* args */);
  ~MapDataFormat();

  //数据属性的标准化
  bool mapdataOriginLonLat(map_projection_reference_s origin_pos);
  bool mapdataRangeXY(Way_Point_ &wp);
  bool mapdataRangeZ(Way_Point_ &wp);
  bool mapdataRangeQuate(Way_Point_ &wp);
  bool mapdataProperty(int type_id, Way_Point_ &wp);
  bool mapdataLinktype(Way_Point_ &wp);
  bool mapdataLimitVelocity(Way_Point_ &wp);
  bool mapdataLaneletId(int type_id, Way_Point_ &wp);
  bool vmapdataFormat(int type_id, Way_Point_ &wp,
                      map_projection_reference_s origin_pos);
};

#endif