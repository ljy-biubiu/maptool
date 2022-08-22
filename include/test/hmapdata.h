#ifndef __HMAPDATA_H__
#define __HMAPDATA_H__

#include <iostream>
#include <vector>
#include <fstream>

#include <stdio.h>
#include <string.h>

namespace cti {
namespace test {

struct PROPERTY_TEST{
    union {
        uint32_t property;
        struct {
            uint8_t LANE:8;
            uint8_t TRAVEL:8;
            uint8_t BYPASS:8;
            uint8_t FREE:8;
        }property_type;
    };
    PROPERTY_TEST(uint32_t data)
    {
        property = data;
    }
};
struct LTYPE_TEST{
    union {
        uint32_t ltype;
        struct {
            uint16_t LINKTYPE:16;
            uint16_t LINKNUM:16;
        }ltype_type;
    };
    LTYPE_TEST(uint32_t data)
    {
        ltype = data;
    }
};

struct Pose {
    double x;
    double y;
    double z;
    double ox;
    double oy;
    double oz;
    double ow;
};

struct Hdata {
    uint32_t  id;      
    uint32_t  property;  
    uint32_t  ltype;
    float     limit_vel;
    Pose      pose;
};

struct LaneData {
    uint32_t id; 
    Hdata    lane;
};

struct RoadEdgeData {
    uint32_t id;
    Hdata    roadedge;
};

struct ClearAreaData {
    uint32_t id;
    Hdata    cleararea;
};

class HmapData
{
public:
    HmapData();
    ~HmapData();
    bool loadLaneFile(const std::string file, std::string &err);
    bool loadRoadEdgeFile(const std::string file, std::string &err);
    bool loadClearAreaFile(const std::string file, std::string &err);
    void addLaneData(std::vector<LaneData> &datas);
    void addRoadEdgeData(std::vector<RoadEdgeData> &datas);
    void addClearAreaData(std::vector<ClearAreaData> &datas);
    //--data--
    std::vector<LaneData> lane_datas;
    std::vector<std::vector<LaneData>> lane_datas_lines;
    std::vector<RoadEdgeData> roadedge_datas;
    std::vector<std::vector<RoadEdgeData>> roadedge_datas_lines;
    std::vector<ClearAreaData> cleararea_datas;
    std::vector<std::vector<ClearAreaData>> cleararea_datas_lines;
private:
    bool getLineData(const std::string &str_line, const char *c, std::vector<double> &data);
};

}
}
#endif