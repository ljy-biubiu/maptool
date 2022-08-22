#ifndef __ROUTETEST_H__
#define __ROUTETEST_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stack>
#include <stdio.h>

#include "test/topomap.h"
#include "test/linecommon.h"

namespace cti {
namespace test {

struct RouteResult
{
    struct Result
    {
        std::string type;
        uint32_t start_id;
        uint32_t end_id;
        Pose     start_pose;
        Pose     end_pose;
        std::string err;
    };
    std::vector<Result> results;
};

class RouteTest
{
public:
    RouteTest();
    ~RouteTest();
    bool startRouteTest(HmapData *data, RouteResult &result);
    void extraLaneHeads(const std::vector<LaneData> &datas);
private:
    void init();
    bool routeTest(const Pose &start_pose, const Pose &end_pose, std::string &err);
    int  isHaveSameID(const LaneData &od, const std::vector<LaneData> &fods);
    void dealPath(std::vector<LaneData> &ods);
    void checkPath(const Pose &start_pose, const LaneData &start_od, const std::vector<LaneData> &start_ods, std::vector<LaneData> &ods);
    int  getPath(std::vector<LaneData> &lanedata, \
                 const Pose &start_pose, \
                 const std::vector<LaneData> &start_ods, \
                 const Pose &end_pose, \
                 const std::vector<LaneData> &end_ods);
private:
    TopoMap topomap;
    double turnRadiusK_;
    bool densemedian_;
    double denseinterval_;
    double search_path_distance_;
    double limit_angle_;
    double max_offset_;
    double min_offset_;
    double serch_near_;
    std::vector<LaneData> lane_heads_;
};

}
}
#endif
