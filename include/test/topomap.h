#ifndef __TOPOMAP_H__
#define __TOPOMAP_H__

#include <iostream>
#include <vector>
#include <stack>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include "test/hmapdata.h"
#include "test/linecommon.h"

namespace cti{
namespace test{

#define SEARCH_MAX_DIS (0.4)

typedef struct {
    int    start_id;
    int    end_id;
    bool   directed;
    double distance;
}TopoNode;

struct RuptTable {
    uint8_t     cmd;  //0:恢复正常 1:阻断 2:不可逆阻断
    std::string name; //阻断者名
    TopoNode    tnode;
};

class TopoMap
{
public:
    TopoMap();
    ~TopoMap();
    Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);
    Eigen::Vector3d quaterniond2Euler(const double x,const double y,const double z,const double w);
    LaneData findNearPointID(const Pose &pose);
    LaneData findNearPointID(const Pose &pose, std::vector<LaneData> &ods, const float findRandge=2.0);
    bool isIntersect(const Pose &pose,const LaneData &od,const float findRandge);
    int  findBestPointID(const Pose &check_pose, std::vector<LaneData> &ods, const float findRandge);
    void setOffsetForZ(const float min_offset, const float max_offset);
    void createTopoMap(std::vector<LaneData> &datas);
    void addRoadEdgeData(std::vector<RoadEdgeData> &datas);
    std::tuple<double, std::vector<LaneData>> find_path(uint32_t start_id, uint32_t end_id);
    void stop_path(uint32_t start_id, uint32_t end_id);
    std::vector<Pose> getDiscretizedPoints(const Pose &start, const Pose &end, const double interval);
    std::vector<Pose> getDiscretizedPointsMedian(const Pose &start, const Pose &end, const double interval);
    double computatDistance(const Pose &p1, const Pose &p2);
    bool getOriginalDataFromID(uint32_t id, LaneData &odata);
    bool isTransferID(int cur_id);
    std::vector<LaneData> getLaneHead();
    void denseOriginalDatas(std::vector<LaneData> &datas, const double interval);
    void denseOriginalDatas(std::vector<std::vector<LaneData>> &datas, const double interval);
    void denseOriginalDatasMedian(std::vector<LaneData> &datas, const double interval);
    void denseOriginalDatasMedian(std::vector<std::vector<LaneData>> &datas, const double interval);
protected:
    void createTopoNode(std::vector<std::vector<LaneData>> &data, std::vector<TopoNode> &toponode);
    void printfTopoNode(std::vector<TopoNode> &toponode);
    void addTopoMap(std::vector<TopoNode> &toponode);
private:
    std::vector<int> transferIDs;
    std::vector<LaneData> laneHeads;
    double min_offset_;
    double max_offset_;

    //保存一份的原始数据
    std::vector<LaneData> lane_datas_;
    std::vector<std::vector<LaneData>> lane_datas_lines_;
    std::vector<RoadEdgeData> roadedge_datas_;
    std::vector<std::vector<RoadEdgeData>> roadedge_datas_lines_;
};

}
}

#endif
