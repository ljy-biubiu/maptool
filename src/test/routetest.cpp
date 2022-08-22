#include "test/routetest.h"

namespace cti {
namespace test {

RouteTest::RouteTest()
{
    init();
}

RouteTest::~RouteTest()
{
}

void RouteTest::init()
{
    densemedian_ = true;
    denseinterval_ = 0.5;
    search_path_distance_ = 15;
    turnRadiusK_ = 2.0;
    limit_angle_ = 90;
    max_offset_ = 3.5;
    min_offset_ = -3.5;
    serch_near_ = 0.5;
    topomap.setOffsetForZ(min_offset_,max_offset_);
}

//return -1:没有找到，>=0:找到返回对应序号
int RouteTest::isHaveSameID(const LaneData &od, const std::vector<LaneData> &fods)
{
    int i=0;
    for(auto fod : fods){
        if(fod.id == od.id){
            return i;
        }
        i++;
    }
    return (-1);
}

void RouteTest::checkPath(const Pose &start_pose, \
                          const LaneData &start_od, \
                          const std::vector<LaneData> &start_ods, \
                          std::vector<LaneData> &ods)
{
    std::vector<LaneData> temp_ods;
    for(auto od : ods)
    {
        //查找临近点
        int index = isHaveSameID(od,start_ods);
        if(index >= 0)
        {
            if(start_ods[index].id != start_od.id && topomap.computatDistance(start_pose,start_ods[index].lane.pose) <= turnRadiusK_)
            {
                temp_ods.clear();
                temp_ods.push_back(start_ods[index]);
                continue;
            }
        }
        temp_ods.push_back(od);
    }
    ods.clear();
    ods = temp_ods;
}

int RouteTest::getPath(std::vector<LaneData> &originaldata, \
                       const Pose &start_pose, \
                       const std::vector<LaneData> &start_ods, \
                       const Pose &end_pose, \
                       const std::vector<LaneData> &end_ods)
{
    int ret = 0;
    if(start_ods.size() == 0 || end_ods.size() == 0)
    {
        return ret;
    }
    originaldata.clear();
    LaneData start_od = start_ods.at(0);
    LaneData end_od = end_ods.at(0);
    //两点距离很短
    if(topomap.computatDistance(start_pose,start_od.lane.pose) > turnRadiusK_ && \
       topomap.computatDistance(start_pose,end_pose) < turnRadiusK_)
    {
        ret = 2;
        LaneData data = start_od;
        data.lane.pose = start_pose;
        originaldata.push_back(data);
        data.lane.pose = end_pose;
        originaldata.push_back(data);
        return ret;
    }
    //--获取最短路径
    double distance = -1;
    int size = 0;
    std::tie(distance, originaldata) = topomap.find_path(start_od.id,end_od.id);
    size = originaldata.size();
    std::cout << "check " << start_od.id << " to " << end_od.id << ",distance=" << distance << ",size=" << size << std::endl;
    if(distance < 0 || size == 0)
    {
        if(start_ods.size() > 2)
        {
            for(int i = 1; i < start_ods.size(); i++)
            {
                std::tie(distance, originaldata) = topomap.find_path(start_ods.at(i).id,end_od.id);
                size = originaldata.size();
                std::cout << "check " << start_ods.at(i).id << " to " << end_od.id << ",distance=" << distance << ",size=" << size << std::endl;
                if(distance > 0 && size > 0)
                {
                    start_od = start_ods[i];
                    break;
                }
            }
        }
    }
    if(size == 1)
    {
        if(topomap.computatDistance(start_od.lane.pose, end_od.lane.pose) > turnRadiusK_)
        {
            originaldata.push_back(originaldata.back());
            originaldata.push_back(originaldata.back());
        }
        originaldata.front().lane.pose = start_od.lane.pose;
        originaldata.back().lane.pose = end_od.lane.pose;
    }
    else if(size > 1)
    {
        double dis1, dis2, dis3;
        dis1 = topomap.computatDistance(start_od.lane.pose,originaldata.at(0).lane.pose);
        dis2 = topomap.computatDistance(start_od.lane.pose,originaldata.at(1).lane.pose);
        dis3 = topomap.computatDistance(originaldata.at(0).lane.pose,originaldata.at(1).lane.pose);
        if((dis1 + dis2 - dis3) >= 1e-1)
        {
            originaldata.insert(originaldata.begin(), start_od);
        }
        else
        {
            originaldata[0] = start_od;
        }
        dis1 = topomap.computatDistance(end_od.lane.pose,originaldata.at(size-1).lane.pose);
        dis2 = topomap.computatDistance(end_od.lane.pose,originaldata.at(size-2).lane.pose);
        dis3 = topomap.computatDistance(originaldata.at(size-1).lane.pose,originaldata.at(size-2).lane.pose);
        if((dis1 + dis2 - dis3) >= 1e-1)
        {
            originaldata.push_back(end_od);
        }
        else
        {
            originaldata[size-1] = end_od;
        }
    }
    else
    {
        return ret;
    }
    //--处理头部尾部曲线,处理尾部曲线
    if(start_ods.size() > 1)
    {
        checkPath(start_pose,start_od,start_ods,originaldata);
    }
    //--
    if(end_ods.size() > 1)
    {
        checkPath(end_pose,end_od,end_ods,originaldata);
    }
    LaneData tmp_od;
    tmp_od.lane.limit_vel = 5;
    tmp_od.lane.pose = start_pose;
    originaldata.insert(originaldata.begin(), tmp_od);
    tmp_od.lane.pose = end_pose;
    originaldata.push_back(tmp_od);
    return originaldata.size();
}

bool RouteTest::routeTest(const Pose &start_pose, const Pose &end_pose, std::string &err)
{
    //--查询最优点--
    std::vector<LaneData> start_nearods, end_nearods;
    int ret = 0;
    ret = topomap.findBestPointID(start_pose,start_nearods,serch_near_);
    if(ret < 0 || start_nearods.size() == 0) 
    {
        if(ret == -1){
            err = "起点离主路太远";
        }
        else if(ret == -2){
            err = "起点路径与路边线相交";
        }
        else{
            err = "未知原因";
        }
        return false;
    }
    ret = topomap.findBestPointID(end_pose,end_nearods,serch_near_);
    if(ret < 0 || end_nearods.size() == 0)
    {
        if(ret == -1){
            err = "终点离主路太远";
        }
        else if(ret == -2){
            err = "终点路径与路边线相交";
        }
        else{
            err = "未知原因";
        }
        return false;
    }
    //--获取最优点路径--
    std::vector<LaneData> originaldata;
    ret = getPath(originaldata,start_pose,start_nearods,end_pose,end_nearods);
    if(ret <= 0) //路径规划失败
    {
        err = "路径不通，请检查主路线";
        return false;
    }
    return true;
}

void RouteTest::extraLaneHeads(const std::vector<LaneData> &datas)
{
    lane_heads_ = datas;
}

bool RouteTest::startRouteTest(HmapData *data, RouteResult &result) //通路测试
{
    bool ret = true;
    //--创建路线--
    topomap.createTopoMap(data->lane_datas);
    topomap.addRoadEdgeData(data->roadedge_datas);
    //--获取不带粘合点线头--
    std::vector<LaneData> lane_heads = topomap.getLaneHead();
    //--增加额外点数据--
    if(lane_heads_.size() > 0)
    {
        lane_heads.insert(lane_heads.end(),lane_heads_.begin(),lane_heads_.end());
    }
    //--轮询起点目标点--
    for(auto start : lane_heads)
    {
        for(auto end : lane_heads)
        {
            if(start.id == end.id)
            {
                continue;
            }
            RouteResult::Result res;
            std::string message;
            if(!routeTest(start.lane.pose, end.lane.pose, message))
            {
                res.start_id = start.lane.id;
                res.start_pose = start.lane.pose;
                res.end_id = end.lane.id;
                res.end_pose = end.lane.pose;
                res.err = message;
                result.results.push_back(res);
                ret = false;
            }
        }
    }
    return ret;
}

}
}