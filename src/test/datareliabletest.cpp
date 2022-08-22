#include "test/datareliabletest.h"

namespace cti {
namespace test {

DataReliableTest::DataReliableTest()
{
}

DataReliableTest::~DataReliableTest()
{
}

double DataReliableTest::computatDistance(const Pose &p1, const Pose &p2)
{
    return sqrt(pow((p2.x - p1.x), 2) + \
                pow((p2.y - p1.y), 2) + \
                pow((p2.z - p1.z), 2));
}

bool DataReliableTest::checkDatas(std::vector<std::vector<RoadEdgeData>> &datas, DataReliableResult &result)
{
    DataReliableResult::Result res;
    for(const auto data : datas)
    {
        RoadEdgeData data_buf;
        for(size_t i = 0; i < data.size(); i++)
        {
            if(std::isnan(data[i].roadedge.pose.x) || \
               std::isnan(data[i].roadedge.pose.y) || \
               std::isnan(data[i].roadedge.pose.z) || \
               std::isnan(data[i].roadedge.pose.ox) || \
               std::isnan(data[i].roadedge.pose.oy) || \
               std::isnan(data[i].roadedge.pose.oz) || \
               std::isnan(data[i].roadedge.pose.ow))
            {
                res.err = "data nan";
                res.type = "roadedge";
                res.id = data[i].roadedge.id;
                result.results.push_back(res);
            }
            if(std::isinf(data[i].roadedge.pose.x) || \
               std::isinf(data[i].roadedge.pose.y) || \
               std::isinf(data[i].roadedge.pose.z) || \
               std::isinf(data[i].roadedge.pose.ox) || \
               std::isinf(data[i].roadedge.pose.oy) || \
               std::isinf(data[i].roadedge.pose.oz) || \
               std::isinf(data[i].roadedge.pose.ow))
            {
                res.err = "data inf";
                res.type = "roadedge";
                res.id = data[i].roadedge.id;
                result.results.push_back(res);
            }
            if(i > 0)
            {
                double dis = computatDistance(data[i].roadedge.pose, data_buf.roadedge.pose);
                if(dis > POINTS_MAX_DIS)
                {
                    res.err = "data max_dis";
                    res.type = "roadedge";
                    res.id = data[i].roadedge.id;
                    result.results.push_back(res);
                }
            }
            data_buf = data[i];
        }
    }
    return result.results.size() == 0;
}

bool DataReliableTest::checkDatas(std::vector<std::vector<LaneData>> &datas, DataReliableResult &result)
{
    DataReliableResult::Result res;
    for(const auto data : datas)
    {
        LaneData data_buf;
        for(size_t i = 0; i < data.size(); i++)
        {
            if(std::isnan(data[i].lane.pose.x) || \
               std::isnan(data[i].lane.pose.y) || \
               std::isnan(data[i].lane.pose.z) || \
               std::isnan(data[i].lane.pose.ox) || \
               std::isnan(data[i].lane.pose.oy) || \
               std::isnan(data[i].lane.pose.oz) || \
               std::isnan(data[i].lane.pose.ow))
            {
                res.err = "data nan";
                res.type = "lane";
                res.id = data[i].lane.id;
                result.results.push_back(res);
            }
            if(std::isinf(data[i].lane.pose.x) || \
               std::isinf(data[i].lane.pose.y) || \
               std::isinf(data[i].lane.pose.z) || \
               std::isinf(data[i].lane.pose.ox) || \
               std::isinf(data[i].lane.pose.oy) || \
               std::isinf(data[i].lane.pose.oz) || \
               std::isinf(data[i].lane.pose.ow))
            {
                res.err = "data inf";
                res.type = "lane";
                res.id = data[i].lane.id;
                result.results.push_back(res);
            }
            if(i > 0)
            {
                double dis = computatDistance(data[i].lane.pose, data_buf.lane.pose);
                if(dis > POINTS_MAX_DIS)
                {
                    res.err = "data max_dis";
                    res.type = "lane";
                    res.id = data[i].lane.id;
                    result.results.push_back(res);
                }
            }
            data_buf = data[i];
        }
    }
    return result.results.size() == 0;
}

bool DataReliableTest::checkDatas(std::vector<std::vector<ClearAreaData>> &datas, DataReliableResult &result)
{
    DataReliableResult::Result res;
    for(const auto data : datas)
    {
        ClearAreaData data_buf;
        for(size_t i = 0; i < data.size(); i++)
        {
            if(std::isnan(data[i].cleararea.pose.x) || \
               std::isnan(data[i].cleararea.pose.y) || \
               std::isnan(data[i].cleararea.pose.z) || \
               std::isnan(data[i].cleararea.pose.ox) || \
               std::isnan(data[i].cleararea.pose.oy) || \
               std::isnan(data[i].cleararea.pose.oz) || \
               std::isnan(data[i].cleararea.pose.ow))
            {
                res.err = "data nan";
                res.type = "cleararea";
                res.id = data[i].cleararea.id;
                result.results.push_back(res);
            }
            if(std::isinf(data[i].cleararea.pose.x) || \
               std::isinf(data[i].cleararea.pose.y) || \
               std::isinf(data[i].cleararea.pose.z) || \
               std::isinf(data[i].cleararea.pose.ox) || \
               std::isinf(data[i].cleararea.pose.oy) || \
               std::isinf(data[i].cleararea.pose.oz) || \
               std::isinf(data[i].cleararea.pose.ow))
            {
                res.err = "data inf";
                res.type = "cleararea";
                res.id = data[i].cleararea.id;
                result.results.push_back(res);
            }
            if(i > 0)
            {
                double dis = computatDistance(data[i].cleararea.pose, data_buf.cleararea.pose);
                if(dis > POINTS_MAX_DIS)
                {
                    res.err = "data max_dis";
                    res.type = "cleararea";
                    res.id = data[i].cleararea.id;
                    result.results.push_back(res);
                }
            }
            data_buf = data[i];
        }
    }
    return result.results.size() == 0;
}

bool DataReliableTest::checkPolygonDatas(std::vector<std::vector<ClearAreaData>> &datas, DataReliableResult &result)
{
    bool ret = true;
    DataReliableResult::Result res;
    for(const auto data : datas)
    {
        ClearAreaData data_buf;
        if(data.size() < 1 || \
          (data.front().cleararea.pose.x == data.back().cleararea.pose.x && \
           data.front().cleararea.pose.y == data.back().cleararea.pose.y))
        {
            continue;
        }
        //--
        for(size_t i = 0; i < data.size()-1; i++)
        {
            LineCommon line_start(data[i].cleararea.pose.x,data[i].cleararea.pose.y,data[i+1].cleararea.pose.x,data[i+1].cleararea.pose.y);
            for(size_t j = i+2; j < data.size()-1; j++)
            {
                LineCommon line_next(data[j].cleararea.pose.x,data[j].cleararea.pose.y,data[j+1].cleararea.pose.x,data[j+1].cleararea.pose.y);
                if(line_start.isIntersect(line_next))
                {
                    res.id = data[i].cleararea.id;
                    res.type = "cleararea";
                    res.err = "清扫区域多边形相交";
                    result.results.push_back(res);
                    ret = false;
                }
            }
        }
    }
    return ret;
}

bool DataReliableTest::startDataReliableTest(HmapData *data, DataReliableResult &result)
{
    checkDatas(data->lane_datas_lines,result);
    checkDatas(data->roadedge_datas_lines,result);
    checkDatas(data->cleararea_datas_lines,result);
    checkPolygonDatas(data->cleararea_datas_lines,result);
    return result.results.size() == 0;
}

}
}