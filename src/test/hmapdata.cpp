#include "test/hmapdata.h"

namespace cti {
namespace test {

HmapData::HmapData()
{
}
HmapData::~HmapData()
{
}

bool HmapData::getLineData(const std::string &str_line, const char *c, std::vector<double> &data)
{
    if(str_line.find(c) == 0)
    {
        return false;
    }
    char *s_input = (char *)str_line.c_str();
    char *p = strtok(s_input,c);
    while(p != NULL)
    {
        double a = 0.0;
        sscanf(p,"%lf",&a);
        data.push_back(a);
        p = strtok(NULL,c);
    }
    return true;
}

bool HmapData::loadLaneFile(const std::string file, std::string &err)
{
    uint32_t count = 0;
    std::ifstream ifs;
    std::string   str_line;
    std::vector<LaneData> datas;
    ifs.open(file.c_str());
    if(ifs.is_open())
    {
        while(!ifs.eof())
        {
            std::getline(ifs, str_line);
            std::vector<double> buf;
            getLineData(str_line,",",buf);
            //--根据文件格式--
            LaneData temp;
            if(buf.size() > 14)
            {   
                temp.lane.id = buf[1];
                temp.lane.pose.x = buf[2];
                temp.lane.pose.y = buf[3];
                temp.lane.pose.z = buf[4];
                temp.lane.pose.ox = buf[5];
                temp.lane.pose.oy = buf[6];
                temp.lane.pose.oz = buf[7];
                temp.lane.pose.ow = buf[8];
                temp.lane.property = buf[12];
                temp.lane.ltype = buf[13];
                temp.lane.limit_vel = buf[14];
                //--
                temp.id = count;
                datas.push_back(temp);
                count++;
            }
        }
    }
    else
    {
        err = "file: " + file + " open error!";
        std::cerr << err << std::endl;
        return false;
    }
    if(datas.size() == 0)
    {
        err = "file: " + file + " empty data error!";
        std::cerr << err << std::endl;
        return false;
    }
    else
    {
        addLaneData(datas);
        return true;
    }
}
bool HmapData::loadRoadEdgeFile(const std::string file, std::string &err)
{
    int count = 0;
    std::ifstream ifs;
    std::string   str_line;
    std::vector<RoadEdgeData> datas;
    ifs.open(file.c_str());
    if(ifs.is_open())
    {
        while(!ifs.eof())
        {
            std::getline(ifs, str_line);
            std::vector<double> buf;
            getLineData(str_line,",",buf);
            //--根据文件格式--
            RoadEdgeData temp;
            if(buf.size() > 14)
            {   
                temp.roadedge.id = buf[1];
                temp.roadedge.pose.x = buf[2];
                temp.roadedge.pose.y = buf[3];
                temp.roadedge.pose.z = buf[4];
                temp.roadedge.pose.ox = buf[5];
                temp.roadedge.pose.oy = buf[6];
                temp.roadedge.pose.oz = buf[7];
                temp.roadedge.pose.ow = buf[8];
                temp.roadedge.property = buf[12];
                temp.roadedge.ltype = buf[13];
                //--
                temp.id = count;
                datas.push_back(temp);
                count++;
            }
        }
    }
    else
    {
        err = "file: " + file + " open error!";
        std::cerr << err << std::endl;
        return false;
    }
    if(datas.size() == 0)
    {
        err = "file: " + file + " empty data error!";
        std::cerr << err << std::endl;
        return false;
    }
    else
    {
        addRoadEdgeData(datas);
        return true;
    }
}

bool HmapData::loadClearAreaFile(const std::string file, std::string &err)
{
    int count = 0;
    std::ifstream ifs;
    std::string   str_line;
    std::vector<ClearAreaData> datas;
    ifs.open(file.c_str());
    if(ifs.is_open())
    {
        while(!ifs.eof())
        {
            std::getline(ifs, str_line);
            std::vector<double> buf;
            getLineData(str_line,",",buf);
            //--根据文件格式--
            ClearAreaData temp;
            if(buf.size() > 14)
            {   
                temp.cleararea.id = buf[1];
                temp.cleararea.pose.x = buf[2];
                temp.cleararea.pose.y = buf[3];
                temp.cleararea.pose.z = buf[4];
                temp.cleararea.pose.ox = buf[5];
                temp.cleararea.pose.oy = buf[6];
                temp.cleararea.pose.oz = buf[7];
                temp.cleararea.pose.ow = buf[8];
                temp.cleararea.property = buf[12];
                temp.cleararea.ltype = buf[13];
                //--
                temp.id = count;
                datas.push_back(temp);
                count++;
            }
        }
    }
    else
    {
        err = "file: " + file + " open error!";
        std::cerr << err << std::endl;
        return false;
    }
    if(datas.size() == 0)
    {
        err = "file: " + file + " empty data error!";
        std::cerr << err << std::endl;
        return false;
    }
    else
    {
        addClearAreaData(datas);
        return true;
    }
}

void HmapData::addRoadEdgeData(std::vector<RoadEdgeData> &datas)
{
    roadedge_datas = datas;
    roadedge_datas_lines.clear();
    if(datas.empty())
    {
        std::cerr << "roadedge输入的数据为空." << std::endl;
        return;
    }
    std::vector<RoadEdgeData> od;
    //划分每一条线
    int eid = datas.front().roadedge.id;
    for(auto d : datas)
    {
        if(d.roadedge.id != eid)
        {
            eid = d.roadedge.id;
            roadedge_datas_lines.push_back(od);
            od.clear();
            od.push_back(d);
        }
        else
        {
            od.push_back(d);
        }
    }
    roadedge_datas_lines.push_back(od);
}

void HmapData::addLaneData(std::vector<LaneData> &datas)
{
    lane_datas = datas;
    lane_datas_lines.clear();
    if(datas.empty())
    {
        std::cerr << "lane输入的数据为空." << std::endl;
        return;
    }
    std::vector<LaneData> od;
    //划分每一条线
    int eid = datas.front().lane.id;
    for(auto d : datas)
    {
        if(d.lane.id != eid)
        {
            eid = d.lane.id;
            lane_datas_lines.push_back(od);
            od.clear();
            od.push_back(d);
        }
        else
        {
            od.push_back(d);
        }
    }
    lane_datas_lines.push_back(od);
}

void HmapData::addClearAreaData(std::vector<ClearAreaData> &datas)
{
    cleararea_datas = datas;
    cleararea_datas_lines.clear();
    if(datas.empty())
    {
        std::cerr << "lane输入的数据为空." << std::endl;
        return;
    }
    std::vector<ClearAreaData> od;
    //划分每一条线
    int eid = datas.front().cleararea.id;
    for(auto d : datas)
    {
        if(d.cleararea.id != eid)
        {
            eid = d.cleararea.id;
            cleararea_datas_lines.push_back(od);
            od.clear();
            od.push_back(d);
        }
        else
        {
            od.push_back(d);
        }
    }
    cleararea_datas_lines.push_back(od);
}

}
}