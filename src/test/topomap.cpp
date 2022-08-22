#include "test/topomap.h"
#include "test/Dijkstra.h"

namespace cti{
namespace test{

bool sortFun(const std::pair<double, LaneData> &p1, std::pair<double, LaneData> &p2)
{
    return p1.first < p2.first;//升序排列
}

//--
Graph *g_ = NULL;

TopoMap::TopoMap()
{
    setOffsetForZ(-999.9, 999.9);
    lane_datas_.clear();
}

TopoMap::~TopoMap()
{
    if(g_){
        delete g_;
    }
}

void TopoMap::setOffsetForZ(const float min_offset, const float max_offset)
{
    min_offset_ = min_offset;
    max_offset_ = max_offset;
}

double TopoMap::computatDistance(const Pose &p1, const Pose &p2)
{
    return sqrt(pow((p2.x - p1.x), 2) + \
                pow((p2.y - p1.y), 2) + \
                pow((p2.z - p1.z), 2));
}

Eigen::Quaterniond TopoMap::euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
 
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    std::cout << "quaterniond x:" << q.x() << " y:" << q.y() << " z:" << q.z() << " w:" << q.w() << std::endl;
    return q;
}
 
Eigen::Vector3d TopoMap::quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    std::cout << "euler roll:" << euler[2] << " pitch:" << euler[1] << " yaw:" << euler[0] << std::endl;
    return euler;
}

std::vector<Pose> TopoMap::getDiscretizedPoints(const Pose &start, const Pose &end, const double interval)
{
    std::vector<Pose> points_;
    double d = computatDistance(start,end);
    double k = interval / d;
    points_.push_back(start);
    double limit = (1.0-2.0*k/3.0); //
    for(double i = k; i <= limit; i += k)
    {
        Pose pose;
        pose.x = start.x * (1-i) + end.x * (i);
        pose.y = start.y * (1-i) + end.y * (i);
        pose.z = start.z * (1-i) + end.z * (i);
        pose.ox = start.ox;
        pose.oy = start.oy;
        pose.oz = start.oz;
        pose.ow = start.ow;
        points_.push_back(pose);
    }
    return points_;
}

std::vector<Pose> TopoMap::getDiscretizedPointsMedian(const Pose &start, const Pose &end, const double interval)
{
    std::vector<Pose> points_;
    double d = computatDistance(start,end);
    //实际间隔个数
    unsigned int n = abs(d / interval);
    double k = 1.0f/(double)n;
    points_.push_back(start);
    for(unsigned int i=1; i < n; i ++)
    {
        Pose pose;
        pose.x = start.x * (1-i*k) + end.x * (i*k);
        pose.y = start.y * (1-i*k) + end.y * (i*k);
        pose.z = start.z * (1-i*k) + end.z * (i*k);
        pose.ox = start.ox;
        pose.oy = start.oy;
        pose.oz = start.oz;
        pose.ow = start.ow;
        points_.push_back(pose);
    }
    return points_;
}

void TopoMap::denseOriginalDatasMedian(std::vector<LaneData> &data, const double interval)
{
    std::vector<LaneData> data_buf;
    data_buf.clear();
    for(int i = 0; i < data.size()-1; i++)
    {
        std::vector<Pose> poses = getDiscretizedPointsMedian(data[i].lane.pose, data[i+1].lane.pose, interval);
        LaneData od;
        uint16_t j = 0, size = floor((float)poses.size()/2.0f);
        for(auto pose : poses)
        {
            if(j <= size)
            {
                od = data[i];
            }
            else
            {
                od = data[i+1];
            }
            od.lane.pose = pose;
            data_buf.push_back(od);
            j++;
        }
    }
    data_buf.push_back(data.back());
    //--clear
    data.clear();
    data = data_buf;
}
void TopoMap::denseOriginalDatasMedian(std::vector<std::vector<LaneData>> &datas, const double interval)
{
    std::vector<std::vector<LaneData>> datas_buf;
    datas_buf.clear();
    for(auto data : datas)
    {
        denseOriginalDatasMedian(data,interval);
        datas_buf.push_back(data);
    }
    //--clear
    for(auto data : datas)
    {
        data.clear();
    }
    datas = datas_buf;
}
void TopoMap::denseOriginalDatas(std::vector<LaneData> &data, const double interval)
{
    std::vector<LaneData> data_buf;
    data_buf.clear();
    for(int i = 0; i < data.size()-1; i++)
    {
        std::vector<Pose> poses = getDiscretizedPoints(data[i].lane.pose, data[i+1].lane.pose, interval);
        LaneData od;
        for(auto pose : poses)
        {
            od = data[i];
            od.lane.pose = pose;
            data_buf.push_back(od);
        }
    }
    data_buf.push_back(data.back());
    //--clear
    data.clear();
    data = data_buf;
}
void TopoMap::denseOriginalDatas(std::vector<std::vector<LaneData>> &datas, const double interval)
{
    std::vector<std::vector<LaneData>> datas_buf;
    datas_buf.clear();
    for(auto data : datas)
    {
        denseOriginalDatas(data,interval);
        datas_buf.push_back(data);
    }
    //--clear
    for(auto data : datas)
    {
        data.clear();
    }
    datas = datas_buf;
}

//---------------------------------------------

void TopoMap::createTopoMap(std::vector<LaneData> &datas)
{
    //保存一份原始数据
    lane_datas_ = datas;
    // if(g_){
    //     delete g_;
    //     g_ = NULL;
    // }
    std::cout << "lane node num:" << lane_datas_.size() << std::endl;
    g_ = new Graph(lane_datas_.size());
    //--
    std::vector<LaneData> od;
    //划分每一条线
    int eid = lane_datas_.front().lane.id;
    for(auto d : lane_datas_)
    {
        if(d.lane.id != eid)
        {
            eid = d.lane.id;
            lane_datas_lines_.push_back(od);
            od.clear();
            od.push_back(d);
        }
        else
        {
            od.push_back(d);
        }
    }
    lane_datas_lines_.push_back(od);
    //
    std::vector<TopoNode> toponode;
    createTopoNode(lane_datas_lines_,toponode);
    //添加到算法中
    addTopoMap(toponode);
    //密集数据
    denseOriginalDatasMedian(lane_datas_lines_,0.2);
}

void TopoMap::createTopoNode(std::vector<std::vector<LaneData>> &datas, std::vector<TopoNode> &toponode)
{
    //--获取首尾点
    std::vector<LaneData> lineHeads;
    for(auto ts : datas)
    {
        lineHeads.push_back(ts.front());
        lineHeads.push_back(ts.back());
    }
    std::cout << "head and tail num:" << lineHeads.size() << std::endl;
    //---
    laneHeads.clear();
    for(auto linehead : lineHeads)
    {
        bool linehead_node = false;
        for(auto oneroad : datas)
        {
            size_t size = oneroad.size();
            TopoNode node;
            for(size_t i = 0; i < size; i++)
            {
                //线头与线链接 默认双向
                if(oneroad.at(i).id != linehead.id)
                {
                    double dis = computatDistance(oneroad[i].lane.pose,linehead.lane.pose);
                    if(dis < SEARCH_MAX_DIS)
                    {
                        node.start_id = oneroad[i].id;
                        node.end_id = linehead.id;
                        node.distance = (fabs(dis) < DOUBLE_MIN)?DOUBLE_MIN:dis;
                        node.directed = false; //链接点都设置为无方向的
                        toponode.push_back(node);
                        transferIDs.push_back(node.start_id);
                        transferIDs.push_back(node.end_id);
                        linehead_node = true;
                    }
                }
                //一条线上的方向可自定义
                if((i+1) < size)
                {
                    double dis = computatDistance(oneroad[i].lane.pose,oneroad[i+1].lane.pose);
                    node.start_id = oneroad[i].id;
                    node.end_id = oneroad[i+1].id;
                    node.distance = (fabs(dis) < DOUBLE_MIN)?DOUBLE_MIN:dis;
                    PROPERTY_TEST test(oneroad[i].lane.property);
                    node.directed = (test.property_type.LANE&0x0f)==0?false:true;//通过道路属性来设置
                    toponode.push_back(node);
                }
            }
        }
        //线头非粘合点
        if(!linehead_node)
        {
            laneHeads.push_back(linehead);
        }
    }
    //--链接点集合 去除重复点--
    sort(transferIDs.begin(), transferIDs.end());
    std::vector<int>::iterator iter = unique(transferIDs.begin(), transferIDs.end());
    transferIDs.erase(iter, transferIDs.end());
}

void TopoMap::addRoadEdgeData(std::vector<RoadEdgeData> &datas)
{
    roadedge_datas_ = datas;
    std::vector<RoadEdgeData> od;
    //划分每一条线
    int eid = roadedge_datas_.front().roadedge.id;
    for(auto d : roadedge_datas_)
    {
        if(d.roadedge.id != eid)
        {
            eid = d.roadedge.id;
            roadedge_datas_lines_.push_back(od);
            od.clear();
            od.push_back(d);
        }
        else
        {
            od.push_back(d);
        }
    }
    roadedge_datas_lines_.push_back(od);
}
//打印所有距离
void TopoMap::printfTopoNode(std::vector<TopoNode> &toponode)
{
    for(auto tn : toponode)
    {
        std::cout << tn.start_id << "-->" << tn.end_id << " distance:" << tn.distance << std::endl;
    }
}

void TopoMap::addTopoMap(std::vector<TopoNode> &toponode)
{
    if(!g_){
        std::cerr << "dijkstra null error!" << std::endl;
        return;
    }
    for(auto tn : toponode){
        g_->insert_edge(tn.start_id, tn.end_id, tn.distance, tn.directed);
    }
    g_->print_Graph();
}

//返回id = -1 为无效点
LaneData TopoMap::findNearPointID(const Pose &pose)
{
    //距离最小
    double distance_min = DOUBLE_MAX;
    LaneData od;
    od.id = -1;
    for(auto ods_lines : lane_datas_lines_)
    {
        for(auto odl : ods_lines)
        {
            float max_limit = pose.z + max_offset_;
            float min_limit = pose.z + min_offset_;
            if(odl.lane.pose.z <= max_limit && odl.lane.pose.z >= min_limit)
            {
                double dis = computatDistance(pose,odl.lane.pose);
                //全局最小值
                if(dis < distance_min){
                    distance_min = dis;
                    od = odl;
                }
            }
        }
    }
    return od;
}
//获取临近最优点
LaneData TopoMap::findNearPointID(const Pose &pose, std::vector<LaneData> &ods, const float findRandge)
{
    std::vector<std::pair<double, LaneData>> ods_buff;
    ods_buff.clear();
    double g_dis_min = DOUBLE_MAX;
    LaneData g_od;
    for(auto ods_lines : lane_datas_lines_)
    {
        double l_dis_min = DOUBLE_MAX;
        double dis_pre = -1;
        double dis_tes = -1;
        double dis_nex = -1;
        LaneData od_tmp;
        LaneData l_od;
        std::vector<std::pair<double, LaneData>> foot_lods;
        for(auto od : ods_lines)
        {
            float max_limit = pose.z + max_offset_;
            float min_limit = pose.z + min_offset_;
            if(od.lane.pose.z <= max_limit && od.lane.pose.z >= min_limit)
            {
                dis_pre = dis_tes;
                dis_tes = dis_nex;
                dis_nex = computatDistance(pose,od.lane.pose);
                //线内垂点
                if(dis_tes < dis_pre && dis_tes < dis_nex)
                {
                    foot_lods.push_back(std::make_pair(dis_tes,od_tmp));
                }
                od_tmp = od;
                //线内最小距离值
                if(dis_nex <= l_dis_min)
                {
                    l_dis_min = dis_nex;
                    l_od = od;
                }
                //全局最小距离值
                if(dis_nex < g_dis_min)
                {
                    g_dis_min = dis_nex;
                    g_od = od;
                }
            }
        }
        if(l_dis_min < findRandge)
        {
            ods_buff.push_back(std::make_pair(l_dis_min, l_od));
        }
        for(auto foot_lod : foot_lods)
        {
            if(foot_lod.first < findRandge && foot_lod.second.id != l_od.id)
            {
                ods_buff.push_back(foot_lod);
            }
        }
    }
    std::sort(ods_buff.begin(), ods_buff.end(), sortFun);
    ods.clear();
    for(auto od_buf : ods_buff)
    {
        ods.push_back(od_buf.second);
    }
    return g_od;
}
//--计算是否相交--
bool TopoMap::isIntersect(const Pose &pose,const LaneData &od,const float findRandge)
{
    bool intersect_flag = false;
    float max_limit = pose.z + max_offset_;
    float min_limit = pose.z + min_offset_;
    for(auto roadedge_line : roadedge_datas_lines_)
    {
        for (size_t i = 1; i < roadedge_line.size(); i++)
        {
            Pose line_start = roadedge_line[i-1].roadedge.pose;
            Pose line_end   = roadedge_line[i].roadedge.pose;

            if (line_start.z <= max_limit && line_start.z >= min_limit)
            {
                double dis_x = fabs(line_start.x - pose.x);
                double dis_y = fabs(line_start.y - pose.y);
                if((dis_x == 0 && dis_y == 0) || !(dis_x <= findRandge && dis_y <= findRandge))
                {
                    continue;
                }
                LineCommon line1(LineCommon::LinePoint(pose.x,pose.y),
                                 LineCommon::LinePoint(od.lane.pose.x,od.lane.pose.y)),
                           line2(LineCommon::LinePoint(line_start.x,line_start.y),
                                 LineCommon::LinePoint(line_end.x,line_end.y));
                intersect_flag = line1.isIntersect(line2);
                if(intersect_flag)
                {
                    return intersect_flag;
                }
            }
        }
    }
    return intersect_flag;
}
//----------------------------------------------
//--查找最优点--
//0:正常-1:离主路太远-2:路径与路边线相交
int TopoMap::findBestPointID(const Pose &check_pose,
                             std::vector<LaneData> &ods,
                             const float findRandge)
{
    int ret = 0;
    //--检测最优点
    std::vector<LaneData> point_ods;
    LaneData best_od = findNearPointID(check_pose, point_ods, findRandge);
    if(point_ods.size() > 0)
    {
        //判断临近点是否相交
        for(auto od : point_ods)
        {
            if(!isIntersect(check_pose,od,findRandge))
            {
                ods.push_back(od);
            }
        }
        //--
        if (ods.size() == 0)
        {
            ret = -2; //路边线相交
            ods.push_back(best_od);//即使与边线相交也需要提供一个最优点
        }
    }
    else
    {
        ret = -1; //搜索点离主路太远
    }
    return ret;
}

//-----------------------------------------------
std::tuple<double, std::vector<LaneData>> TopoMap::find_path(uint32_t start_id, uint32_t end_id)
{
    double distance = -1;
    std::vector<uint32_t> path;
    path.clear();
    std::vector<LaneData> lanedata;
    lanedata.clear();
    if(g_)
    {
        std::tie(distance, path) = g_->dijkstra(start_id,end_id);
        for(auto p : path){
            lanedata.push_back(lane_datas_[p]);
        }
    }
    return std::make_tuple(distance, lanedata);
}

bool TopoMap::getOriginalDataFromID(uint32_t id, LaneData &odata)
{
    bool ret = false;
    if(lane_datas_.size() > id){
        odata = lane_datas_[id];
        ret = true;
    }
    return ret;
}

std::vector<LaneData> TopoMap::getLaneHead()
{
    return laneHeads;
}

bool TopoMap::isTransferID(int cur_id)
{
    bool ret = false;
    for(auto id : transferIDs){
        if(id == cur_id){
            ret = true;
            break;
        }
    }
    return ret;
}

}
}
