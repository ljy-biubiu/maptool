#ifndef MAPTOOL_H
#define MAPTOOL_H

#include <vector>
#include "Connection.hpp"
#include <mutex>
#include <QTime>

// namespace cti
// {
//     namespace  mapinfo_robot
//     {

class Maptool
{
public:
    cti::mapinfo_robot::Connection connection;
    MapInfo mapinfo;
    char map_sql[1024];
    // std::vector<std::string> vec_robot;
    std::string robot_id;
    std::set<std::string>::iterator it;
    MapData mapdata;

    void init_mysql()
    {
        //初始化 连接数据库
        connection.setOption(DatabaseOption()); //初始化配置
        connection.connect();                   //连接数据库
    }

    void close_mysql()
    { // 关闭数据库
        connection.FreeConnect();
    }

    // 通过车号获取车辆的全部信息
    void getMapinfo(std::string map_name, std::set<std::string> set_robots, std::vector<MapInfo> &vec_info)
    {
        std::unique_lock<std::mutex> lck(mutex_);
        for (it = set_robots.begin(); it != set_robots.end(); it++)
        {
            robot_id = *it;
            //通过车号索引查看信息
            sprintf(map_sql, "SELECT *from lr_state force index(id_index) where robot_id ='%s' order by record_id desc limit 1;", robot_id.c_str()); //只通过车号查询信息（不用园区名称）
            std::cout << "查询园区信息sql :" << map_sql << std::endl;
            connection.SelectData(map_sql, mapinfo);
            vec_info.push_back(mapinfo);
        }
        // for (int i = 0; i < set_robots.size(); i++)
        // {
        //     robot_id = set_robots.find(5);
        //     sprintf(map_sql, "SELECT *from lr_state where map_name ='%s' and robot_id ='%s' order by record_id desc limit 1;", map_name.c_str(), robot_id.c_str());
        //     std::cout << "查询园区信息sql :" << map_sql << std::endl;
        //     connection.SelectData(map_sql, mapinfo);
        //     vec_info.push_back(mapinfo);
        // }
    }

    // 通过园区名称获取整个园区的所有车辆
    void getRobotId(std::string map_name, std::set<std::string> &set_robots)
    {
        //强制通过园区索引查找车辆
        sprintf(map_sql, "SELECT robot_id  from lr_state force index(map_index) WHERE map_name ='%s' GROUP BY robot_id", map_name.c_str());
        std::cout << "查询园区车辆sql :" << map_sql << std::endl;
        connection.SelectRobot(map_sql, set_robots);
    }

    //  获取所有园区mapdata存储信息
    void getMapData(std::vector<MapData> &vec_mapdata)
    {
        vec_mapdata.clear();
        connection.SelectMapData(vec_mapdata);
        std::cout << "园区mapdata总数：" << vec_mapdata.size() << std::endl;
    }

    //   增加园区mapdata地图信息
    void savaMapData(MapData &mapdata)
    {
        if (connection.DeleteMapData(mapdata))
        {
            if (connection.InsertMapData(mapdata)) {
                std::cout << "增加" << mapdata.projectName << "地图成功" << std::endl;
            }
            else
            {
                std::cout << "增加" << mapdata.projectName << "地图失败" << std::endl;
            }
        }
    }
    
    //   删除园区地图mapdata信息
    void deleteMapData(MapData &mapdata)
    {
        connection.DeleteMapData(mapdata);
    }

    //   修改园区地图mapdata信息
    void updateMapData(MapData &mapdata)
    {
        if (connection.UpdateMapData(mapdata))
        {
            std::cout << "修改" << mapdata.projectName << "地图成功" << std::endl;
        }
        else
        {
            std::cout << "修改" << mapdata.projectName << "地图失败" << std::endl;
        }
    }

private:
    std::mutex mutex_;
};

//     }
// }

#endif // MAPTOOL_H