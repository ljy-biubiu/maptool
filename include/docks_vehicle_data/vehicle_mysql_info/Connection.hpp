#include "mysql/mysql.h"
// #include "MapInfo.h"
// #include "DatabaseOption.hpp"
#include <vector>
#include <string.h>
#include <iostream>
#include <set>
#include <mutex>

struct Stafix
{
    double latitude;
    double longitude;
    double altitude;
    double direction;
};
struct Point
{
    float pose_pos_x;
    float pose_pos_y;
    float pose_pos_z;
    float pose_orient_x;
    float pose_orient_y;
    float pose_orient_z;
    float pose_orient_w;
};
struct MapInfo
{
    std::string robot_id;
    std::string map_name;
    Stafix stafix;
    Point point;
    float battery_pow;
    std::string time;
    std::string nav_version;
};

struct MapData
{
    std::string projectName; //项目名
    std::string dataVersion; //数据版本
    std::string dataStr;     //数据路径
    std::string mapVersion;  //地图版本
    std::string mapStr;      //地图路径
    std::string rmapVersion; // rmap地图版本
    std::string rmapStr;     // rmap地图路径
    std::string vmapVersion; // vmap地图版本
    std::string vmapStr;     // vmap地图路径
};

/**
 * 数据库配置类
 */
struct DatabaseOption
{
    std::string host = "br.ctirobot.com"; //数据库ip地址  localhost  192.168.1.106
    // std::string host = "fe80::42:1dff:fe02:19b0";
    int port = 3306;                       //端口号    本机docker mysql为3307    3306
    std::string username = "lr_robot";     //用户名
    std::string password = "lr_robot888";  //数据库密码
    std::string databaseName = "lr_state"; //数据库名
    int connectTimeout = 0;                //连接的超时时间
    std::string charsetName = "utf8";      //编码
    int reconnectTime = 1;                 //超时重连次数
};

namespace cti
{
    namespace mapinfo_robot
    {
        class Connection
        {
        private:
            MYSQL *connection = nullptr; // mysql连接  是否需要初始化为 nullptr;
            MYSQL_RES *res;              //这个结构代表返回行的一个查询结果集
            MYSQL_ROW column;            //一个行数据的类型安全(type-safe)的表示，表示数据行的列
            char query[1024 * 2];        //查询语句
            std::string lastError;       //最后的错误信息
            DatabaseOption option;       //数据库配置信息
            MapInfo mapinfo;             //查询的数据 加进结构体 MapInfo
            int rows = 0;
            int fields = 0;

        public:
            void setOption(const DatabaseOption &option)
            { //数据库连接的初始化配置
                this->option = option;
            }

            void setLastError(const std::string &lastError)
            {
                this->lastError = "[" + lastError + "] " + mysql_error(connection);
                std::cerr << this->lastError << std::endl;
            }

            bool connect()
            {
                //初始化mysql
                connection = mysql_init(connection);
                if (nullptr == connection)
                {
                    setLastError("mysql init failed");
                    return false;
                }
                //设置超时时间
                if (option.connectTimeout > 0)
                {
                    if (0 != mysql_options(connection, MYSQL_OPT_CONNECT_TIMEOUT, &option.connectTimeout))
                    {
                        setLastError("set option error");
                        return false;
                    }
                }
                //编码
                mysql_options(connection, MYSQL_SET_CHARSET_NAME, option.charsetName.c_str());
                mysql_options(connection, MYSQL_OPT_RECONNECT, &option.reconnectTime);
                if (!mysql_real_connect(connection, option.host.c_str(), option.username.c_str(), option.password.c_str(),
                                        option.databaseName.c_str(), option.port, nullptr, 0))
                {
                    setLastError("failed to connect to database");
                    return false;
                }
                return true;
            }

            void FreeConnect()
            {
                mysql_free_result(res);  //释放一个结果集合使用的内存。
                mysql_close(connection); //关闭一个服务器连接。
            }

            void SelectData(const std::string &sql, MapInfo &mapinfo)
            { // 查询数据   返回查询值String类型 taskinfo
                strcpy(query, sql.c_str());
                mysql_query(connection, "set names gbk");
                if (!(mysql_query(connection, query)))
                {
                    printf("Query success\n");
                }
                else
                {
                    printf("Query failed:%s\n", mysql_error(connection));
                }
                if (!(res = mysql_store_result(connection)))
                {
                    printf("Could't get result：%s\n", mysql_error(connection));
                }
                while (column = mysql_fetch_row(res)) //输出 查询后的
                {
                    rows = mysql_num_rows(res);
                    fields = mysql_num_fields(res);

                    mapinfo.robot_id = column[2];
                    mapinfo.map_name = column[4];
                    mapinfo.stafix.latitude = std::stod(column[18]);
                    mapinfo.stafix.longitude = std::stod(column[19]);
                    mapinfo.stafix.altitude = std::stod(column[20]);
                    mapinfo.stafix.direction = std::stod(column[21]);
                    mapinfo.point.pose_pos_x = atof(column[22]);
                    mapinfo.point.pose_pos_y = atof(column[23]);
                    mapinfo.point.pose_pos_z = atof(column[24]);
                    mapinfo.point.pose_orient_x = atof(column[25]);
                    mapinfo.point.pose_orient_y = atof(column[26]);
                    mapinfo.point.pose_orient_z = atof(column[27]);
                    mapinfo.point.pose_orient_w = atof(column[28]);
                    mapinfo.battery_pow = atof(column[32]);
                    mapinfo.time = column[35];
                    mapinfo.nav_version = column[37];
                }
            }
            void SelectRobot(const std::string &sql, std::set<std::string> &set_robots)
            {
                strcpy(query, sql.c_str());
                mysql_query(connection, "set names gbk");
                if (!(mysql_query(connection, query)))
                {
                    printf("Query success\n");
                }
                else
                {
                    printf("Query failed:%s\n", mysql_error(connection));
                    return;
                }
                if (!(res = mysql_store_result(connection)))
                {
                    printf("Could't get result：%s\n", mysql_error(connection));
                    return;
                }
                while ((column = mysql_fetch_row(res)))
                {
                    rows = mysql_num_rows(res);
                    fields = mysql_num_fields(res);
                    std::string robots = column[0];
                    set_robots.insert(robots);
                }
                std::cout << "查询园区车辆数： " << set_robots.size() << std::endl;
                std::cout << "查询结果列数： " << fields << std::endl;
            }

            void SelectMapData(std::vector<MapData> &vec_mapdata) //查询map地图数据
            {
                strcpy(query, "SELECT * FROM map_state;");
                mysql_query(connection, "set names gbk");
                MapData mapdata;
                if (!(mysql_query(connection, query)))
                {
                    printf("SelectMapData success\n");
                }
                else
                {
                    printf("SelectMapData failed:%s\n", mysql_error(connection));
                    return;
                }

                if (!(res = mysql_store_result(connection)))
                {
                    printf("Could't get result：%s\n", mysql_error(connection));
                    return;
                }
                while ((column = mysql_fetch_row(res)))
                {
                    mapdata.projectName = column[1];
                    mapdata.dataVersion = column[2];
                    mapdata.dataStr = column[3];
                    mapdata.mapVersion = column[4];
                    mapdata.mapStr = column[5];
                    mapdata.rmapVersion = column[6];
                    mapdata.rmapStr = column[7];
                    mapdata.vmapVersion = column[8];
                    mapdata.vmapStr = column[9];
                    vec_mapdata.push_back(mapdata);
                };
            }

            bool InsertMapData(MapData &mapdata) //插入map地图存储数据
            {
                std::string projectName = mapdata.projectName;
                std::string dataVersion = mapdata.dataVersion;
                std::string dataStr = mapdata.dataStr;
                std::string mapVersion = mapdata.mapVersion;
                std::string mapStr = mapdata.mapStr;
                std::string rmapVersion = mapdata.rmapVersion;
                std::string rmapStr = mapdata.rmapStr;
                std::string vmapVersion = mapdata.vmapVersion;
                std::string vmapStr = mapdata.vmapStr;

                sprintf(query, "INSERT INTO map_state (projectName, dataVersion, \
                dataStr, mapVersion, mapStr, rmapVersion, rmapStr, vmapVersion, vmapStr, time) \
                VALUES('%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', now());",
                        projectName.c_str(),
                        dataVersion.c_str(),
                        dataStr.c_str(),
                        mapVersion.c_str(),
                        mapStr.c_str(),
                        rmapVersion.c_str(),
                        rmapStr.c_str(),
                        vmapVersion.c_str(),
                        vmapStr.c_str());

                mysql_query(connection, "set names gbk");
                if (!(mysql_query(connection, query)))
                {
                    printf("Insertdata success\n");
                    return true;
                }
                else
                {
                    printf("Insertdata failed:%s\n", mysql_error(connection));
                    return false;
                }
            }

            bool DeleteMapData(MapData &mapdata)   //删除地图数据
            {
                std::string projectName = mapdata.projectName;
                sprintf(query, "DELETE FROM map_state WHERE projectName='%s';", projectName.c_str());
                if (!(mysql_query(connection, query)))
                {
                    printf("deleteMapData success:\n");
                    return true;
                }
                else
                {
                    printf("deleteMapData failed:%s\n", mysql_error(connection));
                    return false;
                }
            }

            bool UpdateMapData(MapData &mapdata)    //修改地图数据
            {
                std::string projectName = mapdata.projectName;
                std::string dataVersion = mapdata.dataVersion;
                std::string dataStr = mapdata.dataStr;
                std::string mapVersion = mapdata.mapVersion;
                std::string mapStr = mapdata.mapStr;
                std::string rmapVersion = mapdata.rmapVersion;
                std::string rmapStr = mapdata.rmapStr;
                std::string vmapVersion = mapdata.vmapVersion;
                std::string vmapStr = mapdata.vmapStr;

                sprintf(query, "UPDATE map_state SET dataVersion = '%s',dataStr = '%s', \
                     mapVersion = '%s',  mapStr = '%s', rmapVersion = '%s', rmapStr = '%s',vmapVersion = '%s',  \
                     vmapStr = '%s' ,time = now() WHERE projectName = '%s'",
                        dataVersion.c_str(),
                        dataStr.c_str(),
                        mapVersion.c_str(),
                        mapStr.c_str(),
                        rmapVersion.c_str(),
                        rmapStr.c_str(),
                        vmapVersion.c_str(),
                        vmapStr.c_str(),
                        projectName.c_str());

                mysql_query(connection, "set names gbk");
                if (!(mysql_query(connection, query)))
                {
                    printf("Updatedata success\n");
                    return true;
                }
                else
                {
                    printf("Updatedata failed:%s\n", mysql_error(connection));
                    return false;
                }
            }
        };
    }
}