#include <string>


namespace cti{
    namespace mapinfo_robot{
/**
 * 数据库配置类
 */
struct DatabaseOption {
    std::string host = "br.ctirobot.com";//数据库ip地址  localhost  192.168.1.106
    // std::string host = "fe80::42:1dff:fe02:19b0";
    int port = 3306;//端口号    本机docker mysql为3307    3306
    std::string username = "lr_robot";//用户名 
    std::string password = "lr_robot888";//数据库密码
    std::string databaseName = "lr_state";//数据库名
    int connectTimeout = 0;//连接的超时时间
    std::string charsetName = "utf8";//编码
    int reconnectTime = 1;//超时重连次数
};

}}
