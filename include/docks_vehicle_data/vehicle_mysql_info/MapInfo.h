
#include <string>

namespace cti
{
    namespace mapinfo_robot
    {
        struct stafix
        {
            double latitude;
            double longitude;
            double altitude;
            double direction;
        };

        struct point
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

            stafix stafix;
            point point;
            float battery_pow;
            std::string time;
            std::string nav_version;
        };

    }
}