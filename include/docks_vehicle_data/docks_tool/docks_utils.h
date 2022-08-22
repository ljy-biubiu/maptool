#ifndef _DOCKS_UTILS_
#define _DOCKS_UTILS_

#include <stdio.h>
#include <string>
#include <Eigen/Dense>

struct MapDockInfo {
    std::string id; // dock id
    std::string name;
    std::string type;
    Eigen::Quaterniond q;
    Eigen::Vector3d post;

    // for map update
    double longitude; // east west
    double latitude;  // south north
    double altitude;
    double yaw; // radius
};

struct TokenInfo {
    std::string accessToken; // token
    std::string token_type;
    std::string scope;
    std::string refresh_token;
};

struct ParkInfo {
    std::string parkName;
    std::string parkId;
};

#endif