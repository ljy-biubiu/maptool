#ifndef __DATARELIABLETEST_H__
#define __DATARELIABLETEST_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stack>
#include <stdio.h>
#include <cmath>

#include "test/hmapdata.h"
#include "test/linecommon.h"

#define POINTS_MAX_DIS (1000.f)

namespace cti {
namespace test {

struct DataReliableResult
{
    struct Result
    {
        uint32_t id;
        std::string type;
        std::string err;
    };
    std::vector<Result> results;
};

class DataReliableTest
{
public:
    DataReliableTest();
    ~DataReliableTest();
    bool startDataReliableTest(HmapData *data, DataReliableResult &result);
private:
    double computatDistance(const Pose &p1, const Pose &p2);
    bool checkDatas(std::vector<std::vector<LaneData>> &datas, DataReliableResult &result);
    bool checkDatas(std::vector<std::vector<RoadEdgeData>> &datas, DataReliableResult &result);
    bool checkDatas(std::vector<std::vector<ClearAreaData>> &datas, DataReliableResult &result);
    bool checkPolygonDatas(std::vector<std::vector<ClearAreaData>> &datas, DataReliableResult &result);
};

}
}
#endif