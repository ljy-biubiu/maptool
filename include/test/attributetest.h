#ifndef __ATTRIBUTETEST_H__
#define __ATTRIBUTETEST_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stack>
#include <stdio.h>
#include <cmath>

#include "test/hmapdata.h"

namespace cti {
namespace test {

struct AttributeResult
{
    struct Result
    {
        uint32_t id;
        std::string type;
        std::string err;
    };
    std::vector<Result> results;
};

class AttributeTest
{
public:
    AttributeTest();
    ~AttributeTest();
    bool startAttributeTest(HmapData *data, AttributeResult &result);
private:
    bool checkAttributeDatas(std::vector<std::vector<ClearAreaData>> &datas, AttributeResult &result);
};

}
}
#endif