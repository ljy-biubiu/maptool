#include "test/attributetest.h"

namespace cti {
namespace test {

AttributeTest::AttributeTest()
{
}

AttributeTest::~AttributeTest()
{
}
bool AttributeTest::checkAttributeDatas(std::vector<std::vector<ClearAreaData>> &datas, AttributeResult &result)
{
    bool ret = true;
    AttributeResult::Result res;
    for(const auto data : datas)
    {

        uint32_t property = data.front().cleararea.property;
        for(size_t i = 1; i < data.size(); i++)
        {
            if(data[i].cleararea.property != property)
            {
                res.id = data[i].cleararea.id;
                res.type = "cleararea";
                res.err = "清扫区域属性不一致";
                result.results.push_back(res);
                ret = false;
                break;
            }
        }
    }
    return ret;
}

bool AttributeTest::startAttributeTest(HmapData *data, AttributeResult &result)
{
    checkAttributeDatas(data->cleararea_datas_lines,result);
    return result.results.size() == 0;
}

}
}