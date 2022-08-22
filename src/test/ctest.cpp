#include "test/ctest.h"

CTest::CTest()
{
    hdata = new HmapData();
}

CTest::~CTest()
{
    delete hdata;
}

void CTest::test(const std::string filepath)
{
    resulterrs_.clear();
    dataresults_.clear();
    clearattresults_.clear();

    RouteResult route_result;
    RouteTest route_test;
    std::string err_load;
    std::string resulterr;
    hdata->loadLaneFile(filepath + "/lane.csv", err_load);
    hdata->loadRoadEdgeFile(filepath + "/roadedge.csv", err_load);
    hdata->loadClearAreaFile(filepath + "/cleararea.csv", err_load);
    if(!route_test.startRouteTest(hdata,route_result))
    {
        std::cout << "检测结果：" << std::endl;
        for(auto res : route_result.results)
        {
            resulterr = "start(id:" + std::to_string(res.start_id) + \
                        ",x:" + std::to_string(res.start_pose.x) + \
                        ",y:" + std::to_string(res.start_pose.y) + \
                        ",z:" + std::to_string(res.start_pose.z) + ")->" + \
                        "end(id:"  + std::to_string(res.end_id) + \
                        ",x:" + std::to_string(res.end_pose.x) + \
                        ",y:" + std::to_string(res.end_pose.y) + \
                        ",z:" + std::to_string(res.end_pose.z) + \
                        ")" + res.err + ";\n";
            std::cout << resulterr << std::endl;
            resulterrs_.push_back(resulterr);
        }
    } else {
        resulterr = "路线可通,正常.";
        resulterrs_.push_back(resulterr);
    }
    DataReliableTest datareliable_test;
    DataReliableResult datareliable_result;
    std::string dataresult;
    if(!datareliable_test.startDataReliableTest(hdata,datareliable_result))
    {
        std::cout << "检测结果：" << std::endl;
        for(auto res : datareliable_result.results)
        {   
            dataresult = "id:" + std::to_string(res.id) + ", type:" + res.type + ", err:" + res.err + ";\n";
            dataresults_.push_back(dataresult);
            std::cout << dataresult << std::endl;
        }
    } else {
        dataresult = "数据正常.";
        dataresults_.push_back(dataresult);
    }
    //清扫区域属性
    AttributeTest attribute_test;
    AttributeResult attribute_result;
    std::string clearattresult;
    if(!attribute_test.startAttributeTest(hdata, attribute_result))
    {
        std::cout << "检测结果：" << std::endl;
        std::cout << "result size:" << attribute_result.results.size() << std::endl;
        for(auto res : attribute_result.results)
        {
            clearattresult = "id:" + std::to_string(res.id) + ", type:" + res.type + ", err:" + res.err + ";\n";
            clearattresults_.push_back(clearattresult);
            std::cout << clearattresult << std::endl;
            std::cout << "id:"  << std::endl;
        }
    } else {
        clearattresult = "区域属性正常.";
        clearattresults_.push_back(clearattresult);
    }
}