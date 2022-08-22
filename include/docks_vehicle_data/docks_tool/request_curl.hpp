#include <string>
#include <deque>
#include <math.h>
#include <iostream>

#include <curl/curl.h>
#include "json.hpp"
using json = nlohmann::json;

class RelocationSite
{
public:
    std::string name;
    double x;
    double y;
    double z;
    double yaw;

public:
    RelocationSite(std::string name, double x, double y, double z, double yawArg)
        : name(name), x(x), y(y), z(z)
    {
        double yaw_map = (yawArg < 270) ? (-yawArg + 90) : (450 - yawArg);

        while ((yaw_map >= 180) || (yaw_map < -180))
        {
            if (yaw_map >= 180)
                yaw_map -= 360;
            else if (yaw_map < -180)
                yaw_map += 360;
        }

        yaw = yaw_map * M_PI / 180;
    }

    void print()
    {
        std::cout << "name: " << name
                  << "\tx: " << x
                  << "\ty: " << y
                  << "\tz: " << z
                  << "\tyaw: " << yaw << std::endl;
    }
};

class Request
{
public:

    std::deque<RelocationSite> pullByName(std::string mapFileDirectory)
    {
        std::string mapBlockId = nameRequest(mapFileDirectory);
        std::cout << "mapBlockId: " << mapBlockId << std::endl;
        return pullById(mapBlockId);
    }

    std::deque<RelocationSite> pullById(std::string mapBlockId)
    {
        std::string resultString = siteRequest(mapBlockId);
        return siteParser(resultString);
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp)
    {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    // https://curl.se/libcurl/c/httpcustomheader.html
    // https://stackoverflow.com/questions/51540741/how-do-i-set-authorization-bearer-header-in-c-curl-code-im-getting-insuffici
    // https://stackoverflow.com/questions/9786150/save-curl-content-result-into-a-string-in-c/9786295
    std::string nameRequest(std::string mapFileDirectory)
    {
        std::string resultString;
        std::string mapBlockId;

        if(mapFileDirectory.empty()){
            return  mapBlockId;
        }

        std::string tocken = "Bearer eyJhbGciOiJIUzI1NiIsImNhbGciOiJHWklQIn0.H4sIAAAAAAAAAFXMQQqDMBBG4bvMOgOOmUjiFTxFMvkL6aJII7VFvLvtstvHxzuo9U4z3feNO56vZiBHLW80SxCJkw7RO8J7_QX1Osrgg6NHuf2LvLYFn-9JUs0RvrKhBFbFxNnSyBWarEQrFULnBWPus9N4AAAA.ziRA3yyD0E-IKlv7Ry-UN685J7s3RIDAZqZlnm-AczQ";
        std::string headers = "Authorization:" + tocken;
        std::string api = "https://api.ctirobot.com/api/map/v1/map-block?mapFileDirectory=" + mapFileDirectory;

        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl)
        {
            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());

            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);

            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            // std::cout << "[result]: " << resultString << std::endl;

            /* Check for errors */
            if (res != CURLE_OK)
            {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            }

            /* always cleanup */
            curl_easy_cleanup(curl);

            /* free the custom headers */
            curl_slist_free_all(chunk);
        }

        curl_global_cleanup();

        json jsonTmp;
        try
        {

            if(resultString.empty()){
                return  mapBlockId;
            }

            jsonTmp = json::parse(resultString);
            // std ::cout << jsonTmp.dump() << std ::endl;

            if (!jsonTmp.is_array())
            {
                return mapBlockId;
            }

            if (jsonTmp[0]["id"].is_string())
            {
                mapBlockId = jsonTmp[0]["id"];
                // std ::cout << "mapBlockId: " << mapBlockId << std ::endl;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return mapBlockId;
        }

        return mapBlockId;
    }

    // https://curl.se/libcurl/c/httpcustomheader.html
    // https://stackoverflow.com/questions/51540741/how-do-i-set-authorization-bearer-header-in-c-curl-code-im-getting-insuffici
    // https://stackoverflow.com/questions/9786150/save-curl-content-result-into-a-string-in-c/9786295
    std::string siteRequest(std::string mapBlockId)
    {
        std::string resultString;

        if(mapBlockId.empty()){
            return resultString;
        }

        std::string tocken = "Bearer eyJhbGciOiJIUzI1NiIsImNhbGciOiJHWklQIn0.H4sIAAAAAAAAAFXMQQqDMBBG4bvMOgOOmUjiFTxFMvkL6aJII7VFvLvtstvHxzuo9U4z3feNO56vZiBHLW80SxCJkw7RO8J7_QX1Osrgg6NHuf2LvLYFn-9JUs0RvrKhBFbFxNnSyBWarEQrFULnBWPus9N4AAAA.ziRA3yyD0E-IKlv7Ry-UN685J7s3RIDAZqZlnm-AczQ";
        std::string headers = "Authorization:" + tocken;
        std::string api = "https://api.ctirobot.com/api/map/v1/waypoint?page=0&pageSize=50000&deleted=false&indoor=false&relocatable=true&mapBlockId=" + mapBlockId;

        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl)
        {
            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());

            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);

            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            // std::cout << "[result]: " << resultString << std::endl;

            /* Check for errors */
            if (res != CURLE_OK)
            {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            }

            /* always cleanup */
            curl_easy_cleanup(curl);

            /* free the custom headers */
            curl_slist_free_all(chunk);
        }

        curl_global_cleanup();

        return resultString;
    }

    // https://github.com/nlohmann/json/
    std::deque<RelocationSite> siteParser(std::string jsonString)
    {
        std::deque<RelocationSite> siteDeque;

        if(jsonString.empty()){
            return  siteDeque;
        }

        json jsonTmp;
        try
        {
            jsonTmp = json::parse(jsonString);
            // std ::cout << jsonTmp.dump() << std ::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return siteDeque;
        }

        if (!jsonTmp.is_array())
        {
            return siteDeque;
        }

        for (int i = 0; i < jsonTmp.size(); i++)
        {
            std::string resultName;
            double resultLocationX, resultLocationY, resultAltitude, resultDirection;

            json ds = jsonTmp[i];

            // (1) name
            if (ds["name"].is_string())
            {
                resultName = ds["name"];
                // std ::cout << "resultName: " << resultName << std ::endl;
            }
            else
            {
                continue;
            }

            // (2) location
            if (ds["location"].is_array())
            {
                if (ds["location"][0].is_number())
                {
                    resultLocationX = ds["location"][0];
                    // std ::cout << "resultLocationX: " << resultLocationX << std ::endl;
                }
                else
                {
                    continue;
                }
                if (ds["location"][1].is_number())
                {
                    resultLocationY = ds["location"][1];
                    // std ::cout << "resultLocationY: " << resultLocationY << std ::endl;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

            // (3) altitude
            if (ds["altitude"].is_number())
            {
                resultAltitude = ds["altitude"];
                // std ::cout << "resultAltitude: " << resultAltitude << std ::endl;
            }
            else
            {
                continue;
            }

            // (4) direction
            if (ds["direction"].is_number())
            {
                resultDirection = ds["direction"];
                // std ::cout << "resultDirection: " << resultDirection << std ::endl;
            }
            else
            {
                continue;
            }

            RelocationSite relocationSite(resultName, resultLocationX, resultLocationY, resultAltitude, resultDirection);
            siteDeque.push_back(relocationSite);
        }

        return siteDeque;
    }
};


    // bool test_code(std::string dockid, double lat, double lon, double alt, double direction) {
    //     std::string domain = "api.ctirobot.com";
    //     // std::string dockid = "d9fe278e-9894-48ee-a65d-e04053aedb72";
    //     std::string uploadinfo;
    //     std::string api;

    //     {
    //         Json::Value item;
    //         // item["name"] = "园区/户外/基地";
    //         item["location"][0] = lat;     // 22.57001401666666;
    //         item["location"][1] = lon;     // 113.9440205833333;
    //         item["altitude"] = alt;        // 0.0;
    //         item["direction"] = direction; // 90.0;
    //         uploadinfo = item.toStyledString();
    //     }

    //     CURL *curl;
    //     CURLcode res;
    //     curl_global_init(CURL_GLOBAL_DEFAULT);
    //     curl = curl_easy_init();
    //     if (!curl) return false;
    //     api = "https://" + domain + "/api/map/v1/waypoint/" + dockid;
    //     std::string headers = "Authorization:" + authorization_token;

    //     struct curl_slist *chunk = NULL;
    //     chunk = curl_slist_append(chunk, headers.c_str());
    //     chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
    //     curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
    //     curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
    //     curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
    //     curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PATCH");
    //     curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, strlen(uploadinfo.c_str()));
    //     curl_easy_setopt(curl, CURLOPT_POSTFIELDS, uploadinfo.c_str());
    //     curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
    //     curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
    //     curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
    //     curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
    //     /* Perform the request, res will get the return code */
    //     res = curl_easy_perform(curl);
    //     // std::cout << uploadinfo << std::endl;
    //     /* Check for errors */
    //     if (res != CURLE_OK) {
    //         std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
    //     }
    //     /* always cleanup */
    //     curl_easy_cleanup(curl);
    //     /* free the custom headers */
    //     curl_slist_free_all(chunk);
    //     curl_global_cleanup();
    //     return true;
    // }
