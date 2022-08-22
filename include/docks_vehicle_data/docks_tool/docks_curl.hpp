#include "iconv.h"
#include <deque>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <string> // to_string

#include "jsoncpp/json/json.h"
#include <curl/curl.h>
#include "docks_utils.h"
#include <geo/geo.h>

#ifndef DEGREE2RAD
#define DEGREE2RAD 0.0174532925199
#endif
#ifndef RAD2DEGREE
#define RAD2DEGREE 57.2957795130823
#endif

static std::string authorization_token =
    "Bearer "
    "eyJhbGciOiJIUzI1NiIsImNhbGciOiJHWklQIn0."
    "H4sIAAAAAAAAAFXMQQqDMBBG4bvMOgOOmUjiFTxFMvkL6aJII7VFvLvtstvHxzuo9U4z3feNO5"
    "6vZiBHLW80SxCJkw7RO8J7_QX1Osrgg6NHuf2LvLYFn-"
    "9JUs0RvrKhBFbFxNnSyBWarEQrFULnBWPus9N4AAAA.ziRA3yyD0E-IKlv7Ry-"
    "UN685J7s3RIDAZqZlnm-AczQ";

    // Bearer eyJhbGciOiJIUzI1NiIsImNhbGciOiJHWklQIn0.H4sIAAAAAAAAAFXMQQqDMBBG4bvMOgOOmUjiFTxFMvkL6aJII7VFvLvtstvHxzuo9U4z3feNO56vZiBHLW80SxCJkw7RO8J7_QX1Osrgg6NHuf2LvLYFn-9JUs0RvrKhBFbFxNnSyBWarEQrFULnBWPus9N4AAAA.ziRA3yyD0E-IKlv7Ry-UN685J7s3RIDAZqZlnm-AczQ

class RequestDocks {
public:
    RequestDocks() : domain_("api.ctirobot.com") {
        map_projection_init(&gnssZero_, 0., 0.); // , lat, lon
    }
    RequestDocks(std::string domain) : domain_(domain) {
        map_projection_init(&gnssZero_, 0., 0.);
    }
    void setRequestDomain(std::string domain) {
        domain_ = domain;
    }

    void setGNSSZero(double latitude, double longitude) {
        map_projection_init(&gnssZero_, latitude, longitude);
    }

    std::vector<MapDockInfo> pullByName(std::string mapFileDirectory) {
        mapBlockId_ = nameRequest(mapFileDirectory);
        std::cout << "mapBlockId: " << mapBlockId_ << std::endl;
        return pullById(mapBlockId_);
    }

    std::string turnBackBlockId() {
         return mapBlockId_;
    }

    std::vector<MapDockInfo> pullById(std::string mapBlockId) {
        std::vector<MapDockInfo> res;
        if (mapBlockId == "")
        {
            return res;
        }
        std::string api;
        {
            api = "https://" + domain_ +
                  "/api/map/v1/"
                  "waypoint?page=0&pageSize=50000&deleted=false&indoor=false&"
                  "mapBlockId=" +
                  mapBlockId;
            std::string resultString = siteRequest(api);
            // std::cout << "!!!!!!!!!!!!!!!!\n" << resultString << "      \n" << std::endl;
            std::vector<MapDockInfo> parse_res = siteParser(siteRequest(api));
            res.insert(res.end(), parse_res.begin(), parse_res.end());
            // std::cout << "raw res indoor false" << std::endl << resultString <<
            // std::endl;
        }
        for (int i = 1; i > -100; i--) {
            if (i == 0) continue;
            api = "https://" + domain_ +
                  "/api/map/v1/"
                  "waypoint?page=0&pageSize=50000&deleted=false&indoor=true&"
                  "floorName=" +
                  std::to_string(i) + "&mapBlockId=" + mapBlockId;
            std::string resultString = siteRequest(api);
            std::vector<MapDockInfo> parse_res = siteParser(siteRequest(api));
            // std::cout << "raw res floor " << i << std::endl << resultString <<
            // std::endl;
            if (parse_res.size() == 0) break;
            res.insert(res.end(), parse_res.begin(), parse_res.end());
        }

        return res;
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    // https://curl.se/libcurl/c/httpcustomheader.html
    // https://stackoverflow.com/questions/51540741/how-do-i-set-authorization-bearer-header-in-c-curl-code-im-getting-insuffici
    // https://stackoverflow.com/questions/9786150/save-curl-content-result-into-a-string-in-c/9786295
    std::string nameRequest(std::string mapFileDirectory) {
        std::string resultString;
        std::string mapBlockId;

        if (mapFileDirectory.empty()) { return mapBlockId; }

        std::string headers = "Authorization:" + authorization_token;
        std::string api =
            "https://api.ctirobot.com/api/map/v1/map-block?mapFileDirectory=" + mapFileDirectory;

        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl) {
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
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            }

            /* always cleanup */
            curl_easy_cleanup(curl);

            /* free the custom headers */
            curl_slist_free_all(chunk);
        }

        curl_global_cleanup();

        Json::Reader Reader;
        Json::Value mapsId_res;

        try {
            if (resultString.empty()) { return mapBlockId; }

            Reader.parse(resultString, mapsId_res);
            // std ::cout << jsonTmp.dump() << std ::endl;
            if (!mapsId_res.isArray()) { return mapBlockId; }

            if (mapsId_res[0]["id"].isString()) {
                mapBlockId = mapsId_res[0]["id"].asString();
                // std ::cout << "mapBlockId: " << mapBlockId << std ::endl;
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
            return mapBlockId;
        }

        return mapBlockId;
    }

    // https://curl.se/libcurl/c/httpcustomheader.html
    // https://stackoverflow.com/questions/51540741/how-do-i-set-authorization-bearer-header-in-c-curl-code-im-getting-insuffici
    // https://stackoverflow.com/questions/9786150/save-curl-content-result-into-a-string-in-c/9786295
    std::string siteRequest(std::string api) {
        std::string resultString;

        if (api.empty()) { return resultString; }

        std::string headers = "Authorization:" + authorization_token;

        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl) {
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
            if (res != CURLE_OK) {
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
    std::vector<MapDockInfo> siteParser(std::string jsonString) {
        std::vector<MapDockInfo> siteInfos;

        if (jsonString.empty()) { return siteInfos; }

        Json::Reader Reader;
        Json::Value docksVal;
        Reader.parse(jsonString, docksVal);

        if (!docksVal.isArray()) return siteInfos;

        for (int i = 0; i < docksVal.size(); i++) {
            Json::Value dockVal = docksVal[i];
            std::vector<std::string> dock_info;
            std::string resultName, resultId;
            double resultLat, resultLon, resultAltitude, resultDirection;
            if (!dockVal["id"].isString()) continue;
            if (!dockVal["name"].isString()) continue;
            if (dockVal["location"].isArray()) {
                if (dockVal["location"].size() != 2) continue;
                if (!dockVal["location"][0].isDouble()) continue;
                if (!dockVal["location"][1].isDouble()) continue;
            }
            else
                continue;
            if (!dockVal["altitude"].isDouble()) continue;
            if (!dockVal["direction"].isDouble()) continue;
            if (!dockVal["waypointType"].isString()) continue;
            MapDockInfo tmp_dock;
            tmp_dock.id = dockVal["id"].asString();
            tmp_dock.name = dockVal["name"].asString();
            tmp_dock.type = dockVal["waypointType"].asString();
            tmp_dock.latitude = dockVal["location"][0].asDouble();
            tmp_dock.longitude = dockVal["location"][1].asDouble();
            tmp_dock.altitude = dockVal["altitude"].asDouble();
            tmp_dock.post.z() = tmp_dock.altitude;
            double yaw_degree = dockVal["direction"].asDouble();
            double yaw_map = (yaw_degree < 270.) ? (-yaw_degree + 90.) : (450. - yaw_degree);
            yaw_map = fmod(yaw_map, 360.);
            if (yaw_map >= 180.)
                yaw_map -= 360.;
            else if (yaw_map < -180.)
                yaw_map += 360.;
            yaw_map *= DEGREE2RAD;
            tmp_dock.yaw = yaw_map;
            {
                Eigen::Matrix3d headM;
                headM = Eigen::AngleAxisd(yaw_map, Eigen::Vector3d::UnitZ());
                tmp_dock.q = headM;
            }
            {
                Eigen::Vector3d pos;
                map_projection_project(&gnssZero_, tmp_dock.latitude, tmp_dock.longitude,
                                       &tmp_dock.post.y(), &tmp_dock.post.x());
            }
            siteInfos.push_back(tmp_dock);
        }
        return siteInfos;
    }

private:
    std::string mapBlockId_;
    std::string domain_;
    map_projection_reference_s gnssZero_;
};

class UploadDocks {
public:
    UploadDocks() : domain_("api.ctirobot.com") {
    }
    UploadDocks(std::string domain) : domain_(domain) {
    }
    void setRequestDomain(std::string domain) {
        domain_ = domain;
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    bool uploadById(std::vector<std::pair<std::string, std::vector<double>>> uploadDockInfos) {
        std::vector<std::string> uploadDocks;
        for (int i = 0; i < uploadDockInfos.size(); i++) {
            Json::Value item;
            // item["name"] = "园区/户外/基地";
            item["location"][0] = uploadDockInfos[i].second[0];
            item["location"][1] = uploadDockInfos[i].second[1];
            item["altitude"] = uploadDockInfos[i].second[2];
            item["direction"] = uploadDockInfos[i].second[3];
            std::string uploadinfo = item.toStyledString();
            uploadDocks.push_back(uploadinfo);
        }
        std::string api;
        std::string headers = "Authorization:" + authorization_token;
        CURL *curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);

        for (int i = 0; i < uploadDockInfos.size(); i++) {
            curl = curl_easy_init();
            if (!curl) return false;
            api = "https://" + domain_ + "/api/map/v1/waypoint/" + uploadDockInfos[i].first;

            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PATCH");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, strlen(uploadDocks[i].c_str()));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, uploadDocks[i].c_str());
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            }
            /* always cleanup */
            curl_easy_cleanup(curl);
            /* free the custom headers */
            curl_slist_free_all(chunk);
        }
        curl_global_cleanup();
        return true;
    }

    bool uploadById(std::vector<MapDockInfo> uploadDockInfos) {
        std::vector<std::string> uploadDocks;
        for (int i = 0; i < uploadDockInfos.size(); i++) {
            Json::Value item;
            // item["name"] = "园区/户外/基地";
            item["location"][0] = uploadDockInfos[i].latitude;
            item["location"][1] = uploadDockInfos[i].longitude;
            item["altitude"] = uploadDockInfos[i].altitude;
            double yaw_map = uploadDockInfos[i].yaw;
            double update_yaw_deg = ((-yaw_map + M_PI * 0.5) < 0) ?
                                        (-yaw_map + 2.5 * M_PI) * RAD2DEGREE :
                                        (-yaw_map + M_PI * 0.5) * RAD2DEGREE;
            update_yaw_deg = fmod(update_yaw_deg, 360.);
            if (update_yaw_deg >= 180.)
                update_yaw_deg -= 360.;
            else if (update_yaw_deg < -180.)
                update_yaw_deg += 360.;
            item["direction"] = update_yaw_deg;
            std::string uploadinfo = item.toStyledString();
            uploadDocks.push_back(uploadinfo);
        }
        std::string api;
        std::string headers = "Authorization:" + authorization_token;
        CURL *curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);

        for (int i = 0; i < uploadDockInfos.size(); i++) {
            curl = curl_easy_init();
            if (!curl) return false;
            api = "https://" + domain_ + "/api/map/v1/waypoint/" + uploadDockInfos[i].id;

            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PATCH");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, strlen(uploadDocks[i].c_str()));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, uploadDocks[i].c_str());
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            }
            /* always cleanup */
            curl_easy_cleanup(curl);
            /* free the custom headers */
            curl_slist_free_all(chunk);
        }
        curl_global_cleanup();
        return true;
    }

private:
    std::string domain_;
};
