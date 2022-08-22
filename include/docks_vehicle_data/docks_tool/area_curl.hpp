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
#include <geo/geo.h>
#include "docks_utils.h"
#include "tool/sha1.hpp"
#include "tool/EncryptionRSA.h"
#include <sys/time.h>
#include <QDateTime>
#include <QList>
#include <QMap>

// using namespace Encrypt;

//普通用户
static std::string authorization_oridinary_token_ =
    "Bearer "
    "eyJhbGciOiJIUzI1NiIsImNhbGciOiJHWklQIn0."
    "H4sIAAAAAAAAAFXMQQqDMBBG4bvMOgOOmUjiFTxFMvkL6aJII7VFvLvtstvHxzuo9U4z3feNO5"
    "6vZiBHLW80SxCJkw7RO8J7_QX1Osrgg6NHuf2LvLYFn-"
    "9JUs0RvrKhBFbFxNnSyBWarEQrFULnBWPus9N4AAAA.ziRA3yyD0E-IKlv7Ry-"
    "UN685J7s3RIDAZqZlnm-AczQ";

//部署帐号--预发布
static std::string authorization_deploy_staging_token_ = 
    "Basic "
    "UFlpWGJtOWRpTUp5YnJoTjpZVnduZVFpTFQ0TFdQbW9j";

//部署帐号--生产
static std::string authorization_deploy_api_token_ = 
    "Basic "
    "UFlpWGJtOWRpTUp5YnJoTjptamtCQWFwdGs5UE1qaXlw";

//--部署token请求
class GetToken { //GET请求
public:
    GetToken() : domain_("staging.api.ctirobot.com") {
    }
    GetToken(std::string domain) : domain_(domain) {
    }
    void setRequestDomain(std::string domain) {
        domain_ = domain;
    }

    std::string getTokenString(std::string authorizationToken, std::string code) {
        DeloyToken_ = tokenGet(authorizationToken, code);
        return DeloyToken_;
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    // https://curl.se/libcurl/c/httpcustomheader.html
    // https://stackoverflow.com/questions/51540741/how-do-i-set-authorization-bearer-header-in-c-curl-code-im-getting-insuffici
    // https://stackoverflow.com/questions/9786150/save-curl-content-result-into-a-string-in-c/9786295
    std::string tokenGet(std::string authorizationToken, std::string code) {
        std::string resultString;
        std::string resultToken;

        std::string headers = "Authorization: " + authorizationToken;
        std::string api = "https://" + domain_ +
                          "/api/uaa/oauth/token?code=" + code + "&grant_type=authorization_code";
        std::cout << "api: " << api << std::endl;
        std::cout << "header:" << headers << std::endl;

        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl) {
            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            // chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);

            res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            }

            curl_easy_cleanup(curl);
            curl_slist_free_all(chunk);
        }

        curl_global_cleanup();

        Json::Reader Reader;
        Json::Value mapsId_res;

        try {
            std::cout << "enter..." << std::endl;
            if (resultString.empty()) { 
                std::cout << "empty..." << std::endl;
                return resultToken; 
            }
            std::cout << "result..." << resultString << std::endl;
            Reader.parse(resultString, mapsId_res);
            if (mapsId_res["access_token"].isString()) {
                resultToken = mapsId_res["access_token"].asString();
            }
            std::cout << "token: -- " << resultToken << std::endl;
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
            return resultToken;
        }

        return resultToken;
    }

private:
    std::string domain_;
    std::string DeloyToken_;
};

//--用户登录请求
class RequestToken { //POST请求
public:
    RequestToken() : domain_("staging.api.ctirobot.com") {
    }
    RequestToken(std::string domain) : domain_(domain) {
    }
    void setRequestDomain(std::string domain) {
        domain_ = domain;
        getToken_.setRequestDomain(domain_);
    }

    void setAuthorizationToken(std::string authorizationToken) {
        authorizationToken_ = authorizationToken;
        setApiStr(authorizationToken_);
    }

    std::string pullByUserPassword(std::string user, std::string password, std::string keyDir) {
        accessToken_ = tokenRequest(user,password,keyDir);
        // std::cout << "accessToken: " << accessToken_ << std::endl;
        return accessToken_;
    }

    void setApiStr(std::string authorizationToken) {
        if (authorizationToken == authorization_oridinary_token_) {
            apiStr_ = "/api/cleaning/v1/business/login/password";
            isDeply_ = false;
        } else {
            apiStr_ = "/api/saas/v1/platform/login/password";
            isDeply_ = true;
        }
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    // https://curl.se/libcurl/c/httpcustomheader.html
    // https://stackoverflow.com/questions/51540741/how-do-i-set-authorization-bearer-header-in-c-curl-code-im-getting-insuffici
    // https://stackoverflow.com/questions/9786150/save-curl-content-result-into-a-string-in-c/9786295
    std::string tokenRequest(std::string user, std::string password, std::string keyDir) {
        std::string resultString;
        std::string accessToken;
        std::string uploadUser;

        std::string passKey = passwordSHARSA(password);
        {
            Json::Value item;
            item["password"] = password;
            item["username"] = user;
            
            uploadUser = item.toStyledString();
        }
        if (user.empty() || password.empty()) { return accessToken; }
        std::cout << "usr: " << user << " pass: " << password << std::endl;

        std::string headers = "Authorization: " + authorizationToken_;
        std::string api = "https://" + domain_ + apiStr_;  ///api/cleaning
        std::cout << "api: " << api << std::endl;
        std::cout << "header:" << headers << std::endl;
        std::cout << "postfileds:" << uploadUser << std::endl;
        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl) {
            std::cout << "enter..." << std::endl;
            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, strlen(uploadUser.c_str()));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, uploadUser.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);
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
        std::cout << "[result]: " << resultString << std::endl;

        Json::Reader Reader;
        Json::Value mapsId_res;

        if (resultString.empty()) { 
            std::cout << "empty..." << std::endl;
            return accessToken; 
        }

        if (isDeply_)
        {
            std::string code;
            Reader.parse(resultString, mapsId_res);

            if (mapsId_res["code"].isString()) {
                code = mapsId_res["code"].asString();
            }
            accessToken = getToken_.getTokenString(authorizationToken_,code);
        } else {
            try {
                std::cout << "enter..." << std::endl;
                
                Reader.parse(resultString, mapsId_res);

                if (mapsId_res["access_token"].isString()) {
                    accessToken = mapsId_res["access_token"].asString();
                }
            } catch (const std::exception &e) {
                std::cerr << e.what() << '\n';
                return accessToken;
            }
        }
        return accessToken;
        
    }

    std::string passwordSHARSA(std::string pass) {
        const std::string input = pass;
        checksum_.update(input);
        const std::string hash = checksum_.final();
        std::cout << "The SHA-1 of \"" << input << "\" is: " << hash << std::endl;

        QDateTime current_date_time = QDateTime::currentDateTime();
        QString current_date = current_date_time.toString("yyyyMMddhhmmsszzz");
        // struct timeval time;
        // gettimeofday(&time, NULL);
        // char fmt[64] = "";
        // char timestampBuf[128] = "";
        // struct tm tm;
        
        // localtime_r(&time.tv_sec, &tm); // thread safe
        
        // strftime(fmt, sizeof(fmt), "%Y-%m-%d %H:%M:%S.%%06u", &tm);               //no time zone
        // sprintf(timestampBuf, fmt, time.tv_usec);

        // std::string strUSeconds = boost::lexical_cast<std::string>(time.tv_usec);
        std::string encryptpass = hash + ":" + current_date.toStdString();
        std::cout << "hasUs: " << encryptpass;
        // printf("s: %ld, ms: %ld\n", time.tv_sec, (time.tv_sec*1000 + time.tv_usec/1000));
        return encryptpass;
    }

private:
    std::string accessToken_;
    std::string authorizationToken_;
    std::string apiStr_;
    std::string domain_;
    bool isDeply_;
    SHA1 checksum_;
    Encrypt::EncryptionRSA rsastring_;
    GetToken getToken_;
};

//--园区信息请求
class RequestPark { //GET请求
public:
    RequestPark() : domain_("staging.api.ctirobot.com") {
    }
    RequestPark(std::string domain) : domain_(domain) {
    }
    void setRequestDomain(std::string domain) {
        domain_ = domain;
    }

    QList<QMap<QString, QString>> pullByToken(std::string authorizationToken) {
        listParksIdName_ = parkRequest(authorizationToken);
        return listParksIdName_;
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    // https://curl.se/libcurl/c/httpcustomheader.html
    // https://stackoverflow.com/questions/51540741/how-do-i-set-authorization-bearer-header-in-c-curl-code-im-getting-insuffici
    // https://stackoverflow.com/questions/9786150/save-curl-content-result-into-a-string-in-c/9786295
    QList<QMap<QString, QString>> parkRequest(std::string authorizationToken) {
        std::string resultString;

        std::string headers = "Authorization: Bearer " + authorizationToken;
        std::string api = "https://" + domain_ +
                          "/api/cleaning/v1/park?page=0&pageSize=1000";
        std::cout << "api: " << api << std::endl;
        std::cout << "header:" << headers << std::endl;

        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl) {
            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            // chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);

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


        Json::Reader Reader;
        Json::Value mapsId_res;
        QList<QMap<QString, QString>> listParksIdName;

        try {
            std::cout << "enter..." << std::endl;
            if (resultString.empty()) { 
                std::cout << "empty..." << std::endl;
                return listParksIdName; 
            }

            Reader.parse(resultString, mapsId_res);
            for (int i = 0; i < mapsId_res.size(); i++)
            {
                QMap<QString, QString> parksIdName;
                QString nameInfo = QString::fromStdString(mapsId_res[i]["name"].asString());
                QString idInfo = QString::fromStdString(mapsId_res[i]["id"].asString());
                parksIdName.insert(nameInfo,idInfo);
                listParksIdName.push_back(parksIdName);
            }
            

        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
            return listParksIdName;
        }

        return listParksIdName;
    }

private:
    std::string domain_;
    QList<QMap<QString, QString>> listParksIdName_;
};

//--初始点位创建
class PostDock { //POST请求
public:
    PostDock() : domain_("staging.api.ctirobot.com") {
    }
    PostDock(std::string domain) : domain_(domain) {
    }
    void setRequestDomain(std::string domain) { 
        domain_ = domain;
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    bool postToMap(std::string map_id, std::vector<double> uploadDockInfo, std::string authorizationToken) {
        std::string resultString;
        std::string uploadDock;
        {
            Json::Value item;
            // item["name"] = "园区/户外/基地";
            item["latitude"] = uploadDockInfo[0];
            item["longitude"] = uploadDockInfo[1];
            item["altitude"] = uploadDockInfo[2];
            item["direction"] = uploadDockInfo[3];
            item["indoor"] = false;
            // item["mapFileId"] = map_id;
            item["name"] = "建图初始点";
            item["parkId"] = map_id;
            item["waypointType"] = "HOME";
            uploadDock = item.toStyledString();
        }
        std::string api;
        std::string headers = "Authorization: Bearer " + authorizationToken;
        CURL *curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);

        {
            curl = curl_easy_init();
            if (!curl) return false;
            api = "https://" + domain_ + "/api/cleaning/v1/waypoint";
            std::cout << "***api:" << api << std::endl;
            std::cout << "***header:" << headers << std::endl;
            std::cout << "***postfileds" << uploadDock << std::endl;

            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, strlen(uploadDock.c_str()));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, uploadDock.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                return false;
            }
            /* always cleanup */
            curl_easy_cleanup(curl);
            /* free the custom headers */
            curl_slist_free_all(chunk);
        }
        curl_global_cleanup();
        std::cout << "post res:" << resultString << std::endl;
        return true;
    }
private:
    std::string domain_;
};

//--批量区域数据处理
class DealAllData { //POST DELETE PUT请求
public:
    DealAllData() : domain_("staging.api.ctirobot.com") {
    }
    DealAllData(std::string domain) : domain_(domain) {
    }
    void setRequestDomain(std::string domain) { 
        domain_ = domain;
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    //--数据批量删除(DELETE)
    bool deleteAllData(std::string authorizationToken, std::string dataType, std::string parkId) {
        std::string resultString;
        std::string api;
        std::string headers = "Authorization: Bearer " + authorizationToken;
        CURL *curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);

        {
            curl = curl_easy_init();
            if (!curl) return false;
            api = "https://" + domain_ + "/api/cleaning/v1/area/park/" + 
                  parkId + "/" + dataType;
            std::cout << "***api:" << api << std::endl;
            std::cout << "***header:" << headers << std::endl;
            // std::cout << "postfileds" << uploadData << std::endl;

            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8"); //type
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                return false;
            }
            /* always cleanup */
            curl_easy_cleanup(curl);
            /* free the custom headers */
            curl_slist_free_all(chunk);
        }
        curl_global_cleanup();
        std::cout << "[delete res]:" << resultString << std::endl;
        return true;
    }
    //--数据批量创建(POST)
    bool postUploadAllData(std::string authorizationToken, std::string uploadData) {
        std::string resultString;
        std::string api;
        std::string headers = "Authorization: Bearer " + authorizationToken;
        CURL *curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);

        {
            curl = curl_easy_init();
            if (!curl) return false;
            api = "https://" + domain_ + "/api/cleaning/v1/area";
            std::cout << "***api:" << api << std::endl;
            std::cout << "***header:" << headers << std::endl;
            // std::cout << "postfileds" << uploadData << std::endl;

            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8"); //type
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, strlen(uploadData.c_str()));
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, uploadData.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                return false;
            }
            /* always cleanup */
            curl_easy_cleanup(curl);
            /* free the custom headers */
            curl_slist_free_all(chunk);
        }
        curl_global_cleanup();
        std::cout << "[post res]:" << resultString << std::endl;
        return true;
    }
    //--批量修改区域,index存在则修改,不存在新增
    bool odinaryAllData(std::string authorizationToken, std::string uploadData, std::string parkId) {
        std::string resultString;
        std::string api;
        std::string headers = "Authorization: Bearer " + authorizationToken;
        CURL *curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);

        {
            curl = curl_easy_init();
            if (!curl) return false;
            api = "https://" + domain_ + "/api/cleaning/v1/area/park/" + parkId;
            std::cout << "***api:" << api << std::endl;
            std::cout << "***header:" << headers << std::endl;
            // std::cout << "postfileds" << uploadData << std::endl;

            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8"); //type
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, uploadData.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                return false;
            }
            /* always cleanup */
            curl_easy_cleanup(curl);
            /* free the custom headers */
            curl_slist_free_all(chunk);
        }
        curl_global_cleanup();
        std::cout << "[put res]:" << resultString << std::endl;
        return true;
    }

private:
    std::string domain_;
};

//--处理单个数据
class DealSignalData { //GET DELETE PUT请求
public:
    DealSignalData() : domain_("staging.api.ctirobot.com") {
    }
    DealSignalData(std::string domain) : domain_(domain) {
    }
    void setRequestDomain(std::string domain) { 
        domain_ = domain;
    }

    QList<QMap<int, QString>> getByParkId(std::string authorizationToken, std::string parkId, std::string dataType) {
        listDataSignalId_ = getSignalDataIdFromParkId(authorizationToken,parkId,dataType);
        // std::cout << "accessToken: " << accessToken_ << std::endl;
        return listDataSignalId_;
    }

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string *)userp)->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    //--查找区域.获取单数据id(GET)
    QList<QMap<int, QString>> getSignalDataIdFromParkId(std::string authorizationToken, std::string parkId, std::string dataType) {
        std::string resultString;
        std::string accessToken;

        std::string headers = "Authorization: Bearer " + authorizationToken;
        std::string api = "https://" + domain_ +
                          "/api/cleaning/v1/park/" +
                          parkId + "?areaType=" + dataType;
        std::cout << "api: " << api << std::endl;
        std::cout << "header:" << headers << std::endl;

        CURL *curl;
        CURLcode res;

        curl_global_init(CURL_GLOBAL_DEFAULT);

        curl = curl_easy_init();
        if (curl) {
            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            // chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);

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

        Json::Reader Reader;
        Json::Value datasId_res;
        QList<QMap<int, QString>> listDataSignalId;

        try {
            if (resultString.empty()) {
                std::cout << "empty..." << std::endl;
                return listDataSignalId; 
            }

            Reader.parse(resultString, datasId_res);
            for (int i = 0; i < datasId_res.size(); i++)
            {
                QMap<int, QString> parksIdName;
                int index = datasId_res[i]["index"].asInt();
                QString idInfo = QString::fromStdString(datasId_res[i]["id"].asString());
                parksIdName.insert(index,idInfo);
                listDataSignalId.push_back(parksIdName);
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
            return listDataSignalId;
        }
        return listDataSignalId;
    }
    //--根据单数据id,删除数据
    bool deletSignalDataFromId(std::string authorizationToken, std::string id) {
        std::string resultString;
        std::string api;
        std::string headers = "Authorization: Bearer " + authorizationToken;
        CURL *curl;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_DEFAULT);

        {
            curl = curl_easy_init();
            if (!curl) return false;
            api = "https://" + domain_ + "/api/cleaning/v1/area/" + id;
            std::cout << "***api:" << api << std::endl;
            std::cout << "***header:" << headers << std::endl;
            // std::cout << "postfileds" << uploadData << std::endl;

            struct curl_slist *chunk = NULL;
            chunk = curl_slist_append(chunk, headers.c_str());
            chunk = curl_slist_append(chunk, "Content-Type: application/json;charset=UTF-8"); //type
            curl_easy_setopt(curl, CURLOPT_HEADER, 0L);
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
            curl_easy_setopt(curl, CURLOPT_URL, api.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resultString);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
            curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
            /* Perform the request, res will get the return code */
            res = curl_easy_perform(curl);
            /* Check for errors */
            if (res != CURLE_OK) {
                std::cout << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                return false;
            }
            /* always cleanup */
            curl_easy_cleanup(curl);
            /* free the custom headers */
            curl_slist_free_all(chunk);
        }
        curl_global_cleanup();
        std::cout << "[delete res]:" << resultString << std::endl;
        return true;
    }

private:
    std::string domain_;
    QList<QMap<int, QString>> listDataSignalId_;
};

