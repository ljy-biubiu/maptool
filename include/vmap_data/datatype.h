#ifndef DATATYPE_H
#define DATATYPE_H

//所需要包含的头文件
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <OgreVector3.h>

#define FRAME_ID "/map"

enum Po_F : int {
  TimeStamp = 0,
  XF,
  YF,
  ZF,
  Qwf,
  Qxf,
  Qyf,
  Qzf,
  Po_Num
};

enum Vm_T : int {
  Lane = 0,       //主路线 + lanelet
  RoadEdge,       //路边线 + LineString(lanelet)
  CrossWalk,      //斑马线 + Crosswalk(lanelet)
  WaitLine,       //等待线 + StopLine
  Signal,         //信号灯 + SignLight
  StopPoint,      //停止点
  Junction,       //路口 + Intersection-poly
  DeceZone,       //减速带 + Bump
  SprayArea,      //喷洒区域
  ClearArea,      //清理区域
  Gate,           //门
  Elevator,       //电梯
  ConvergePoint,  //汇车点
  NodePoint,      //节点
  AttributeArea,   //障碍区域
  Type_Num
};

const std::string data_Type[Type_Num] = {
    "LANE",           //主路线
    "DRIVE",          //路边线
    "CROSSWALK",      //斑马线
    "WAIT_LINE",      //等待线
    "SIGNAL",         //信号灯
    "STOP_POINT",     //停止点
    "JUNCTION",       //路口
    "DECEZONE",       //路口特殊路边线
    "SPRAY_AREA",     //喷洒区域
    "CLEAN",          //清理区域
    "GATE",           //门
    "ELEVATOR",       //电梯
    "CONVERGE",       //汇车点
    "NODE_POINT",     //节点
    "ATTRIBUTE_AREA"  //障碍区域
};

const std::string type_name_str[Type_Num] = {
    "lane",           //主路线
    "roadedge",       //路边线
    "crosswalk",      //斑马线
    "waitline",       //等待线
    "signal",         //信号灯
    "stoppoint",      //停止点
    "junction",       //路口
    "decezone",       //路口特殊路边线
    "SprayArea",      //喷洒区域
    "cleararea",      //清理区域
    "gate",           //门
    "elevator",       //电梯
    "convergepoint",  //汇车点
    "nodepoint",      //节点
    "attributearea"    //障碍区域
};
enum RegulatoryElements : int { STOP = 0, LIMIT, RegulatoryElementsNUM };
const std::string regulatory_elements_str[RegulatoryElementsNUM] = {"STOP",
                                                                    "LIMIT"};
enum RegulatoryManeuver : int { cross = 0, merge, RegulatoryManeuverNUM };
const std::string regulatory_maneuver_str[RegulatoryManeuverNUM] = {"cross",
                                                                    "merge"};
enum RegulatoryRole : int {
  stop_line = 0,
  intersection,
  ref_lanelets,
  RegulatoryRoleNUM
};
const std::string regulatory_role_str[RegulatoryRoleNUM] = {
    "stop_line", "intersection", "ref_lanelets"};

struct satfix_ {
  double latitude;   //纬度
  double longitude;  //经度
  double altitude;   //高度
  double direction;  //方向
};

struct point_ {
  double x;  //坐标
  double y;
  double z;
  double quate_x;  //四元素
  double quate_y;
  double quate_z;
  double quate_w;
};

struct Way_Point_ {
  point_ point;
  satfix_ satfix;
  union {
    uint32_t property;
    struct {
      uint8_t P_TYPE;  //机动车leix
      int8_t P_PASS[3];
    } property_type1;
    struct {
      uint8_t P1_TYPE;  //机动车leix
      uint8_t P2_TRAV;
      uint8_t P3_PASS;
      uint8_t P4_MODE;
    } property_type;
  };
  union {
    uint32_t ltype;
    struct {
      uint16_t LINKTYPE;
      uint16_t LINKNUM;
    } ltype_type;
  };
  float limit_vel;
  uint32_t lanelet_id;
};

struct VectorMap_ {
  int type_id;      //类型 Vm_T
  int type_num_id;  //类型编号
  std::vector<Way_Point_> wp;
};

// P1_TYPE
struct PRO_LANE_TEST {
  union {
    uint8_t waytype;
    struct {
      uint8_t TYPE1 : 3;
      uint8_t TYPE2 : 4;
      uint8_t TYPE3 : 1;
    } pro_type;
  };
  PRO_LANE_TEST(uint8_t data) { waytype = data; }
  static uint8_t getProway(uint8_t type1, uint8_t type2, uint8_t type3) {
    PRO_LANE_TEST test(0);
    test.pro_type.TYPE1 = type1;
    test.pro_type.TYPE2 = type2;
    test.pro_type.TYPE3 = type3;
    return test.waytype;
  }
};
struct PRO_GATE_TEST {
  union {
    uint8_t waytype;
    struct {
      uint8_t TYPE1 : 4;
      uint8_t TYPE2 : 4;
    } pro_type;
  };
  PRO_GATE_TEST(uint8_t data) { waytype = data; }
  static uint8_t getProway(uint8_t type1, uint8_t type2) {
    PRO_GATE_TEST test(0);
    test.pro_type.TYPE1 = type1;
    test.pro_type.TYPE2 = type2;
    return test.waytype;
  }
};
struct PRO_SIGNAL_TEST {
  union {
    uint8_t waytype;
    struct {
      uint8_t TYPE1 : 4;
      uint8_t TYPE2 : 4;
    } pro_type;
  };
  PRO_SIGNAL_TEST(uint8_t data) { waytype = data; }
  static uint8_t getProway(uint8_t type1, uint8_t type2) {
    PRO_SIGNAL_TEST test(0);
    test.pro_type.TYPE1 = type1;
    test.pro_type.TYPE2 = type2;
    return test.waytype;
  }
};
struct PRO_DECEZONE_TEST {
  union {
    uint8_t waytype;
    struct {
      uint8_t TYPE1 : 8;
    } pro_type;
  };
  PRO_DECEZONE_TEST(uint8_t data) { waytype = data; }
  static uint8_t getProway(uint8_t type1) {
    PRO_DECEZONE_TEST test(0);
    test.pro_type.TYPE1 = type1;
    return test.waytype;
  }
};
struct PRO_STOPPOINT_TEST {
  union {
    uint8_t waytype;
    struct {
      uint8_t TYPE1 : 8;
    } pro_type;
  };
  PRO_STOPPOINT_TEST(uint8_t data) { waytype = data; }
  static uint8_t getProway(uint8_t type1) {
    PRO_STOPPOINT_TEST test(0);
    test.pro_type.TYPE1 = type1;
    return test.waytype;
  }
};
struct PRO_ELEVATOR_TEST {
  union {
    uint8_t waytype;
    struct {
      uint8_t TYPE1 : 8;
    } pro_type;
  };
  PRO_ELEVATOR_TEST(uint8_t data) { waytype = data; }
  static uint8_t getProway(uint8_t type1) {
    PRO_ELEVATOR_TEST test(0);
    test.pro_type.TYPE1 = type1;
    return test.waytype;
  }
};
struct PRO_CLEARAREA_TEST {
  union {
    uint8_t waytype;
    struct {
      uint8_t TYPE1 : 8;
    } pro_type;
  };
  PRO_CLEARAREA_TEST(uint8_t data) { waytype = data; }
  static uint8_t getProway(uint8_t type1) {
    PRO_CLEARAREA_TEST test(0);
    test.pro_type.TYPE1 = type1;
    return test.waytype;
  }
};
// struct PRO_OBSTACLEAREA_TEST {
//   union {
//     uint8_t waytype;
//     struct {
//       uint8_t TYPE1 : 8;
//     } pro_type;
//   };
//   PRO_OBSTACLEAREA_TEST(uint8_t data) { waytype = data; }
//   static uint8_t getProway(uint8_t type1) {
//     PRO_OBSTACLEAREA_TEST test(0);
//     test.pro_type.TYPE1 = type1;
//     return test.waytype;
//   }
// };
// P2_TRAV
struct TRAV_LANE_TEST {
  union {
    uint8_t travel;
    struct {
      uint8_t TYPE1 : 1;
      uint8_t TYPE2 : 1;
      uint8_t TYPE3 : 3;
      uint8_t TYPE4 : 3;
    } travel_type;
  };
  TRAV_LANE_TEST(uint8_t data) { travel = data; }
  static uint8_t getTravel(uint8_t type1, uint8_t type2, uint8_t type3,
                           uint8_t type4) {
    TRAV_LANE_TEST test(0);
    test.travel_type.TYPE1 = type1;
    test.travel_type.TYPE2 = type2;
    test.travel_type.TYPE3 = type3;
    test.travel_type.TYPE4 = type4;
    return test.travel;
  }
};

// P3_PASS
struct PASS_LANE_TEST {
  union {
    uint8_t pass;
    struct {
      uint8_t TYPE1 : 8;
    } pass_type;
  };
  PASS_LANE_TEST(uint8_t data) { pass = data; }
  static uint8_t getPass(uint8_t type1) {
    PASS_LANE_TEST test(0);
    test.pass_type.TYPE1 = type1;
    return test.pass;
  }
};
struct PASS_SIGNAL_TEST {
  union {
    uint8_t pass;
    struct {
      uint8_t TYPE1 : 8;
    } pass_type;
  };
  PASS_SIGNAL_TEST(uint8_t data) { pass = data; }
  static uint8_t getPass(uint8_t type1) {
    PASS_SIGNAL_TEST test(0);
    test.pass_type.TYPE1 = type1;
    return test.pass;
  }
};
struct PASS_GATE_TEST {
  union {
    uint8_t pass;
    struct {
      uint8_t TYPE1 : 8;
    } pass_type;
  };
  PASS_GATE_TEST(uint8_t data) { pass = data; }
  static uint8_t getPass(uint8_t type1) {
    PASS_GATE_TEST test(0);
    test.pass_type.TYPE1 = type1;
    return test.pass;
  }
};
struct PASS_ELEVATOR_TEST {
  union {
    int8_t pass[3];
    struct {
      int8_t TYPE1 : 8;
      int8_t TYPE2 : 8;
      int8_t TYPE3 : 8;
    } pass_type;
  };
  PASS_ELEVATOR_TEST(int8_t data[3]) {  // pass = data;
    pass[0] = data[0];
    pass[1] = data[1];
    pass[2] = data[2];
  }
  static std::string getPass(int8_t type1, int8_t type2, int8_t type3) {
    int8_t undata[3];
    PASS_ELEVATOR_TEST test(undata);
    test.pass_type.TYPE1 = type1;
    test.pass_type.TYPE2 = type2;
    test.pass_type.TYPE3 = type3;
    char c[3]="";
    c[0] = test.pass[0];
    c[1] = test.pass[1];
    c[2] = test.pass[2];
    std::string pass_str(c);
    return pass_str;
  }
};
/*-------------------------------------LANE类型属性连接关系------------------------------------*/
enum TYPE_P1_TYPE_LANE : uint8_t {
  TWO_LANE_TYPE = 0,
  ONE_LANE_TYPE = 1,
  VEHICLE_LANE_TYPE = 2,
  GARAGE_LANE_TYPE = 3,
  TYPE_P1_TYPE_LANE_NUM
};
const std::string TYPE_P1_TYPE_LANE_STR[TYPE_P1_TYPE_LANE_NUM] = {
    "园区双行道", "园区单行道", "机动车道", "车库车道"};
// enum TYPE_P1_TYPE1_LANE : uint8_t {
//   ONE_TYPE = 0,
//   TOW_TYPE = 1,
//   THREE_TYPE = 2,
//   FOUR_TYPE = 3,
//   TYPE_P1_TYPE1_LANE_NUM
// };
// const std::string TYPE_P1_TYPE1_LANE_STR[TYPE_P1_TYPE1_LANE_NUM] = {
//     "一车道",
//     "二车道",
//     "三车道",
//     "四车道",
// };
enum TYPE_P1_TYPE1_LANE : uint8_t {
  NORMAL1_TYPE = 0,
  TUNNEL_TYPE = 1,
  UNTUNNEL_TYPE = 2,
  WIDE_TYPE = 3,
  UNWIDE_TYPE = 4,
  TYPE_P1_TYPE1_LANE_NUM
};
const std::string TYPE_P1_TYPE1_LANE_STR[TYPE_P1_TYPE1_LANE_NUM] = {
    "正常",
    "隧道",
    "非隧道",
    "宽阔",
    "非宽阔",
};
enum TYPE_P1_TYPE2_LANE : uint8_t {
  OUTDOOR_TYPE = 0,
  INDOOR_TYPR = 1,
  TYPE_P1_TYPE2_LANE_NUM = 2
};
const std::string TYPE_P1_TYPE2_LANE_STR[TYPE_P1_TYPE2_LANE_NUM] = {
  "室外", "室内"};
  
/*8位数据，按位标识
 * 第0位:(0表示正常,1表示倒车)
 * 第2位1位:(00表示正常，01表示坡上，10表示坡下)
 * 第4位3位:(00表示正常,01表示路尽头(胡同),10表示窄路,11表示胡同&窄路)
 * 第5位6位7位:(000表示正常，001表示上坡度5，010表示上坡度10，011表示上坡度15，100表示坡中，101表示下坡度5，110表示下坡度10，111表示下坡度15)
 */
enum TYPE_P2_TRAV1_LANE : uint8_t {
  STRAIGHT_TYPE = 0,  //直行
  BACK_TYPE = 1,      //倒车
  TYPE_P2_TRAV1_LANE_NUM = 2
};
const std::string TYPE_P2_TRAV1_LANE_STR[TYPE_P2_TRAV1_LANE_NUM] = {"直行",
                                                                    "倒车"};

enum TYPE_P2_TRAV2_LANE : uint8_t {
  SMOOTH_TYPE = 0,
  // UPSLOPE_TYPE = 1,
  // DOWNSLOPE_TYPE = 2,
  SLOPE_TYPE = 1,
  TYPE_P2_TRAV2_LANE_NUM = 2
};
const std::string TYPE_P2_TRAV2_LANE_STR[TYPE_P2_TRAV2_LANE_NUM] = {
    "正常", "坡路"};
    // "正常", "坡上", "坡下"};

enum TYPE_P2_TRAV3_LANE : uint8_t {
  BROAD_TYPE = 0,
  ROADHEAD_TYPE = 1,
  NARROW_TYPE = 2,
  ROADHEAD_NARROW_TYPE = 3,
  TYPE_P2_TRAV3_LANE_NUM = 4
};
const std::string TYPE_P2_TRAV3_LANE_STR[TYPE_P2_TRAV3_LANE_NUM] = {
    "正常", "胡同", "窄路", "胡同＆窄路"};

enum TYPE_P2_TRAV4_LANE : uint8_t {
  NORMAL_TYPE = 0,
  UPSLOPE_TYPE05 = 1,
  UPSLOPE_TYPE10 = 2,
  UPSLOPE_TYPE15 = 3,
  INSLOPE_TYPE00 = 4,
  DOWNSLOPE_TYPE05 = 5,
  DOWNSLOPE_TYPE10 = 6,
  DOWNSLOPE_TYPE15 = 7,
  TYPE_P2_TRAV4_LANE_NUM = 8
};
const std::string TYPE_P2_TRAV4_LANE_STR[TYPE_P2_TRAV4_LANE_NUM] = {
    "正常", "上坡度5", "上坡度10", "上坡度15",
    "坡中", "下坡度5", "下坡度10", "下坡度15"};
//-------------
enum TYPE_P3_PASS_LANE : uint8_t {
  BYPASS_TYPE = 0,
  NOBYPASS_TYPE = 1,
  LIMITATION = 2,
  SAFE_TYPE = 3,
  Fix_TYPE = 4,
  CLOSELINE_TYPE = 5, //靠主路
  TYPE_P3_PASS_LANE_NUM = 6
};
const std::string TYPE_P3_PASS_LANE_STR[TYPE_P3_PASS_LANE_NUM] = {
    "绕障", "不绕障", "极限路", "安全路","固定路","靠主路"};
/*--GATE类型属性连接关系--*/
enum TYPE_P1_TYPE_GATE : uint8_t {
  DOOR_TYPE = 0,
  GATE_TYPE = 1,
  INDUCTION_GATE_TYPE = 2,
  MANUAL_GATE_TYPE = 3,
  TYPE_P1_TYPE_GATE_NUM
};
const std::string TYPE_P1_TYPE_GATE_STR[TYPE_P1_TYPE_GATE_NUM] = {
    "普通门", "闸机门", "感应门", "手动门"};
enum TYPE_P1_TYPE1_GATE : uint8_t {
  NUM0_GATE_TYPE = 0,
  NUM1_GATE_TYPE,
  NUM2_GATE_TYPE,
  NUM3_GATE_TYPE,
  NUM4_GATE_TYPE,
  NUM5_GATE_TYPE,
  NUM6_GATE_TYPE,
  NUM7_GATE_TYPE,
  NUM8_GATE_TYPE,
  NUM9_GATE_TYPE,
  NUM10_GATE_TYPE,
  NUM11_GATE_TYPE,
  NUM12_GATE_TYPE,
  NUM13_GATE_TYPE,
  NUM14_GATE_TYPE,
  TYPE_P1_TYPE1_GATE_NUM
};
const std::string TYPE_P1_TYPE1_GATE_STR[TYPE_P1_TYPE1_GATE_NUM] = {
    "0", "1", "2",  "3",  "4",  "5",  "6", "7",
    "8", "9", "10", "11", "12", "13", "14"};
enum TYPE_P3_PASS_GATE : uint8_t {
  NONRECOG_GATE_TYPE = 0,
  AUTO_GATE_TYPE = 1,
  REQUEST_GATE_TYPE = 2,
  TYPE_P3_PASS_GATE_NUM
};
const std::string TYPE_P3_PASS_GATE_STR[TYPE_P3_PASS_GATE_NUM] = {
    "不识别", "自识别", "请求识别"};

/*--SIGNAL类型属性连接关系--*/
enum TYPE_P1_TYPE_SIGNAL : uint8_t {
  PEOPLE_SIGNAL_TYPE = 0,
  VEHICLE_SIGNAL_TYPE = 1,
  TYPE_P1_TYPE_SIGNAL_NUM
};
const std::string TYPE_P1_TYPE_SIGNAL_STR[TYPE_P1_TYPE_SIGNAL_NUM] = {"人行灯",
                                                                      "车行灯"};
enum TYPE_P1_TYPE1_SIGNAL : uint8_t {
  NUM0_SIGNAL_TYPE = 0,
  NUM1_SIGNAL_TYPE,
  NUM2_SIGNAL_TYPE,
  NUM3_SIGNAL_TYPE,
  TYPE_P1_TYPE1_SIGNAL_NUM
};
const std::string TYPE_P1_TYPE1_SIGNAL_STR[TYPE_P1_TYPE1_SIGNAL_NUM] = {
    "0", "1", "2", "3"};
enum TYPE_P3_PASS_SIGNAL : uint8_t {
  NONRECOG_SIGNAL_TYPE = 0,
  AUTO_SIGNAL_TYPE = 1,
  REQUEST_SIGNAL_TYPE = 2,
  TYPE_P3_PASS_SIGNAL_NUM
};
const std::string TYPE_P3_PASS_SIGNAL_STR[TYPE_P3_PASS_SIGNAL_NUM] = {
    "不识别", "自识别", "请求识别"};
/*--DecelerationZone类型属性连接关系--*/
enum TYPE_P1_TYPE_DECEZONE : uint8_t {
  MOUND_DECEZONE_TYPE = 0,
  BAND_DECEZONE_TYPE,
  TYPE_P1_TYPE_DECEZONE_NUM
};
const std::string TYPE_P1_TYPE_DECEZONE_STR[TYPE_P1_TYPE_DECEZONE_NUM] = {
    "减速丘", "减速条"};

/*--STOPPOINT类型属性连接关系--*/
enum TYPE_P1_TYPE_STOPPOINT : uint8_t {
  STRAIGHT_STOPPOINT_TYPE = 0,
  TURNLEFT_STOPPOINT_TYPE,
  TURNRIGHT_STOPPOINT_TYPE,
  TURNBACK_STOPPOINT_TYPE,
  TYPE_P1_TYPE_STOPPOINT_NUM
};
const std::string TYPE_P1_TYPE_STOPPOINT_STR[TYPE_P1_TYPE_STOPPOINT_NUM] = {
    "直行", "左转", "右转", "掉头"};
/*--cleararea类型属性连接关系--*/
enum TYPE_P1_TYPE_CLEARAREA : uint8_t {
  CLEARAREA_TYPE = 0, //清扫区
  UNCLEARAREA_TYPE, //非清扫区
  CLEARLINE_TYPE, //清扫线
  PURGEBOUNDARY_TYPE, //吹扫边界线
  TYPE_P1_TYPE_CLEARAREA_NUM
};
const std::string TYPE_P1_TYPE_CLEARAREA_STR[TYPE_P1_TYPE_CLEARAREA_NUM] = {
    "清扫区", "非清扫区","清扫线","吹扫边界线"};
enum TYPE_NULLDATA : uint8_t { NULLDATA_TYPE = 0, TYPE_NULLDATA_NUM };
const std::string TYPE_NULLDATA_STR[TYPE_NULLDATA_NUM] = {"正常"};

#endif  // DATATYPE_H
