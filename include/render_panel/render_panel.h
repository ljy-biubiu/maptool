#ifndef RENDER_POINTS_TEST_H
#define RENDER_POINTS_TEST_H

#include <QWidget>
#include <QTimer>
#include <unordered_map>

#ifndef Q_MOC_RUN
#include <OgreMesh.h>
#include <OgreMeshManager.h>
#include <OgreManualObject.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>
#include <OgreLight.h>
#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "ogre_helpers/qt_ogre_render_window.h"
#include "ogre_helpers/grid.h"
#include "ogre_helpers/orbit_camera.h"
#include "ogre_helpers/axes.h"
#include "ogre_helpers/shape.h"
#include "ogre_helpers/arrow.h"
#include "ogre_helpers/point_cloud.h"
#include "ogre_helpers/billboard_line.h"
#include "ogre_helpers/line.h"
#include "ogre_helpers/render_system.h"
#include "ogre_helpers/orthographic.h"
#include "ogre_helpers/movable_text.h"
#include "viewport_mouse_event.h"

#endif

using namespace rviz;
typedef Eigen::Matrix<float, 3, 1> point3f;
//--
typedef Ogre::Vector3 OrgePoint;
typedef std::vector<OrgePoint> OrgePoints;
typedef Ogre::Quaternion OgreQuaternion;
typedef struct{
  OrgePoint point;
  OgreQuaternion quaternion;
}OrgePose;
typedef std::vector<OrgePose> OrgePoses;

class MyRenderPanel : public QtOgreRenderWindow
{
  Q_OBJECT
public:
  enum Plane
  {
    XY,
    XZ,
    YZ,
  };
  enum WorkMode
  {
    VIEW = 0, //显示模式
    DRAW = 1, //画图模式
  };
  enum DisplayMode
  {
    VIEW_3D = 0,  //3D模式
    VIEW_2D = 1,  //2D模式
    VIEW_T3D = 2, //TOP3D模式
  };
  struct OrthoCameraView
  {
    bool  ortho_view = false;
    float scale = 10.0f;
    float angle = M_PI_2;
    float x = 0.0f;
    float y = 0.0f;
  };

  MyRenderPanel(QWidget *parent = 0);
  virtual ~MyRenderPanel();
  //设置点云显示模式
  void setIntensityMode(bool mode, double min_value, double max_value);
  void setIntensityModeDisplay(bool mode, double min_value, double max_value, std::string id="");
  //指定id点云可视化
  void setPointsVisible(const std::string id, bool visible);
  //添加点云显示
  void setPointsDimensions(const std::string id, float width, float height, float depth);
  void addPointsDisplay(const std::string id, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  //显示模式和画图模式
  void setWorkMode(WorkMode mode);
  //显示模式2D/3D/T3D
  void setDispalyMode(DisplayMode mode);
  //获取刷新率
  int  getFps() {return fps;}
  //时间戳
  double getTimeStamp();
  //回到零点位置
  void setCameraZero();
  //获取相机姿态
  float getPitch();
  float getYaw();
  float getDistance();
  const Ogre::Vector3& getFocalPoint();
  //物体删除
  void seceneObjectRemove(); //to do
  //地图删除
  void seceneMapRemove();
  //桩点删除
  void seceneDocksRemove();
  //吹扫方向删除
  void seceneClearareRemove();
  //物体显示/屏蔽
  void seceneObjectVisible(bool visible);
  //地图显示/屏蔽
  void seceneMapVisible(bool visible);
  //桩点显示/屏蔽
  void seceneDocksVisible(bool visible);
  //----------------Grid------------------
  //颜色
  void setGridColor(const Ogre::ColourValue& color);
  Ogre::ColourValue getGridColor();
  //数量
  void setGridCellCount(uint32_t count);
  float getGridCellCount();
  //长度
  void setGridCellLength(float len);
  float getGridCellLength();
  //线宽
  void setGridLineWidth(float width);
  float getGridLineWidth();
  //--------------------------------------
  //--边界线--
  void createLineArrowObject(const std::string id,
                             const OrgePose &arrow,
                             const float shaft_length,
                             const float shaft_diameter,
                             const float head_length,
                             const float head_diameter,
                             const Ogre::ColourValue color);
  //--桩点--
  void createArrowObject(const std::string id,
                         const OrgePose &arrow,
                         const float shaft_length,
                         const float shaft_diameter,
                         const float head_length,
                         const float head_diameter,
                         const Ogre::ColourValue color);
  //--点--
  void createPointsObject(const std::string id,
                          const Shape::Type type,
                          const OrgePoints &points,
                          const float scale,
                          const Ogre::ColourValue color);
  //--文本--
  void createTextObject(const std::string id,
                        const std::string text,
                        const OrgePoint pos,
                        const float scale,
                        const Ogre::ColourValue color);
  //--线--
  void createLineObject(const std::string id,
                        const OrgePoints &line,
                        const float line_width,
                        const Ogre::ColourValue color);
  //--多段线--
  void createLinesObject(const std::string id,
                         const std::vector<OrgePoints> &lines,
                         const float line_width,
                         const Ogre::ColourValue color);
protected Q_SLOTS:
  void doRender();
Q_SIGNALS:
  void rightmouseEventClicked(float, float, float, int, int);
  void mouseEventClicked(float, float, float);
protected:
  void update();
  void fpsUpdate();
  void updateCamera();
  void setOrthographicView(bool enable);
  void getRainbowColor(float value, Ogre::ColourValue &color);
  Ogre::Quaternion getQuaternionFromPlane(const Plane mode);
  bool getPointOnPlaneFromWindowXY(Ogre::Viewport *viewport,
                                   Ogre::Plane &plane,
                                   int window_x,
                                   int window_y,
                                   Ogre::Vector3 &intersection_out);
  // Called when any mouse event happens inside the render window
  void onRenderWindowMouseEvents(QMouseEvent* event );
  void handleMouseEvent(ViewportMouseEvent &event);
  virtual void mouseMoveEvent(QMouseEvent* event) { onRenderWindowMouseEvents(event); }
  virtual void mousePressEvent(QMouseEvent* event) { onRenderWindowMouseEvents(event); }
  virtual void mouseReleaseEvent(QMouseEvent* event) { onRenderWindowMouseEvents(event); }
  virtual void mouseDoubleClickEvent(QMouseEvent* event) { onRenderWindowMouseEvents(event); }
  /// Called when there is a mouse-wheel event.
  virtual void wheelEvent(QWheelEvent* event);
  virtual void keyPressEvent(QKeyEvent* event);
private:
  struct CloudInfo
  {
    Ogre::SceneNode *scene_node=NULL;
    std::shared_ptr<PointCloud> cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr message;
  };
  struct MovableTextInfo
  {
    Ogre::SceneNode *scene_node=NULL;
    std::shared_ptr<MovableText> movable_text;
  };
  struct ExtremumMode_{
    bool   intensity_mode{false};
    double min_value{-10};
    double max_value{20};
    double min_intensity{0};
    double max_intensity{255};
  };
  ExtremumMode_ extrenmum_mode_;
  //--
  CameraBase *camera_base_;
  PointCloud *pointCloud_;
  Grid *grid_;
  Arrow* arrow_;
  //--
  Ogre::SceneNode *scene_node_;
  Ogre::SceneNode *arraw_scene_node_;
  Ogre::SceneNode *arraw_c_scene_node_;
  Ogre::SceneNode *obj_scene_node_;
  Ogre::SceneNode *map_scene_node_;
  Ogre::SceneManager *scene_manager_;
  Ogre::Radian roll_yaw_ = Ogre::Radian(0);
  //模式
  bool plane_mode_;
  OrthoCameraView ortho_param_;
  WorkMode work_mode_;
  //鼠标位置
  Ogre::Vector3 pos_;
  int mouse_y_;
  int mouse_x_;
  //定时器
  QTimer render_timer_;
  float fps{30};
  //数据
  std::unordered_map<std::string, Arrow *> arrow_lines_;
  std::unordered_map<std::string, Arrow *> arrow_maps_;
  std::unordered_map<std::string, std::vector<Shape *>> points_maps_;
  std::unordered_map<std::string, MovableTextInfo *> textinfo_maps_;
  std::unordered_map<std::string, BillboardLine *> bline_maps_;
  std::unordered_map<std::string, CloudInfo *> cloudinfo_maps_;
};

#endif // RENDER_POINTS_TEST_H
