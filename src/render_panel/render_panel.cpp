#include "render_panel.h"

#include <OgreCamera.h>
#include <OgreSceneNode.h>

#include <QMouseEvent>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <cmath>
#include <sys/timeb.h>

using namespace rviz;

MyRenderPanel::MyRenderPanel(QWidget *parent)
    : QtOgreRenderWindow(parent),
      mouse_x_(0),
      mouse_y_(0),
      plane_mode_(false),
      work_mode_(VIEW)
{
  try
  {
    scene_manager_ = ogre_root_->createSceneManager(Ogre::ST_GENERIC);
    scene_node_ = scene_manager_->getRootSceneNode();
    arraw_scene_node_ = scene_node_->createChildSceneNode();
    arraw_c_scene_node_ = scene_node_->createChildSceneNode();
    map_scene_node_ = scene_node_->createChildSceneNode();
    obj_scene_node_ = scene_node_->createChildSceneNode();
    //--
    camera_base_ = new OrbitCamera(scene_manager_);
    camera_base_->setRelativeNode(scene_node_);
    camera_base_->getOgreCamera()->setNearClipDistance(0.01f);
    camera_base_->setPosition(0, 0, 50);
    camera_base_->lookAt(Ogre::Vector3(0, 0, 0));

    Ogre::Light *light = scene_manager_->createLight("MainDirectional");
    light->setType(Ogre::Light::LT_DIRECTIONAL);
    light->setDirection((-1)*Ogre::Vector3::UNIT_Z);
    light->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));
    //--
    setCamera(camera_base_->getOgreCamera());
    setBackgroundColor(Ogre::ColourValue(0.2, 0.2, 0.2));
    //--网格--
    grid_ = new Grid(scene_manager_,
                     NULL,
                     Grid::Lines,
                     10,
                     1.0f,
                     0.02,
                     Ogre::ColourValue(1.0f, 1.0f, 1.0f, 0.5f));
    grid_->getSceneNode()->setPosition(Ogre::Vector3(0, 0, 0));
    grid_->getSceneNode()->setOrientation(getQuaternionFromPlane(XY));

    //--
    arrow_ = new Arrow(scene_manager_, NULL, 0.5f, 0.1f, 0.2f, 0.3f);
    arrow_->setColor(1.0f, 0.0f, 0.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);
    //--
    setOrthographicView(false);
  }
  catch (Ogre::Exception &e)
  {
    printf("Fatal error: %s\n", e.what());
    exit(1);
  }
  connect(&render_timer_, SIGNAL(timeout()), this, SLOT(doRender()));
  render_timer_.start(33);
}

MyRenderPanel::~MyRenderPanel()
{
}

void MyRenderPanel::getRainbowColor(float value, Ogre::ColourValue &color)
{
  // this is HSV color palette with hue values going only from 0.0 to 0.833333.
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  float n = 1 - f;

  if (i <= 1)
    color[0] = n, color[1] = 0, color[2] = 1;
  else if (i == 2)
    color[0] = 0, color[1] = n, color[2] = 1;
  else if (i == 3)
    color[0] = 0, color[1] = 1, color[2] = n;
  else if (i == 4)
    color[0] = n, color[1] = 1, color[2] = 0;
  else if (i >= 5)
    color[0] = 1, color[1] = n, color[2] = 0;
}

void MyRenderPanel::doRender()
{
  ogre_root_->renderOneFrame();
  update();
}

void MyRenderPanel::update()
{
  fpsUpdate();
}
double MyRenderPanel::getTimeStamp()
{
  timeb t;
  ftime(&t);
  double time_stamp = t.millitm/1000.f;
  time_stamp += t.time;
  return time_stamp;
}
void MyRenderPanel::fpsUpdate()
{
  static int frame_count = 0;
  static double last_fps_calc_time = getTimeStamp();
  frame_count++;
  double calc_time = getTimeStamp() - last_fps_calc_time;
  if (calc_time > 1.0)
  {
    fps = frame_count / calc_time;
    frame_count = 0;
    last_fps_calc_time = getTimeStamp();
    //printf("fps:%d\n",(int)fps);
  }
}

//显示模式和画图模式
void MyRenderPanel::setWorkMode(WorkMode mode)
{
  work_mode_ = mode;
}
//显示模式2D/3D/T3D
void MyRenderPanel::setDispalyMode(DisplayMode mode)
{
  if(mode == VIEW_3D){
    plane_mode_ = false;
    setOrthographicView(false);
  }
  else if(mode == VIEW_2D){
    plane_mode_ = false;
    setOrthographicView(true);
  }
  else if(mode == VIEW_T3D){
    plane_mode_ = true;
    setOrthographicView(false);
  }
}

void MyRenderPanel::setCameraZero()
{
  camera_base_->setPosition(0, 0, 50);
  camera_base_->setOrientation(Ogre::Quaternion(Ogre::Radian(0), Ogre::Vector3::UNIT_Z));
  camera_base_->lookAt(Ogre::Vector3(0, 0, 0));
}

float MyRenderPanel::getPitch()
{
  return camera_base_->getPitch();
}
float MyRenderPanel::getYaw()
{
  return camera_base_->getYaw();
}
float MyRenderPanel::getDistance()
{
  return camera_base_->getDistance();
}
const Ogre::Vector3& MyRenderPanel::getFocalPoint()
{
  return camera_base_->getFocalPoint();
}

void MyRenderPanel::setOrthographicView(bool enable)
{
  OrthoCameraView param;
  ortho_param_ = param;
  ortho_param_.ortho_view = enable;
  camera_base_->setOrthographicView(ortho_param_.ortho_view);
  if(ortho_param_.ortho_view){
    setCameraZero();
    updateCamera();
  }
  else{
    setCameraZero();
    camera_base_->getOgreCamera()->setCustomProjectionMatrix(false,Ogre::Matrix4::IDENTITY);
  }
}

Ogre::Quaternion MyRenderPanel::getQuaternionFromPlane(const Plane mode)
{
  Ogre::Quaternion orient;
  switch (mode)
  {
  case XZ:
    orient = Ogre::Quaternion(1, 0, 0, 0);
    break;
  case YZ:
    orient = Ogre::Quaternion(Ogre::Vector3(0, -1, 0), Ogre::Vector3(0, 0, 1),
                              Ogre::Vector3(1, 0, 0));
    break;
  case XY:
  default:
    orient = Ogre::Quaternion(Ogre::Vector3(1, 0, 0), Ogre::Vector3(0, 0, -1),
                              Ogre::Vector3(0, 1, 0));
    break;
  }
  return orient;
}

bool MyRenderPanel::getPointOnPlaneFromWindowXY(Ogre::Viewport *viewport,
                                          Ogre::Plane &plane, int window_x,
                                          int window_y,
                                          Ogre::Vector3 &intersection_out)
{
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();

  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
      (float)window_x / (float)width, (float)window_y / (float)height);
  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(plane);
  if (!intersection.first)
  {
    return false;
  }
  intersection_out = mouse_ray.getPoint(intersection.second);
  return true;
}

void MyRenderPanel::onRenderWindowMouseEvents(QMouseEvent* event)
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;
  mouse_x_ = event->x();
  mouse_y_ = event->y();
  setFocus(Qt::MouseFocusReason);
  //--
  ViewportMouseEvent vme(getViewport(), event, last_x, last_y);
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    QWindow* window = this->windowHandle();
    if (window)
    {
        double pixel_ratio = window->devicePixelRatio();
        vme.x = static_cast<int>(pixel_ratio * vme.x);
        vme.y = static_cast<int>(pixel_ratio * vme.y);
        vme.last_x = static_cast<int>(pixel_ratio * vme.last_x);
        vme.last_y = static_cast<int>(pixel_ratio * vme.last_y);
    }
#endif
  handleMouseEvent(vme);
  event->accept();
}

void MyRenderPanel::handleMouseEvent(ViewportMouseEvent &event)
{
  if(event.leftDown() || event.rightDown())
  {
    Ogre::Vector3 intersection;
    Ogre::Plane plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if(getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, intersection))
    {
        pos_ = intersection;
        arrow_->setPosition(pos_);
    }
  }
  else if(event.type == QEvent::MouseMove)
  {
    int32_t diff_x = event.x - event.last_x;
    int32_t diff_y = event.y - event.last_y;

    if (event.left())
    {
      if(work_mode_ == DRAW)
      {
        Ogre::Vector3 cur_pos;
        Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
        if( getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, cur_pos))
        {
          float angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );
          arrow_->getSceneNode()->setVisible(true);
          //we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
          Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);
          arrow_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );
        }
      }
      else
      {
        if(plane_mode_ || ortho_param_.ortho_view)
        {
           diff_y = 0;
        }
        camera_base_->mouseLeftDrag(diff_x, diff_y, event.control(), event.alt(), event.shift());
      }
    }
    else if (event.middle())
    {
      camera_base_->mouseMiddleDrag(diff_x, diff_y, event.control(), event.alt(), event.shift());
    }
    else if (event.right())
    {
      if(ortho_param_.ortho_view)
      {
        ortho_param_.scale = ortho_param_.scale + float(diff_y)/100.f;
        updateCamera();
      }
      else
      {
        camera_base_->mouseRightDrag(diff_x, diff_y, event.control(), event.alt(), event.shift());
      }
    }
  }
  else if(event.leftUp())
  {
    Ogre::Vector3 cur_pos;
    Ogre::Plane plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if(getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, cur_pos))
    {
      float angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
      Q_EMIT mouseEventClicked(pos_.x, pos_.y, angle);
    }
    if(work_mode_ == DRAW)
    {
      arrow_->getSceneNode()->setVisible(false);
    }
  }
  else if(event.rightUp())
  {
    Ogre::Vector3 cur_pos;
    Ogre::Plane plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if(getPointOnPlaneFromWindowXY(event.viewport, plane, event.x, event.y, cur_pos))
    {
      float angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
      Q_EMIT rightmouseEventClicked(pos_.x, pos_.y, angle, event.x, event.y);
    }
  }
}

void MyRenderPanel::wheelEvent(QWheelEvent *event)
{
  if (event->delta() != 0)
  {
    if(ortho_param_.ortho_view)
    {
      ortho_param_.scale = ortho_param_.scale * (1 + float(event->delta()) / 1000);
      updateCamera();
    }
    else
    {
      bool cmd = event->modifiers() & Qt::ControlModifier;
      bool shift = event->modifiers() & Qt::ShiftModifier;
      bool alt = event->modifiers() & Qt::AltModifier;
      camera_base_->scrollWheel(event->delta(), cmd, alt, shift);
      camera_base_->getOgreCamera()->setCustomProjectionMatrix(false,Ogre::Matrix4::IDENTITY);
    }
  }
}

void MyRenderPanel::updateCamera()
{
  float width = camera_->getViewport()->getActualWidth();
  float height = camera_->getViewport()->getActualHeight();

  float scale = ortho_param_.scale;
  Ogre::Matrix4 proj;
  buildScaledOrthoMatrix(proj, -width/scale/2, width/scale/2, -height/scale/2, height/scale/2,
                         camera_base_->getOgreCamera()->getNearClipDistance(),
                         camera_base_->getOgreCamera()->getFarClipDistance());
  camera_base_->getOgreCamera()->setCustomProjectionMatrix(true, proj);
}

void MyRenderPanel::keyPressEvent(QKeyEvent *event)
{
  event->accept();
}

//-----------------------------DISPLAY---------------------------------
//颜色
void MyRenderPanel::setGridColor(const Ogre::ColourValue& color)
{
  grid_->setColor(color);
}

Ogre::ColourValue MyRenderPanel::getGridColor()
{
  return grid_->getColor();
}

//数量
void MyRenderPanel::setGridCellCount(uint32_t count)
{
  grid_->setCellCount(count);
}

float MyRenderPanel::getGridCellCount()
{
  return grid_->getCellCount();
}

//长度
void MyRenderPanel::setGridCellLength(float len)
{
  grid_->setCellLength(len);
}

float MyRenderPanel::getGridCellLength()
{
  return grid_->getCellLength();
}

//线宽
void MyRenderPanel::setGridLineWidth(float width)
{
  grid_->setLineWidth(width);
}

float MyRenderPanel::getGridLineWidth()
{
  return grid_->getLineWidth();
}
//颜色强度并显示
void MyRenderPanel::setIntensityModeDisplay(bool mode, double min_value, double max_value, std::string id)
{
  setIntensityMode(mode,min_value,max_value);
  if(!id.empty())
  {
    if(cloudinfo_maps_.count(id) > 0)
    {
      addPointsDisplay(id,cloudinfo_maps_[id]->message);
    }
  }
  else
  {
    for(auto &map : cloudinfo_maps_)
    {
      addPointsDisplay(map.first,map.second->message);
    }
  }
}
//颜色强度
void MyRenderPanel::setIntensityMode(bool mode, double min_value, double max_value)
{
  extrenmum_mode_.intensity_mode = mode;
  if(extrenmum_mode_.intensity_mode)
  {
    extrenmum_mode_.min_intensity = min_value;
    extrenmum_mode_.max_intensity = max_value;
  }
  else
  {
    extrenmum_mode_.min_value = min_value;
    extrenmum_mode_.max_value = max_value;
  }
}

void MyRenderPanel::setPointsDimensions(const std::string id, float width, float height, float depth)
{
  if(cloudinfo_maps_.count(id) > 0)
  {
    cloudinfo_maps_[id]->cloud->setDimensions(width, height, depth);
  }
}

void MyRenderPanel::setPointsVisible(const std::string id, bool visible)
{
  if(cloudinfo_maps_.count(id) > 0)
  {
    cloudinfo_maps_[id]->scene_node->setVisible(visible);
  }
}

void MyRenderPanel::addPointsDisplay(const std::string id, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  if(!cloud)
  {
    std::cerr << "点云数据为空." << std::endl;
    return;
  }
  //--
  if(cloudinfo_maps_.count(id) <= 0)
  {
    cloudinfo_maps_[id] = new CloudInfo();
    cloudinfo_maps_[id]->cloud = std::make_shared<PointCloud>();
    cloudinfo_maps_[id]->scene_node = map_scene_node_->createChildSceneNode();
    cloudinfo_maps_[id]->message = cloud;

    cloudinfo_maps_[id]->cloud->setRenderMode(PointCloud::RM_FLAT_SQUARES);
    cloudinfo_maps_[id]->cloud->setAlpha(1.0f);
    cloudinfo_maps_[id]->cloud->setDimensions(0.05f, 0.05f, 0.05f);
    cloudinfo_maps_[id]->cloud->setHighlightColor(1, 1, 1);
  }
  else
  {
    cloudinfo_maps_[id]->cloud->clear();
  }

  float min_value_current, max_value_current;
  if(extrenmum_mode_.intensity_mode)
  {
      min_value_current = extrenmum_mode_.min_intensity;
      max_value_current = extrenmum_mode_.max_intensity;
  }
  else
  {
      min_value_current = extrenmum_mode_.min_value;
      max_value_current = extrenmum_mode_.max_value;
  }
  float range = max_value_current - min_value_current;
  if (range == 0)
  {
    range = 0.001f;
  }

  std::vector<PointCloud::Point> pointCloudPoint;
  for (auto mpoint : cloud->points)
  {
    PointCloud::Point point;
    if (std::isnan(mpoint.x) || std::isnan(mpoint.y) || std::isnan(mpoint.z) || std::isinf(mpoint.x) || std::isinf(mpoint.y) || std::isinf(mpoint.z))
    {
      std::cout << "nan point" << std::endl;
      mpoint.x = -999000;
      mpoint.y = -999000;
      mpoint.z = -999000;
    }
    else
    {
      point.position.x = mpoint.x;
      point.position.y = mpoint.y;
      point.position.z = mpoint.z;
      //--
      double extrenmum_val = extrenmum_mode_.intensity_mode ? mpoint.intensity : mpoint.z;
      float val = 1.0 - (extrenmum_val - min_value_current) / range;
      getRainbowColor(val, point.color);
      pointCloudPoint.push_back(point);
    }
  }
  cloudinfo_maps_[id]->cloud->addPoints(&pointCloudPoint.front(), pointCloudPoint.size());
  cloudinfo_maps_[id]->scene_node->detachAllObjects();
  cloudinfo_maps_[id]->scene_node->attachObject(cloudinfo_maps_[id]->cloud.get());
}

void MyRenderPanel::createArrowObject(const std::string id,
                                       const OrgePose &arrow,
                                       const float shaft_length,
                                       const float shaft_diameter,
                                       const float head_length,
                                       const float head_diameter,
                                       const Ogre::ColourValue color)
{
  if(arrow_maps_.count(id) <= 0){
    arrow_maps_[id] = new Arrow(scene_manager_, arraw_scene_node_, shaft_length, shaft_diameter, head_length, head_diameter);
  }
  arrow_maps_[id]->setColor(color);
  arrow_maps_[id]->setPosition(arrow.point);
  arrow_maps_[id]->setOrientation(arrow.quaternion);
}

void MyRenderPanel::createLineArrowObject(const std::string id,
                                          const OrgePose &arrow,
                                          const float shaft_length,
                                          const float shaft_diameter,
                                          const float head_length,
                                          const float head_diameter,
                                          const Ogre::ColourValue color)
{
  if(arrow_lines_.count(id) <= 0){
    arrow_lines_[id] = new Arrow(scene_manager_, arraw_c_scene_node_, shaft_length, shaft_diameter, head_length, head_diameter);
  }
  arrow_lines_[id]->setColor(color);
  arrow_lines_[id]->setPosition(arrow.point);
  arrow_lines_[id]->setOrientation(arrow.quaternion);
}

void MyRenderPanel::createPointsObject(const std::string id,
                                       const Shape::Type type,
                                       const OrgePoints &points,
                                       const float scale,
                                       const Ogre::ColourValue color)
{
  Shape *shape = NULL;
  if(points_maps_.count(id) <= 0)
  {
    for (auto &point : points)
    {
      shape = new Shape(type, scene_manager_, obj_scene_node_);
      shape->setPosition(point);
      shape->setScale(Ogre::Vector3(scale, scale, scale));
      shape->setColor(color);
      points_maps_[id].push_back(shape);
    }
  }
  else
  {
    size_t msize = points_maps_[id].size();
    size_t psize = points.size();
    size_t cnt = 0;
    for (auto &point : points)
    {
      if(msize > cnt)
      {
        points_maps_[id][cnt]->setPosition(point);
        points_maps_[id][cnt]->setScale(Ogre::Vector3(scale, scale, scale));
        points_maps_[id][cnt]->setColor(color);
      }
      else
      {
        shape = new Shape(type, scene_manager_, obj_scene_node_);
        shape->setPosition(point);
        shape->setScale(Ogre::Vector3(scale, scale, scale));
        shape->setColor(color);
        points_maps_[id].push_back(shape);
      }
      cnt++;
    }
    //--
    if(msize > psize)
    {
      for(size_t del_n = psize; del_n < msize; del_n++)
      {
        delete points_maps_[id][del_n];
      }
      points_maps_[id].erase(points_maps_[id].begin()+psize, points_maps_[id].begin()+msize);
    }
  }
}

void MyRenderPanel::createTextObject(const std::string id,
                                     const std::string text,
                                     const OrgePoint pos,
                                     const float scale,
                                     const Ogre::ColourValue color)
{
  if(textinfo_maps_.count(id) > 0){
    textinfo_maps_[id]->movable_text->setCaption(text);
    textinfo_maps_[id]->movable_text->setColor(color);
  }
  else{
    textinfo_maps_[id] = new MovableTextInfo();
    textinfo_maps_[id]->movable_text = std::make_shared<MovableText>(text);
    textinfo_maps_[id]->movable_text->setTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);
    textinfo_maps_[id]->scene_node = obj_scene_node_->createChildSceneNode();
  }
  textinfo_maps_[id]->scene_node->detachAllObjects();
  textinfo_maps_[id]->scene_node->attachObject(textinfo_maps_[id]->movable_text.get());
  textinfo_maps_[id]->scene_node->setPosition(pos);
  textinfo_maps_[id]->scene_node->setScale(Ogre::Vector3(scale, scale, scale));
}

void MyRenderPanel::createLineObject(const std::string id,
                                     const OrgePoints &line,
                                     const float line_width,
                                     const Ogre::ColourValue color)
{
  uint32_t points_per_line = line.size()+1;
  if(bline_maps_.count(id) <= 0){
    bline_maps_[id] = new BillboardLine(scene_manager_, obj_scene_node_);
  }
  else{
    bline_maps_[id]->clear();
  }
  bline_maps_[id]->setNumLines(1);
  bline_maps_[id]->setMaxPointsPerLine(points_per_line);
  bline_maps_[id]->setLineWidth(line_width);
  for (auto &data : line) {
    bline_maps_[id]->addPoint(data, color);
  }
}

void MyRenderPanel::createLinesObject(const std::string id,
                                      const std::vector<OrgePoints> &lines,
                                      const float line_width,
                                      const Ogre::ColourValue color)
{
  uint32_t points_per_line = 0;
  for(auto line : lines){
    if(line.size() > points_per_line){
      points_per_line = line.size();
    }
  }
  points_per_line++;
  std::string line_id = "line_" + id;
  if(bline_maps_.count(line_id) <= 0){
    bline_maps_[line_id] = new BillboardLine(scene_manager_, obj_scene_node_);
  }
  else{
    bline_maps_[line_id]->clear();
  }
  bline_maps_[line_id]->setNumLines(lines.size());
  bline_maps_[line_id]->setMaxPointsPerLine(points_per_line);
  bline_maps_[line_id]->setLineWidth(line_width);
  for (auto &line : lines) {
    for (auto &data : line){
       bline_maps_[line_id]->addPoint(data, color);
    }
    bline_maps_[line_id]->newLine();
  }
}

void MyRenderPanel::seceneMapVisible(bool visible)
{
  map_scene_node_->setVisible(visible);
}

void MyRenderPanel::seceneObjectVisible(bool visible)
{
  obj_scene_node_->setVisible(visible);
}

void MyRenderPanel::seceneDocksVisible(bool visible)
{
  arraw_scene_node_->setVisible(visible);
}

void MyRenderPanel::seceneMapRemove()
{
  for(auto &map : cloudinfo_maps_){
    map.second->cloud->clear();
    delete map.second;
  }
  cloudinfo_maps_.clear();
  map_scene_node_->removeAndDestroyAllChildren();
}

void MyRenderPanel::seceneObjectRemove()
{
  for(auto &bmap : bline_maps_){
    bmap.second->clear();
    delete bmap.second;
  }
  bline_maps_.clear();
  for(auto &pmap : points_maps_){
    for(auto &p : pmap.second){
      delete p;
    }
    pmap.second.clear();
  }
  points_maps_.clear();
  for(auto &tmap : textinfo_maps_){
    tmap.second->movable_text.reset();
    delete tmap.second;
  }
  textinfo_maps_.clear();
  obj_scene_node_->removeAndDestroyAllChildren();
}

void MyRenderPanel::seceneDocksRemove() {
  for(auto &dock : arrow_maps_){
    delete dock.second;
  }
  arrow_maps_.clear();
  arraw_scene_node_->removeAndDestroyAllChildren();
}

void MyRenderPanel::seceneClearareRemove() {
  for(auto &line : arrow_lines_){
    delete line.second;
  }
  arrow_lines_.clear();
  arraw_c_scene_node_->removeAndDestroyAllChildren();
}
