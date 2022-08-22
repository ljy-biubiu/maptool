#include <QApplication>
#include "render_points.h"
int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    //----------
    Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    pcl::PCLPointCloud2 pointcloud2;
    if (pcl::io::loadPCDFile("/home/cti/cti_map/wkyc/wkyc.pcd", pointcloud2, origin, orientation) < 0)
    {
        std::cout << "-----------------Open PCD File Failed!! Please Check file path !!----------------" << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromPCLPointCloud2(pointcloud2, *map);
    double map_init_lat = origin.x() + orientation.x();
    double map_init_lon = origin.y() + orientation.y();
    double map_init_alt = origin.z() + orientation.z();
    map_init_lat *= 1e-3;
    map_init_lon *= 1e-3;
    map_init_alt *= 1e-3;
    printf("map init lat %lf ,lon %lf , alt %lf", map_init_lat, map_init_lon, map_init_alt);
    std::cout << "size of the map cloud " << map->size() << std::endl;
    MyFrame frame;
    std::cout << "pt0" << std::endl;
    std::vector<point3f> points;
    points.push_back({-100.0, -100.0, 0.0});
    points.push_back({100.0, -100.0, 0.0});
    points.push_back({100.0, 100.0, 0.0});
    points.push_back({-100.0, 100.0, 0.0});
    points.push_back({-100.0, 100.0, 0.0});
    std::cout << "pt1" << std::endl;
    Ogre::ColourValue ColourValue(33 / 255, 22 / 255, 44 / 255);
    frame.scene_node_->createChildSceneNode()->attachObject(frame.LineObjectCreator(points, "1", ColourValue));
    std::cout << "pt2" << std::endl;
    frame.scene_manager_->destroyAllManualObjects();
    std::cout << "pt3" << std::endl;
    frame.scene_node_->createChildSceneNode()->attachObject(frame.PolygonObjectCreator(points, "1", ColourValue));
    std::cout << "pt4" << std::endl;
    frame.resize(800, 600);
    frame.setWindowTitle("cti map display");
    frame.addPointsDisplay(map);
    frame.show();
    return app.exec();
}
